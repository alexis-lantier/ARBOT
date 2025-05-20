// Lib
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "motor_control.h"
#include <string.h>

// Configuration de l'UART pour le module ESP32-S3-DevKitC-1 v1.1
#define UART_TX (GPIO_NUM_43)               // TX pin
#define UART_RX (GPIO_NUM_44)               // RX pin
#define UART_RTS (UART_PIN_NO_CHANGE)       // RTS pin
#define UART_CTS (UART_PIN_NO_CHANGE)       // CTS pin

#define UART_PORT_NUM       (1)             // Utiliser UART1
#define UART_BAUD_RATE      (115200)        // Débit en bauds
#define UART_NUM            (UART_NUM_1)    // Numéro de l'UART
#define BUFFER_SIZE         (128)           // Taille du tampon de réception
#define TRAME_SIZE          (5)             // Taille de la trame attendue

// Configuration des temps (modifiable via des #define)
#define UART_READ_TIMEOUT_MS (10)   // Timeout pour la lecture UART en millisecondes
#define MOTOR_TASK_DELAY_MS  (10)  // Délai pour la tâche moteur en millisecondes

// Configuration de FreeRTOS
#define TASK_STACK_SIZE    (4096)   // Taille de la pile pour la tâche


// File de transmission des angles
static QueueHandle_t angle_queue;

// Définition de MOTSMAGIC
#define MOTSMAGIC               0x79 // 0x79 = 121 en decimal -> en dehors de la plage -120 à 120
#define MODE_ERROR              0x01 // Code d'erreur pour la trame
#define MODE_OK                 0x00 // Code OK pour la trame
#define MODE_MOTOR              0x02 // Code pour la trame moteur
#define CODE_ERROR_DEPASS       0x05 // Code d'erreur pour dépassement de trame
#define CODE_ERROR_CHECKSUM     0x04 // Code d'erreur pour checksum
#define CODE_ERROR_ANGLE        0x03 // Code d'erreur pour angle invalide
#define CODE_ERROR_FILE         0x02 // Code d'erreur pour file pleine
#define CODE_OK                 0x01 // Code OK pour trame valide

#define ANGLE_MAX 120 // Angle maximum pour les moteurs
#define ANGLE_MIN -120 // Angle minimum pour les moteurs

// Fonction pour checksum simple (somme modulo 256)
static uint8_t calculate_checksum(uint8_t *data, size_t len) {
    uint8_t checksum = 0;
    for (size_t i = 0; i < len; i++) {
        checksum += data[i];
    }
    return checksum; // Retourne le checksum modulo 256
}
// Fonction pour envoyer une trame d'erreur
static void send_uart_status_code(uint8_t mode, uint8_t code) {
    uint8_t error_frame[4];
    error_frame[0] = MOTSMAGIC;         // Identifiant de début de trame
    error_frame[1] = mode;              // Type de trame # erreur 0xE1
    error_frame[2] = code;              // Code d'erreur
    error_frame[3] = calculate_checksum(error_frame, 3); // Checksum des 3 premiers bytes

    // Envoyer la trame d'erreur via UART
    uart_write_bytes(UART_PORT_NUM, (const char *)error_frame, sizeof(error_frame));
}

// Tâche 1 : Lecture UART avec gestion de trame
static void uart_task(void *arg)
{
    uint8_t data[BUFFER_SIZE] = {0};    // Tampon de lecture UART
    uint8_t frame[TRAME_SIZE] = {0};    // Tampon de trame
    size_t frame_index = 0;             // Index d'écriture de la trame
    motor_angles_t motor_angles;        // Structure pour les angles des moteurs

    while (1) {
        // Lire les données reçues via l'UART
        int len = uart_read_bytes(UART_PORT_NUM, data, BUFFER_SIZE, UART_READ_TIMEOUT_MS / portTICK_PERIOD_MS);

        // Traiter les données reçues
        for (int i = 0; i < len; i++) {
            uint8_t byte = data[i];

            // Synchronisation sur le MOTSMAGIC
            if (frame_index == 0 && byte != MOTSMAGIC) {
                continue;
            }

            // Si un nouveau MOTSMAGIC arrive en cours de trame, resync
            if (frame_index > 1 && byte == MOTSMAGIC) {
                frame_index = 1;
                frame[0] = MOTSMAGIC;
                continue;
            }

            // Protection contre dépassement
            if (frame_index >= TRAME_SIZE){
                frame_index = 0;
                send_uart_status_code(MODE_ERROR, 0x05);
                continue;
            }

            // Ajouter le byte à la trame
            frame[frame_index++] = byte;

            // Si la trame est complète
            if (frame_index == TRAME_SIZE) {
                frame_index = 0; // Réinitialiser pour la prochaine trame

                // Vérification de la trame via le CRC
                uint8_t checksum = calculate_checksum(frame, 4);
                if (checksum == frame[TRAME_SIZE - 1]) {

                    // Extraction des angles
                    motor_angles.angle1 = (int8_t)frame[1];
                    motor_angles.angle2 = (int8_t)frame[2];
                    motor_angles.angle3 = (int8_t)frame[3];

                    // Validation des angles
                    if (motor_angles.angle1 >= ANGLE_MIN && motor_angles.angle1 <= ANGLE_MAX &&
                        motor_angles.angle2 >= ANGLE_MIN && motor_angles.angle2 <= ANGLE_MAX &&
                        motor_angles.angle3 >= ANGLE_MIN && motor_angles.angle3 <= ANGLE_MAX) {
                        
                        // Envoi de la trame valide    
                        send_uart_status_code(MODE_OK, CODE_OK);
                        
                        // Envoi des angles à la tâche moteur
                        if (xQueueSend(angle_queue, &motor_angles, 0) != pdPASS) {
                            send_uart_status_code(MODE_ERROR, CODE_ERROR_FILE); // File pleine
                        }

                    } else {
                        send_uart_status_code(MODE_ERROR, CODE_ERROR_ANGLE); // Angle(s) invalide(s)
                    }
                } else {
                    send_uart_status_code(MODE_ERROR, CODE_ERROR_CHECKSUM); // Mauvais checksum
                }
            }
        }
        // Delay pour éviter une surcharge CPU
        vTaskDelay(UART_READ_TIMEOUT_MS / portTICK_PERIOD_MS);
    }
}


// Tâche 2 : Commande du moteur
void motor_task(void *arg)
{
    motor_angles_t angles;
    int targets[3] = {0};
    int distances[MOTOR_COUNT] = {0};
    int totalSteps = 0;
    float counters[MOTOR_COUNT] = {0};

    while (1) {
        // Attendre la réception d'angles
        if (xQueueReceive(angle_queue, &angles, portMAX_DELAY) == pdPASS) {

            // Cpy les angles dans la cible
            targets[0] = angles.angle1;
            targets[1] = angles.angle2;
            targets[2] = angles.angle3;
            
            // Vérification des limites
            memset(distances, 0, sizeof(distances)); // Réinitialiser les distances
            totalSteps = 0;

            for (int i = 0; i < MOTOR_COUNT; ++i) {
                // Nombre de pas à faire par moteur
                motors[i].target = targets[i];

                // Direction du moteur
                motors[i].dir = (targets[i] > motors[i].position);

                // config GPIO pour la direction
                gpio_set_level(motors[i].dirPin, motors[i].dir ? 0 : 1);

                // Calculer la distance restante
                distances[i] = abs(targets[i] - motors[i].position);

                // Reglage la ramp selon la distance la plus grande
                if (distances[i] > totalSteps) totalSteps = distances[i];
            }

            // Déplacement des moteurs de manière synchrone
            memset(counters, 0, sizeof(counters));              // Réinitialiser les compteurs
            for (int step = 0; step < totalSteps; ++step) {
                for (int i = 0; i < MOTOR_COUNT; ++i) {
                    counters[i] += (float)distances[i] / totalSteps;
                    if (counters[i] >= 1.0f) {
                        motor_tick(&motors[i]);
                        counters[i] -= 1.0f;
                    }
                }
                esp_rom_delay_us(get_ramp_delay(step, totalSteps));
            }
        }
        vTaskDelay(MOTOR_TASK_DELAY_MS / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    // Initialisation de la file pour les angles
    angle_queue = xQueueCreate(20, sizeof(motor_angles_t));
    if (angle_queue == NULL) {
        printf("Erreur : Impossible de créer la file\n");
        return;
    }

    // Initialisation de l'UART
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_TX, UART_RX, UART_RTS, UART_CTS));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, BUFFER_SIZE * 2, 0, 0, NULL, 0));

    // Initialisation des moteurs
    motors_init();  // Init structure moteurs
    motors_begin(); // Init GPIOs moteurs

    // Création des tâches
    xTaskCreate(uart_task, "uart_task", TASK_STACK_SIZE, NULL, 5, NULL);
    xTaskCreate(motor_task, "motor_task", TASK_STACK_SIZE, NULL, 15, NULL);

    // Boucle principale (peut être vide si tout est géré par les tâches)
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000)); // Délai pour éviter une surcharge CPU
    }
}
