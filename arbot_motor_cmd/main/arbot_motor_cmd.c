// Lib
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include <string.h>
#include "motor_control.h"

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
#define UART_READ_TIMEOUT_MS (2)   // Timeout pour la lecture UART en millisecondes
#define MOTOR_TASK_DELAY_MS  (30)  // Délai pour la tâche moteur en millisecondes

// Configuration de FreeRTOS
#define TASK_STACK_SIZE    (2048)   // Taille de la pile pour la tâche


// File de transmission des angles
static QueueHandle_t angle_queue;

// Définition de MOTSMAGIC
#define MOTSMAGIC 0xAA // Exemple de valeur pour MOTSMAGIC

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
    error_frame[0] = MOTSMAGIC;          // Identifiant de début de trame
    error_frame[1] = mode;              // Type de trame # erreur 0xE1
    error_frame[2] = code;                              // Code d'erreur
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

            // Protection contre dépassement
            if (frame_index >= TRAME_SIZE){
                frame_index = 0;
                send_uart_status_code(0xE1, 0x05); // Trame trop longue
                continue;
            }

            // Ajouter le byte à la trame
            frame[frame_index++] = byte;

            // Si un nouveau MOTSMAGIC arrive en cours de trame, resync
            if (frame_index > 1 && byte == MOTSMAGIC) {
                frame_index = 1;
                frame[0] = MOTSMAGIC;
                continue;
            }

            // Si la trame est complète (5 octets attendus)
            if (frame_index == TRAME_SIZE) {
                frame_index = 0; // Réinitialiser pour la prochaine trame

                uint8_t checksum = calculate_checksum(frame, 4);
                if (checksum == frame[4]) {
                    int angle1 = (int8_t)frame[1];
                    int angle2 = (int8_t)frame[2];
                    int angle3 = (int8_t)frame[3];

                    // Validation des angles
                    if (angle1 >= -180 && angle1 <= 180 &&
                        angle2 >= -180 && angle2 <= 180 &&
                        angle3 >= -180 && angle3 <= 180) {

                        motor_angles_t angles = {angle1, angle2, angle3};

                        send_uart_status_code(0xA1, 0x01); // Trame valide

                        if (xQueueSend(angle_queue, &angles, 0) != pdPASS) {
                            send_uart_status_code(0xE1, 0x02); // File pleine
                        }

                    } else {
                        send_uart_status_code(0xE1, 0x03); // Angle(s) invalide(s)
                    }
                } else {
                    send_uart_status_code(0xE1, 0x04); // Mauvais checksum
                }
            }
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}


// Tâche 2 : Commande du moteur
void motor_task(void *arg)
{
    motor_angles_t angles;
    while (1) {
        // Attend une commande d'angles
        if (xQueueReceive(angle_queue, &angles, portMAX_DELAY) == pdPASS) {
            // Conversion des angles en pas
            int steps1 = angle_to_steps((float)angles.angle1);
            int steps2 = angle_to_steps((float)angles.angle2);
            int steps3 = angle_to_steps((float)angles.angle3);

            // Mouvement synchronisé des 3 moteurs
            int targets[3] = { steps1, steps2, steps3 };
            for (int i = 0; i < 3; ++i) {
                motors[i].target = targets[i];
                motors[i].dir = (motors[i].target > motors[i].position);
                gpio_set_level(motors[i].dirPin, motors[i].dir ? 0 : 1);
            }

            // Calcul du nombre total de pas à effectuer
            int totalSteps = 0;
            for (int i = 0; i < 3; ++i) {
                int steps = abs(motors[i].target - motors[i].position);
                if (steps > totalSteps) totalSteps = steps;
            }

            // Boucle de mouvement synchronisé
            for (int step = 0; step < totalSteps; ++step) {
                for (int i = 0; i < 3; ++i) {
                    if (motor_is_moving(&motors[i])) motor_tick(&motors[i]);
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
    angle_queue = xQueueCreate(10, sizeof(motor_angles_t)); // Taille de la file : 10 éléments
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
    motors_init();
    motors_begin();

    // Création des tâches
    xTaskCreate(uart_task, "uart_task", TASK_STACK_SIZE, NULL, 10, NULL);
    xTaskCreate(motor_task, "motor_task", TASK_STACK_SIZE, NULL, 10, NULL);
}
