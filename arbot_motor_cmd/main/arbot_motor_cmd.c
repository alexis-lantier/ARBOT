// Lib
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "motor_control.h"

// Configuration de l'UART pour le module ESP32-S3-DevKitC-1 v1.1
#define UART_TX (GPIO_NUM_43)               // TX pin
#define UART_RX (GPIO_NUM_44)               // RX pin
#define UART_RTS (UART_PIN_NO_CHANGE)       // RTS pin
#define UART_CTS (UART_PIN_NO_CHANGE)       // CTS pin

#define UART_PORT_NUM      (1)          // Utiliser UART1
#define UART_BAUD_RATE     (115200)     // Débit en bauds
#define UART_NUM           (UART_NUM_1) // Numéro de l'UART
#define BUF_SIZE           (1024)       // Taille du tampon de réception

// Configuration des temps (modifiable via des #define)
#define UART_READ_TIMEOUT_MS (1)    // Timeout pour la lecture UART en millisecondes
#define MOTOR_TASK_DELAY_MS  (100)  // Délai pour la tâche moteur en millisecondes

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
    return checksum;
}

// Tâche 1 : Lecture UART avec gestion de trame
static void uart_task(void *arg)
{
    uint8_t data[BUF_SIZE]; // Tampon statique
    uint8_t frame[5];       // Tampon pour une trame
    size_t frame_index = 0; // Index pour remplir la trame

    while (1) {
        // Lire les données de l'UART
        int len = uart_read_bytes(UART_PORT_NUM, data, BUF_SIZE, UART_READ_TIMEOUT_MS / portTICK_PERIOD_MS);
        for (int i = 0; i < len; i++) {
            uint8_t byte = data[i];
            
            // Cherche le début de la trame avec MOTSMAGIC
            if (frame_index == 0 && byte != MOTSMAGIC) {
                continue;
            }

            // Ajouter l'octet à la trame
            frame[frame_index++] = byte;

            // Check si la trame est complète
            if (frame_index == 5) {
                // Reset l'index pour la prochaine trame
                frame_index = 0;

                // Vérifier le checksum
                uint8_t checksum = calculate_checksum(frame, 4); // Calculer le checksum des 4 premiers octets
                if (checksum == frame[4]) {
                    // Extraire les angles
                    int angle1 = (int8_t)frame[1]; // Octet signé
                    int angle2 = (int8_t)frame[2];
                    int angle3 = (int8_t)frame[3];

                    // Validation des angles
                    if (angle1 >= -180 && angle1 <= 180 &&
                        angle2 >= -180 && angle2 <= 180 &&
                        angle3 >= -180 && angle3 <= 180) {
                        motor_angles_t angles = {angle1, angle2, angle3};

                        // Envoyer les angles dans la file
                        if (xQueueSend(angle_queue, &angles, 0) != pdPASS) {
                            printf("Erreur : file pleine, angles non envoyés\n");
                        }
                    } else {
                        printf("Angles invalides reçus : %d, %d, %d\n", angle1, angle2, angle3);
                    }
                } else {
                    printf("Checksum invalide : attendu %02X, reçu %02X\n", checksum, frame[4]);
                }
            }
        }

        vTaskDelay(10 / portTICK_PERIOD_MS); // Délai pour éviter de surcharger le processeur
    }
}

// Tâche 2 : Commande du moteur
void motor_task(void *arg)
{
    motor_angles_t angles;

    while (1) {
        // Attendre les angles dans la file
        if (xQueueReceive(angle_queue, &angles, portMAX_DELAY) == pdPASS) {
            printf("Angles reçus : %d, %d, %d\n", angles.angle1, angles.angle2, angles.angle3);

            // Déplacer les moteurs vers les angles reçus
            sync_move_all_to(angles);
        }

        // Délai pour éviter une surcharge inutile
        vTaskDelay(pdMS_TO_TICKS(MOTOR_TASK_DELAY_MS));
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

    // Initialisation des moteurs
    motor_init();

    // Initialisation de l'UART
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_PORT_NUM, UART_TX, UART_RX, UART_RTS, UART_CTS);
    uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);

    // Création des tâches
    xTaskCreate(uart_task, "uart_task", TASK_STACK_SIZE, NULL, 10, NULL);
    xTaskCreate(motor_task, "motor_task", TASK_STACK_SIZE, NULL, 10, NULL);
}