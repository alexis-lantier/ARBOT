// Lib
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "driver/gpio.h"

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

// Tâche 1 : Lecture UART avec écho
static void uart_task(void *arg)
{
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);

    while (1) {
        int len = uart_read_bytes(UART_PORT_NUM, data, (BUF_SIZE - 1), UART_READ_TIMEOUT_MS / portTICK_PERIOD_MS);
        if (len > 0) {
            uart_write_bytes(UART_PORT_NUM, (const char *) data, len);
        }
    }
}

// Tâche 2 : Commande du moteur
void motor_task(void *arg)
{
    while (1) {
        // Commande moteur

        vTaskDelay(MOTOR_TASK_DELAY_MS / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    // Initialisation des GPIO pour le contrôle des moteurs
    

    // Initialisation des moteurs pas à pas


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
    xTaskCreate(uart_task, "uart_task", 2048, NULL, 10, NULL);
    xTaskCreate(motor_task, "motor_task", 2048, NULL, 10, NULL);
}