#include "motor_control.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

// Fonction pour initialiser les moteurs
void motor_init(void) {
    gpio_set_direction(MOTOR1_STEP_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR1_DIR_PIN, GPIO_MODE_OUTPUT);

    gpio_set_direction(MOTOR2_STEP_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR2_DIR_PIN, GPIO_MODE_OUTPUT);

    gpio_set_direction(MOTOR3_STEP_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR3_DIR_PIN, GPIO_MODE_OUTPUT);

    gpio_set_direction(ENABLE_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(ENABLE_PIN, 0); // Activer les moteurs
}

// Fonction pour déplacer un moteur
void move_motor(int step_pin, int dir_pin, int steps, bool direction) {
    gpio_set_level(dir_pin, direction ? 1 : 0); // Définir la direction

    for (int i = 0; i < steps; i++) {
        gpio_set_level(step_pin, 1);
        vTaskDelay(pdMS_TO_TICKS(STEP_DELAY_US / 1000)); // Convertir microsecondes en millisecondes
        gpio_set_level(step_pin, 0);
        vTaskDelay(pdMS_TO_TICKS(STEP_DELAY_US / 1000));
    }
}

// Fonction pour calculer le délai de rampe
static int get_ramp_delay(int step_index, int total_steps) {
    if (step_index < RAMP_STEPS) {
        return STEP_DELAY_US + (STEP_DELAY_US * (RAMP_STEPS - step_index)) / RAMP_STEPS;
    } else if (step_index > total_steps - RAMP_STEPS) {
        return STEP_DELAY_US + (STEP_DELAY_US * (step_index - (total_steps - RAMP_STEPS))) / RAMP_STEPS;
    }
    return STEP_DELAY_US;
}

// Fonction pour déplacer tous les moteurs en synchronisation
void sync_move_all_to(motor_angles_t angles) {
    // Conversion des angles en pas
    int motor1_steps = angles.angle1 * STEPS_PER_DEGREE;
    int motor2_steps = angles.angle2 * STEPS_PER_DEGREE;
    int motor3_steps = angles.angle3 * STEPS_PER_DEGREE;

    // Déterminer la direction pour chaque moteur
    int motor1_dir = (motor1_steps >= 0) ? 1 : 0;
    int motor2_dir = (motor2_steps >= 0) ? 1 : 0;
    int motor3_dir = (motor3_steps >= 0) ? 1 : 0;

    // Configurer la direction des moteurs
    gpio_set_level(MOTOR1_DIR_PIN, motor1_dir);
    gpio_set_level(MOTOR2_DIR_PIN, motor2_dir);
    gpio_set_level(MOTOR3_DIR_PIN, motor3_dir);

    // Prendre la valeur absolue des pas
    motor1_steps = abs(motor1_steps);
    motor2_steps = abs(motor2_steps);
    motor3_steps = abs(motor3_steps);

    // Calculer le nombre total de pas nécessaires
    int total_steps = (int)fmax(fmax(motor1_steps, motor2_steps), motor3_steps);

    // Variables pour synchroniser les moteurs
    int motor1_accumulator = 0;
    int motor2_accumulator = 0;
    int motor3_accumulator = 0;

    for (int step = 0; step < total_steps; step++) {
        // Déplacer le moteur 1 si nécessaire
        if (motor1_accumulator < motor1_steps && (motor1_accumulator * total_steps) / motor1_steps <= step) {
            gpio_set_level(MOTOR1_STEP_PIN, 1);
            motor1_accumulator++;
        }

        // Déplacer le moteur 2 si nécessaire
        if (motor2_accumulator < motor2_steps && (motor2_accumulator * total_steps) / motor2_steps <= step) {
            gpio_set_level(MOTOR2_STEP_PIN, 1);
            motor2_accumulator++;
        }

        // Déplacer le moteur 3 si nécessaire
        if (motor3_accumulator < motor3_steps && (motor3_accumulator * total_steps) / motor3_steps <= step) {
            gpio_set_level(MOTOR3_STEP_PIN, 1);
            motor3_accumulator++;
        }

        // Petite impulsion pour les moteurs
        vTaskDelay(pdMS_TO_TICKS(1));

        // Réinitialiser les broches STEP
        gpio_set_level(MOTOR1_STEP_PIN, 0);
        gpio_set_level(MOTOR2_STEP_PIN, 0);
        gpio_set_level(MOTOR3_STEP_PIN, 0);

        // Délai pour gérer la rampe
        vTaskDelay(pdMS_TO_TICKS(get_ramp_delay(step, total_steps) / 1000));
    }
}

// Fonction pour déplacer un moteur avec une rampe
void ramped_move(int step_pin, int dir_pin, int target_steps, bool direction) {
    gpio_set_level(dir_pin, direction ? 1 : 0); // Définir la direction

    for (int step = 0; step < target_steps; step++) {
        gpio_set_level(step_pin, 1);
        vTaskDelay(pdMS_TO_TICKS(1)); 
        gpio_set_level(step_pin, 0);
        vTaskDelay(pdMS_TO_TICKS(get_ramp_delay(step, target_steps) / 1000)); // Convertir microsecondes en ticks
    }
}