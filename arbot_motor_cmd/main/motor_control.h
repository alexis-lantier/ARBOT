#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "driver/gpio.h"
#include <stdbool.h>

// Définition des broches des moteurs
#define MOTOR1_STEP_PIN GPIO_NUM_18
#define MOTOR1_DIR_PIN  GPIO_NUM_19

#define MOTOR2_STEP_PIN GPIO_NUM_21
#define MOTOR2_DIR_PIN  GPIO_NUM_26

#define MOTOR3_STEP_PIN GPIO_NUM_27
#define MOTOR3_DIR_PIN  GPIO_NUM_28

#define ENABLE_PIN GPIO_NUM_2 // Broche pour activer/désactiver les moteurs

#define STEP_DELAY_US 1000  // Délai entre chaque pas en microsecondes
#define STEPS_PER_DEGREE 10 // Nombre de pas par degré
#define RAMP_STEPS 45       // Nombre de pas pour l'accélération/décélération

// Structure pour les angles des moteurs
typedef struct {
    int angle1;
    int angle2;
    int angle3;
} motor_angles_t;

// Prototypes des fonctions
void motor_init(void);
void move_motor(int step_pin, int dir_pin, int steps, bool direction);
void sync_move_all_to(motor_angles_t angles);
void ramped_move(int step_pin, int dir_pin, int target_steps, bool direction);

#endif // MOTOR_CONTROL_H