#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

// lib
#include <stdbool.h>
#include <math.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

// Définition des broches des moteurs
#define MOTOR1_STEP_PIN GPIO_NUM_37
#define MOTOR1_DIR_PIN  GPIO_NUM_38
#define MOTOR2_STEP_PIN GPIO_NUM_39
#define MOTOR2_DIR_PIN  GPIO_NUM_40
#define MOTOR3_STEP_PIN GPIO_NUM_41
#define MOTOR3_DIR_PIN  GPIO_NUM_42
#define ENABLE_PIN      GPIO_NUM_36

#define STEP_ANGLE          (0.45f)  // Angle par pas (degré)
#define STEP_DELAY_MIN_US   (500)    // Délai minimum entre les pas (microsecondes)
#define STEP_DELAY_MAX_US   (1200)   // Délai maximum entre les pas (microsecondes)
#define STEP_RAMP           (45)     // Nombre de pas pour la rampe

#define MOTOR_COUNT 3

typedef struct {
    gpio_num_t stepPin;
    gpio_num_t dirPin;
    int position;
    int target;
    bool dir;
} StepperMotor;

typedef struct {
    int angle1;
    int angle2;
    int angle3;
} motor_angles_t;

// variables globales
extern StepperMotor motors[];

// Prototypes
void motors_init(void);
void motors_begin(void);
void motors_prepare_move_to(int target);
bool motor_tick(StepperMotor* motor);
bool motor_is_moving(StepperMotor* motor);
int angle_to_steps(float angle);
int get_ramp_delay(int stepIndex, int totalSteps);
void sync_move_all_to(int target);
void ramped_move(StepperMotor* motor, int target);
void motor_control_demo_sequence(void);

#endif // MOTOR_CONTROL_H