#include "motor_control.h"



StepperMotor motors[MOTOR_COUNT];

static const gpio_num_t step_pins[MOTOR_COUNT] = {
    MOTOR1_STEP_PIN, MOTOR2_STEP_PIN, MOTOR3_STEP_PIN
};
static const gpio_num_t dir_pins[MOTOR_COUNT] = {
    MOTOR1_DIR_PIN, MOTOR2_DIR_PIN, MOTOR3_DIR_PIN
};

void motors_init(void) {
    for (int i = 0; i < MOTOR_COUNT; ++i) {
        motors[i].stepPin = step_pins[i];
        motors[i].dirPin = dir_pins[i];
        motors[i].position = 0;
        motors[i].target = 0;
        motors[i].dir = true;
    }
}

void motors_begin(void) {
    for (int i = 0; i < MOTOR_COUNT; ++i) {
        gpio_set_direction(motors[i].stepPin, GPIO_MODE_OUTPUT);
        gpio_set_direction(motors[i].dirPin, GPIO_MODE_OUTPUT);
    }

    // Enabling the drivers
    gpio_set_direction(ENABLE_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(ENABLE_PIN, 0); // Active drivers
}

void motors_prepare_move_to(int target) {
    for (int i = 0; i < MOTOR_COUNT; ++i) {
        motors[i].target = target;
        motors[i].dir = (motors[i].target > motors[i].position);
        gpio_set_level(motors[i].dirPin, motors[i].dir ? 0 : 1);
    }
}

bool motor_tick(StepperMotor* motor) {
    if (motor->position == motor->target) return false;
    gpio_set_level(motor->stepPin, 1);
    esp_rom_delay_us(2);
    gpio_set_level(motor->stepPin, 0);
    esp_rom_delay_us(2);
    motor->position += motor->dir ? 1 : -1;
    return true;
}

bool motor_is_moving(StepperMotor* motor) {
    return motor->position != motor->target;
}

int angle_to_steps(float angle) {
    return (int)roundf(angle / STEP_ANGLE);
}

int get_ramp_delay(int stepIndex, int totalSteps) {
    if (stepIndex < STEP_RAMP)
        return STEP_DELAY_MAX_US - (STEP_DELAY_MAX_US - STEP_DELAY_MIN_US) * stepIndex / STEP_RAMP;
    else if (stepIndex > totalSteps - STEP_RAMP)
        return STEP_DELAY_MAX_US - (STEP_DELAY_MAX_US - STEP_DELAY_MIN_US) * (totalSteps - stepIndex) / STEP_RAMP;
    return STEP_DELAY_MIN_US;
}

void sync_move_all_to(int target) {
    motors_prepare_move_to(target);

    int totalSteps = 0;
    for (int i = 0; i < MOTOR_COUNT; ++i) {
        int steps = abs(motors[i].target - motors[i].position);
        if (steps > totalSteps) totalSteps = steps;
    }

    for (int step = 0; step < totalSteps; ++step) {
        for (int i = 0; i < MOTOR_COUNT; ++i) {
            if (motor_is_moving(&motors[i])) motor_tick(&motors[i]);
        }
        esp_rom_delay_us(get_ramp_delay(step, totalSteps));
    }
}

void ramped_move(StepperMotor* motor, int target) {
    motor->target = target;
    motor->dir = (motor->target > motor->position);
    gpio_set_level(motor->dirPin, motor->dir ? 1 : 0);
    int totalSteps = abs(motor->target - motor->position);
    for (int step = 0; step < totalSteps; ++step) {
        motor_tick(motor);
        esp_rom_delay_us(get_ramp_delay(step, totalSteps));
    }
}

void motor_control_demo_sequence(void) {
    gpio_set_direction(ENABLE_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(ENABLE_PIN, 0); // Active drivers

    motors_init();
    motors_begin();

    int steps45 = angle_to_steps(45.0f);
    int steps90 = angle_to_steps(90.0f);

    sync_move_all_to(steps45); vTaskDelay(pdMS_TO_TICKS(500));
    for (int i = 0; i < MOTOR_COUNT; ++i) ramped_move(&motors[i], steps90);
    for (int i = 0; i < MOTOR_COUNT; ++i) ramped_move(&motors[i], steps45);
    for (int i = 0; i < MOTOR_COUNT; ++i) ramped_move(&motors[i], steps90);
    for (int i = 0; i < MOTOR_COUNT; ++i) ramped_move(&motors[i], steps45);
    for (int i = 0; i < MOTOR_COUNT; ++i) ramped_move(&motors[i], steps90);

    vTaskDelay(pdMS_TO_TICKS(500));
    sync_move_all_to(steps45); vTaskDelay(pdMS_TO_TICKS(500));
    sync_move_all_to(0); vTaskDelay(pdMS_TO_TICKS(500));

    for (int i = 0; i < MOTOR_COUNT; ++i) { ramped_move(&motors[i], steps45); vTaskDelay(pdMS_TO_TICKS(200)); }
    for (int i = 0; i < MOTOR_COUNT; ++i) { ramped_move(&motors[i], steps90); vTaskDelay(pdMS_TO_TICKS(200)); }

    sync_move_all_to(steps45); vTaskDelay(pdMS_TO_TICKS(500));
    sync_move_all_to(steps90); vTaskDelay(pdMS_TO_TICKS(75));
    sync_move_all_to(steps45); vTaskDelay(pdMS_TO_TICKS(75));
    sync_move_all_to(steps90); vTaskDelay(pdMS_TO_TICKS(75));
    sync_move_all_to(steps45); vTaskDelay(pdMS_TO_TICKS(75));
    sync_move_all_to(steps90); vTaskDelay(pdMS_TO_TICKS(100));
    sync_move_all_to(0); vTaskDelay(pdMS_TO_TICKS(500));

    gpio_set_level(ENABLE_PIN, 1); // Désactive drivers
}