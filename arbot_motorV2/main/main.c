#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

// --- CONFIGURATION ---
#define UART_TX (GPIO_NUM_43)
#define UART_RX (GPIO_NUM_44)
#define UART_PORT_NUM (1)
#define UART_BAUD_RATE (115200)
#define UART_NUM (UART_NUM_1)
#define BUFFER_SIZE (128)
#define TRAME_SIZE (5)

#define MOTOR_COUNT 3
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

#define ANGLE_MAX 120
#define ANGLE_MIN -120

#define MOTSMAGIC 0x79
#define MODE_ERROR 0x01
#define MODE_OK 0x00
#define CODE_ERROR_DEPASS 0x05
#define CODE_ERROR_CHECKSUM 0x04
#define CODE_ERROR_ANGLE 0x03
#define CODE_OK 0x01

// --- STRUCTURES ---
typedef struct {
    gpio_num_t stepPin;
    gpio_num_t dirPin;
    int position;
    int target;
    bool dir;
} StepperMotor;

typedef struct {
    int8_t angle1;
    int8_t angle2;
    int8_t angle3;
} motor_angles_t;

// --- VARIABLES GLOBALES ---
StepperMotor motors[MOTOR_COUNT];
static const gpio_num_t step_pins[MOTOR_COUNT] = {MOTOR1_STEP_PIN, MOTOR2_STEP_PIN, MOTOR3_STEP_PIN};
static const gpio_num_t dir_pins[MOTOR_COUNT]  = {MOTOR1_DIR_PIN,  MOTOR2_DIR_PIN,  MOTOR3_DIR_PIN};

// --- ETATS ---
typedef enum {
    STATE_IDLE,
    STATE_UART_RECEIVE,
    STATE_VALIDATE_FRAME,
    STATE_MOVE_MOTORS,
    STATE_ERROR
} main_state_t;

// --- UTILS ---
static uint8_t calculate_checksum(uint8_t *data, size_t len) {
    uint8_t checksum = 0;
    for (size_t i = 0; i < len; i++) checksum += data[i];
    return checksum;
}

void send_uart_status_code(uint8_t mode, uint8_t code) {
    uint8_t error_frame[4];
    error_frame[0] = MOTSMAGIC;
    error_frame[1] = mode;
    error_frame[2] = code;
    error_frame[3] = calculate_checksum(error_frame, 3);
    uart_write_bytes(UART_PORT_NUM, (const char *)error_frame, sizeof(error_frame));
}

int get_ramp_delay(int stepIndex, int totalSteps) {
    if (stepIndex < STEP_RAMP)
        return STEP_DELAY_MAX_US - (STEP_DELAY_MAX_US - STEP_DELAY_MIN_US) * stepIndex / STEP_RAMP;
    else if (stepIndex > totalSteps - STEP_RAMP)
        return STEP_DELAY_MAX_US - (STEP_DELAY_MAX_US - STEP_DELAY_MIN_US) * (totalSteps - stepIndex) / STEP_RAMP;
    return STEP_DELAY_MIN_US;
}

int angle_to_steps(float angle) {
    return (int)roundf(angle / STEP_ANGLE);
}

// --- MOTEURS ---
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
    gpio_set_direction(ENABLE_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(ENABLE_PIN, 0); // Active drivers
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

// --- SYNCHRONOUS MOVE ---
void sync_move_to_targets(int targets[3]) {
    int distances[MOTOR_COUNT] = {0};
    int totalSteps = 0;
    float counters[MOTOR_COUNT] = {0};

    for (int i = 0; i < MOTOR_COUNT; ++i) {
        motors[i].target = targets[i];
        motors[i].dir = (targets[i] > motors[i].position);
        gpio_set_level(motors[i].dirPin, motors[i].dir ? 0 : 1);
        distances[i] = abs(targets[i] - motors[i].position);
        if (distances[i] > totalSteps) totalSteps = distances[i];
    }

    memset(counters, 0, sizeof(counters));
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

// --- TIMER ---
static volatile bool timer_flag = false;
void IRAM_ATTR on_timer(void* arg) { timer_flag = true; }
void timer_init_periodic(uint32_t us_period) {
    const esp_timer_create_args_t timer_args = {
        .callback = &on_timer,
        .name = "main_timer"
    };
    esp_timer_handle_t periodic_timer;
    esp_timer_create(&timer_args, &periodic_timer);
    esp_timer_start_periodic(periodic_timer, us_period);
}

// --- MAIN ---
void app_main(void) {
    // UART init
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_PORT_NUM, UART_TX, UART_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM, BUFFER_SIZE * 2, 0, 0, NULL, 0);

    // Moteurs
    motors_init();
    motors_begin();

    // Timer 10ms
    timer_init_periodic(10000);

    // Machine d'état
    main_state_t state = STATE_IDLE;
    uint8_t uart_buffer[BUFFER_SIZE] = {0};
    uint8_t frame[TRAME_SIZE] = {0};
    size_t frame_index = 0;
    motor_angles_t motor_angles;
    int targets[3] = {0};

    while (1) {
        if (!timer_flag) continue;
        timer_flag = false;

        switch (state) {
            case STATE_IDLE:
                state = STATE_UART_RECEIVE;
                break;

            case STATE_UART_RECEIVE: {
                int len = uart_read_bytes(UART_PORT_NUM, uart_buffer, BUFFER_SIZE, 0);
                for (int i = 0; i < len; i++) {
                    uint8_t byte = uart_buffer[i];
                    if (frame_index == 0 && byte != MOTSMAGIC) continue;
                    if (frame_index > 1 && byte == MOTSMAGIC) {
                        frame_index = 1;
                        frame[0] = MOTSMAGIC;
                        continue;
                    }
                    if (frame_index >= TRAME_SIZE) {
                        frame_index = 0;
                        send_uart_status_code(MODE_ERROR, CODE_ERROR_DEPASS);
                        continue;
                    }
                    frame[frame_index++] = byte;
                    if (frame_index == TRAME_SIZE) {
                        frame_index = 0;
                        state = STATE_VALIDATE_FRAME;
                    }
                }
                break;
            }

            case STATE_VALIDATE_FRAME: {
                uint8_t checksum = calculate_checksum(frame, 4);
                if (checksum == frame[TRAME_SIZE - 1]) {
                    motor_angles.angle1 = (int8_t)frame[1];
                    motor_angles.angle2 = (int8_t)frame[2];
                    motor_angles.angle3 = (int8_t)frame[3];
                    if (motor_angles.angle1 >= ANGLE_MIN && motor_angles.angle1 <= ANGLE_MAX &&
                        motor_angles.angle2 >= ANGLE_MIN && motor_angles.angle2 <= ANGLE_MAX &&
                        motor_angles.angle3 >= ANGLE_MIN && motor_angles.angle3 <= ANGLE_MAX) {
                        send_uart_status_code(MODE_OK, CODE_OK);
                        targets[0] = motor_angles.angle1;
                        targets[1] = motor_angles.angle2;
                        targets[2] = motor_angles.angle3;
                        state = STATE_MOVE_MOTORS;
                    } else {
                        send_uart_status_code(MODE_ERROR, CODE_ERROR_ANGLE);
                        state = STATE_IDLE;
                    }
                } else {
                    send_uart_status_code(MODE_ERROR, CODE_ERROR_CHECKSUM);
                    state = STATE_IDLE;
                }
                break;
            }

            case STATE_MOVE_MOTORS:
                sync_move_to_targets(targets);
                state = STATE_IDLE;
                break;

            case STATE_ERROR:
                state = STATE_IDLE;
                break;
        }
    }
}