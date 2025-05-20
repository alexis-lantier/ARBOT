// ...en-tête...
// filepath: main.c (à renommer en .ino pour Arduino IDE)
#include <Arduino.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>

// --- CONFIGURATION ---
#define UART_BAUD_RATE (115200)
#define BUFFER_SIZE (128)
#define TRAME_SIZE (5)

#define MOTOR_COUNT 3
#define MOTOR1_STEP_PIN 7
#define MOTOR1_DIR_PIN  8
#define MOTOR2_STEP_PIN 9
#define MOTOR2_DIR_PIN  10
#define MOTOR3_STEP_PIN 11
#define MOTOR3_DIR_PIN  12
#define ENABLE_PIN      6

#define STEP_ANGLE          (0.45f)
#define STEP_DELAY_MIN_US   (500)
#define STEP_DELAY_MAX_US   (1200)
#define STEP_RAMP           (45)

#define ANGLE_MAX 120
#define ANGLE_MIN -120

#define MOTSMAGIC 0x79
#define MODE_ERROR 0x01
#define MODE_OK 0x00
#define CODE_ERROR_DEPASS 0x05
#define CODE_ERROR_CHECKSUM 0x04
#define CODE_ERROR_ANGLE 0x03
#define CODE_OK 0x01

typedef struct {
    uint8_t stepPin;
    uint8_t dirPin;
    int8_t angle;
    int position;
    int target;
    bool dir;
} StepperMotor;

static const uint8_t step_pins[MOTOR_COUNT] = {MOTOR1_STEP_PIN, MOTOR2_STEP_PIN, MOTOR3_STEP_PIN};
static const uint8_t dir_pins[MOTOR_COUNT]  = {MOTOR1_DIR_PIN,  MOTOR2_DIR_PIN,  MOTOR3_DIR_PIN};

typedef enum {
    STATE_IDLE,
    STATE_UART_RECEIVE,
    STATE_VALIDATE_FRAME,
    STATE_MOVE_MOTORS,
    STATE_ERROR
} main_state_t;

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
    Serial.write(error_frame, 4);
}

int angle_to_steps(float angle) {
    return (int)roundf(angle / STEP_ANGLE);
}

void motors_init(StepperMotor *motors) {
    for (int i = 0; i < MOTOR_COUNT; ++i) {
        motors[i].stepPin = step_pins[i];
        motors[i].dirPin = dir_pins[i];
        motors[i].position = 0;
        motors[i].target = 0;
        motors[i].dir = true;
    }
}

void motors_begin(StepperMotor *motors) {
    for (int i = 0; i < MOTOR_COUNT; ++i) {
        pinMode(motors[i].stepPin, OUTPUT);
        pinMode(motors[i].dirPin, OUTPUT);
    }
    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, LOW); // Active drivers
}

bool motor_tick(StepperMotor* motor) {
    if (motor->position == motor->target) return false;
    digitalWrite(motor->dirPin, motor->dir ? HIGH : LOW);
    digitalWrite(motor->stepPin, HIGH);
    delayMicroseconds(2);
    digitalWrite(motor->stepPin, LOW);
    delayMicroseconds(2);
    motor->position += motor->dir ? 1 : -1;
    return true;
}

void sync_move_to_targets(StepperMotor *motors){
    int distances[MOTOR_COUNT] = {0};
    int totalSteps = 0;
    float counters[MOTOR_COUNT] = {0};

    for (int i = 0; i < MOTOR_COUNT; ++i) {
        distances[i] = abs(motors[i].target - motors[i].position);
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
        int min_delay = STEP_DELAY_MAX_US;
        for (int i = 0; i < MOTOR_COUNT; ++i) {
            int ramp = distances[i] < STEP_RAMP ? distances[i] : STEP_RAMP;
            int delay;
            if (step < ramp)
                delay = STEP_DELAY_MAX_US - (STEP_DELAY_MAX_US - STEP_DELAY_MIN_US) * step / ramp;
            else if (step > totalSteps - ramp)
                delay = STEP_DELAY_MAX_US - (STEP_DELAY_MAX_US - STEP_DELAY_MIN_US) * (totalSteps - step) / ramp;
            else
                delay = STEP_DELAY_MIN_US;
            if (delay < min_delay) min_delay = delay;
        }
        delayMicroseconds(min_delay);
    }
}

// --- ARDUINO MAIN ---
StepperMotor motors[MOTOR_COUNT];
uint8_t uart_buffer[BUFFER_SIZE] = {0};
uint8_t frame[TRAME_SIZE] = {0};
size_t frame_index = 0;
main_state_t state = STATE_IDLE;

void setup() {
    Serial.begin(UART_BAUD_RATE);
    motors_init(motors);
    motors_begin(motors);
}

void loop() {
    switch (state) {
        case STATE_IDLE:
            state = STATE_UART_RECEIVE;
            break;

        case STATE_UART_RECEIVE: {
            while (Serial.available()) {
                uint8_t byte = Serial.read();
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
                int8_t angle[MOTOR_COUNT] = {0};
                for (int i = 0; i < MOTOR_COUNT; ++i) {
                    angle[i] = (int8_t)frame[i + 1];
                }
                if (angle[0] >= ANGLE_MIN && angle[0] <= ANGLE_MAX &&
                    angle[1] >= ANGLE_MIN && angle[1] <= ANGLE_MAX &&
                    angle[2] >= ANGLE_MIN && angle[2] <= ANGLE_MAX) {
                    send_uart_status_code(MODE_OK, CODE_OK);
                } else {
                    send_uart_status_code(MODE_ERROR, CODE_ERROR_ANGLE);
                    state = STATE_IDLE;
                    break;
                }
                for(int i = 0; i < MOTOR_COUNT; ++i) {
                    motors[i].angle = angle[i];
                    motors[i].target = angle_to_steps(motors[i].angle);
                    motors[i].dir = (motors[i].target > motors[i].position);
                }
                state = STATE_MOVE_MOTORS;
            } else {
                send_uart_status_code(MODE_ERROR, CODE_ERROR_CHECKSUM);
                state = STATE_IDLE;
            }
            break;
        }

        case STATE_MOVE_MOTORS:
            sync_move_to_targets(motors);
            state = STATE_IDLE;
            break;

        case STATE_ERROR:
            state = STATE_IDLE;
            break;

        default:
            state = STATE_IDLE;
            break;
    }
}