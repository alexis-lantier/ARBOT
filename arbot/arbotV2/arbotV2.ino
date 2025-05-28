// --- main.ino ---
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
#define MOTOR1_STEP_PIN 37
#define MOTOR1_DIR_PIN  38
#define MOTOR2_STEP_PIN 39
#define MOTOR2_DIR_PIN  40
#define MOTOR3_STEP_PIN 41
#define MOTOR3_DIR_PIN  42
#define ENABLE_PIN      36

#define ANGLE_MAX 120
#define ANGLE_MIN -120

#define MOTSMAGIC 0x79
#define MODE_ERROR 0x01
#define MODE_OK 0x00
#define CODE_ERROR_DEPASS 0x05
#define CODE_ERROR_CHECKSUM 0x04
#define CODE_ERROR_ANGLE 0x03
#define CODE_OK 0x01

const float stepAngle = 0.45; // ou la valeur de ton moteur
const int delay_min = 1000;    // vitesse max très lente (plus grand = plus lent)
const int delay_max = 2500;  // rampe très douce (plus grand = plus lent)
const int ramp_steps = 80;    // rampe plus longue (optionnel)

static const uint8_t step_pins[MOTOR_COUNT] = {MOTOR1_STEP_PIN, MOTOR2_STEP_PIN, MOTOR3_STEP_PIN};
static const uint8_t dir_pins[MOTOR_COUNT]  = {MOTOR1_DIR_PIN,  MOTOR2_DIR_PIN,  MOTOR3_DIR_PIN};

typedef enum {
    STATE_IDLE,
    STATE_UART_RECEIVE,
    STATE_VALIDATE_FRAME,
    STATE_MOVE_MOTORS,
    STATE_ERROR
} main_state_t;

// --- Classe moteur ---
class StepperMotor {
public:
    int stepPin, dirPin;
    int position = 0;
    int target = 0;
    bool dir = true;

    StepperMotor(int step, int dir) {
        stepPin = step;
        dirPin = dir;
    }

    void begin() {
        pinMode(stepPin, OUTPUT);
        pinMode(dirPin, OUTPUT);
    }

    void prepareMoveTo(int newTarget) {
        target = newTarget;
        dir = (target > position);
        digitalWrite(dirPin, dir ? HIGH : LOW);
    }

    bool tick() {
        if (position == target) return false;
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(2);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(2);
        position += dir ? 1 : -1;
        return true;
    }

    bool isMoving() {
        return position != target;
    }
};

// --- Déclaration des moteurs ---
StepperMotor motor1(MOTOR1_STEP_PIN, MOTOR1_DIR_PIN);
StepperMotor motor2(MOTOR2_STEP_PIN, MOTOR2_DIR_PIN);
StepperMotor motor3(MOTOR3_STEP_PIN, MOTOR3_DIR_PIN);

// --- Fonctions utilitaires ---
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

int angleToSteps(float angle) {
    return round(angle / stepAngle);
}

int getRampDelay(int stepIndex, int totalSteps) {
    int accelSteps = min(ramp_steps, totalSteps / 2);
    if (accelSteps < 1) accelSteps = 1; // Protection division par zéro
    int cruiseSteps = totalSteps - 2 * accelSteps;
    if (cruiseSteps < 0) cruiseSteps = 0;

    if (stepIndex < accelSteps) {
        // Accélération linéaire
        return delay_max - (delay_max - delay_min) * stepIndex / accelSteps;
    } else if (stepIndex < (accelSteps + cruiseSteps)) {
        // Vitesse constante
        return delay_min;
    } else {
        // Décélération linéaire
        int decelStep = stepIndex - (accelSteps + cruiseSteps);
        return delay_min + (delay_max - delay_min) * decelStep / accelSteps;
    }
}

void syncMoveAllTo(int target) {
    motor1.prepareMoveTo(target);
    motor2.prepareMoveTo(target);
    motor3.prepareMoveTo(target);

    int totalSteps = max(abs(motor1.target - motor1.position), 
      max(abs(motor2.target - motor2.position), 
          abs(motor3.target - motor3.position)));
    for (int step = 0; step < totalSteps; step++) {
        if (motor1.isMoving()) motor1.tick();
        if (motor2.isMoving()) motor2.tick();
        if (motor3.isMoving()) motor3.tick();
        delayMicroseconds(getRampDelay(step, totalSteps));
    }
}

void rampedMove(StepperMotor& motor, int target) {
    motor.prepareMoveTo(target);
    int totalSteps = abs(motor.target - motor.position);
    for (int step = 0; step < totalSteps; step++) {
        motor.tick();
        delayMicroseconds(getRampDelay(step, totalSteps));
    }
}

void syncMoveAllTo(int a1, int a2, int a3) {
    motor1.prepareMoveTo(a1);
    motor2.prepareMoveTo(a2);
    motor3.prepareMoveTo(a3);

    int totalSteps1 = abs(motor1.target - motor1.position);
    int totalSteps2 = abs(motor2.target - motor2.position);
    int totalSteps3 = abs(motor3.target - motor3.position);
    int totalSteps = max(totalSteps1, max(totalSteps2, totalSteps3));

    int step1 = 0, step2 = 0, step3 = 0;

    for (int i = 0; i < totalSteps; i++) {
        unsigned long delay1 = (motor1.isMoving()) ? getRampDelay(step1, totalSteps1) : 0;
        unsigned long delay2 = (motor2.isMoving()) ? getRampDelay(step2, totalSteps2) : 0;
        unsigned long delay3 = (motor3.isMoving()) ? getRampDelay(step3, totalSteps3) : 0;

        if (motor1.isMoving()) { motor1.tick(); step1++; }
        if (motor2.isMoving()) { motor2.tick(); step2++; }
        if (motor3.isMoving()) { motor3.tick(); step3++; }

        // On prend le plus petit délai non nul pour ne pas ralentir inutilement
        unsigned long d = 0;
        if (delay1 && (!d || delay1 < d)) d = delay1;
        if (delay2 && (!d || delay2 < d)) d = delay2;
        if (delay3 && (!d || delay3 < d)) d = delay3;
        if (d) delayMicroseconds(d);
    }
}

// --- Setup ---
void setup() {
    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, LOW);  // Active les drivers

    motor1.begin();
    motor2.begin();
    motor3.begin();
    Serial.begin(UART_BAUD_RATE);
    delay(2);
}

// --- Loop principale ---
void loop() {
    static main_state_t state = STATE_IDLE;
    static uint8_t frame[TRAME_SIZE] = {0};
    static size_t frame_index = 0;
    static int step[MOTOR_COUNT] = {0};

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

                    for(int i = 0; i < MOTOR_COUNT; ++i) {
                        step[i] = angleToSteps(angle[i]);
                    }
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
            syncMoveAllTo(step[0], step[1], step[2]);
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