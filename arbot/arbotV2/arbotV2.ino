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

// Quart de pas = 0.45° par pas
const float stepAngle = 0.45;  // Quarter-step mode activé physiquement (M0 = LOW, M1 = HIGH, M2 = LOW)
const int delay_min = 500;     // Delay minimal pour vitesse max
const int delay_max = 1200;    // Delay maximal pour vitesse lente (départ/arrêt)
const int ramp_steps = 45;     // Nombre de pas d'accélération/décélération

int angleToSteps(float angle) {
  return round(angle / stepAngle);
}

// Classe moteur
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

// Déclaration des moteurs
StepperMotor motor1(MOTOR1_STEP_PIN, MOTOR1_DIR_PIN);
StepperMotor motor2(MOTOR2_STEP_PIN, MOTOR2_DIR_PIN);
StepperMotor motor3(MOTOR3_STEP_PIN, MOTOR3_DIR_PIN);

int getRampDelay(int stepIndex, int totalSteps) {
  if (stepIndex < ramp_steps) {
    return delay_max - (delay_max - delay_min) * stepIndex / ramp_steps;
  } else if (stepIndex > totalSteps - ramp_steps) {
    return delay_max - (delay_max - delay_min) * (totalSteps - stepIndex) / ramp_steps;
  }
  return delay_min;
}

void syncMoveAllTo(int target) {
  motor1.prepareMoveTo(target);
  motor2.prepareMoveTo(target);
  motor3.prepareMoveTo(target);

  int totalSteps = max(abs(motor1.target - motor1.position), max(abs(motor2.target - motor2.position), abs(motor3.target - motor3.position)));
  for (int step = 0; step < totalSteps; step++) {
    if (motor1.isMoving()) motor1.tick();
    if (motor2.isMoving()) motor2.tick();
    if (motor3.isMoving()) motor3.tick();
    delayMicroseconds(getRampDelay(step, totalSteps));
  }
}
void rampedMove(StepperMotor& motor, int target);
void rampedMove(StepperMotor& motor, int target) {
  motor.prepareMoveTo(target);
  int totalSteps = abs(motor.target - motor.position);
  for (int step = 0; step < totalSteps; step++) {
    motor.tick();
    delayMicroseconds(getRampDelay(step, totalSteps));
  }
}

// --- Fin UART ---
void setup() {
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);  // Active les drivers

  motor1.begin();
  motor2.begin();
  motor3.begin();

  int steps45 = angleToSteps(45);
  int steps90 = angleToSteps(90);

  // Tous montent à 45° en même temps (rampé)
  syncMoveAllTo(steps45);
  delay(500);

  // Tous montent à 90° l'un après l'autre (rampé)
  rampedMove(motor1, steps90);
  rampedMove(motor2, steps90);
  rampedMove(motor3, steps90);
    // Tous montent à 45° l'un après l'autre (rampé)
  rampedMove(motor1, steps45);
  rampedMove(motor2, steps45);
  rampedMove(motor3, steps45);
    // Tous montent à 90° l'un après l'autre (rampé)
  rampedMove(motor1, steps90);
  rampedMove(motor2, steps90);
  rampedMove(motor3, steps90);
      // Tous montent à 45° l'un après l'autre (rampé)
  rampedMove(motor1, steps45);
  rampedMove(motor2, steps45);
  rampedMove(motor3, steps45);
    // Tous montent à 90° l'un après l'autre (rampé)
  rampedMove(motor1, steps90);
  rampedMove(motor2, steps90);
  rampedMove(motor3, steps90);
  delay(500);

  // Tous redescendent à 45° en même temps (rampé)
  syncMoveAllTo(steps45);
  delay(500);

  // Tous redescendent à 0° en même temps (rampé)
  syncMoveAllTo(0);
  delay(500);

  // Tous remontent à 45° l'un après l'autre (rampé)
  rampedMove(motor1, steps45);
  delay(200);
  rampedMove(motor2, steps45);
  delay(200);
  rampedMove(motor3, steps45);
  delay(500);

  // Tous montent à 90° l'un après l'autre (rampé)
  rampedMove(motor1, steps90);
  delay(200);
  rampedMove(motor2, steps90);
  delay(200);
  rampedMove(motor3, steps90);
  delay(500);

  // Tous redescendent à 45° en même temps (rampé)
  syncMoveAllTo(steps45);
  delay(500);
  syncMoveAllTo(steps90);
  delay(75);
  syncMoveAllTo(steps45);
  delay(75);
  syncMoveAllTo(steps90);
  delay(75);
  syncMoveAllTo(steps45);
  delay(75);
   // Tous redescendent à 90° en même temps (rampé)
  syncMoveAllTo(steps90);
  delay(100);

  // Tous redescendent à 0° en même temps (rampé)
  syncMoveAllTo(0);
  delay(500);

  digitalWrite(ENABLE_PIN, HIGH);  // désactive les drivers
}

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
                send_uart_status_code(MODE_OK, CODE_OK);
            } else {
                send_uart_status_code(MODE_ERROR, CODE_ERROR_ANGLE);
                state = STATE_IDLE;
                break;
            }
            for(int i = 0; i < MOTOR_COUNT; ++i) {
                step[i] = angleToSteps(angle[i]);
            }
            state = STATE_MOVE_MOTORS;
        } else {
            send_uart_status_code(MODE_ERROR, CODE_ERROR_CHECKSUM);
            state = STATE_IDLE;
        }
        break;
    }

    case STATE_MOVE_MOTORS:
        // Tous montent à 90° l'un après l'autre (rampé)
        rampedMove(motor1, step[0]);
        rampedMove(motor2, step[1]);
        rampedMove(motor3, step[2]);
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