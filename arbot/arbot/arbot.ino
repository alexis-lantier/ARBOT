#include <Arduino.h>

#define STEP1 37
#define DIR1  38
#define STEP2 39
#define DIR2  40
#define STEP3 41
#define DIR3  42
#define ENABLE_PIN 36

const float stepAngle = 0.45;
const int delay_min = 500;
const int delay_max = 1200;
const int ramp_steps = 45;

int angleToSteps(float angle) {
  return round(angle / stepAngle);
}

class StepperMotor {
public:
  int stepPin, dirPin;
  int position = 0;
  int target = 0;
  bool dir = true;

  StepperMotor(int step, int dir) : stepPin(step), dirPin(dir) {}

  void begin() {
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
  }

  void prepareMoveTo(int newTarget) {
    target = newTarget;
    dir = (target > position);
    digitalWrite(dirPin, dir ? HIGH : LOW);
  }

  void tick() {
    if (position == target) return;
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(2);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(2);
    position += dir ? 1 : -1;
  }

  bool isMoving() {
    return position != target;
  }
};

StepperMotor m1(STEP1, DIR1);
StepperMotor m2(STEP2, DIR2);
StepperMotor m3(STEP3, DIR3);

int getRampDelay(int i, int total) {
  if (i < ramp_steps)
    return delay_max - (delay_max - delay_min) * i / ramp_steps;
  if (i > total - ramp_steps)
    return delay_max - (delay_max - delay_min) * (total - i) / ramp_steps;
  return delay_min;
}

void syncMoveAllTo(int a1, int a2, int a3) {
  m1.prepareMoveTo(a1);
  m2.prepareMoveTo(a2);
  m3.prepareMoveTo(a3);

  int totalSteps = max(abs(m1.target - m1.position),
                       max(abs(m2.target - m2.position),
                           abs(m3.target - m3.position)));

  for (int i = 0; i < totalSteps; i++) {
    if (m1.isMoving()) m1.tick();
    if (m2.isMoving()) m2.tick();
    if (m3.isMoving()) m3.tick();
    delayMicroseconds(getRampDelay(i, totalSteps));
  }
}

void setup() {
  Serial.begin(115200);
  m1.begin();
  m2.begin();
  m3.begin();
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW); // Active les drivers
}

void loop() {
  char buffer[30];
  if (Serial.available()) {
    size_t len = Serial.readBytesUntil('\n', buffer, sizeof(buffer) - 1);
    buffer[len] = '\0';

    float a1 = 0, a2 = 0, a3 = 0;
    if (sscanf(buffer, "%f:%f:%f", &a1, &a2, &a3) == 3) {
      Serial.printf("ANGLES REÇUS: %.2f %.2f %.2f\n", a1, a2, a3);
      syncMoveAllTo(angleToSteps(a1), angleToSteps(a2), angleToSteps(a3));
      delay(100);
    } else {
      Serial.println("❌ Format invalide");
    }
  }
}

