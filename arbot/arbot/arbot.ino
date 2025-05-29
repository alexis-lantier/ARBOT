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
const int ramp_steps = 120;

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
/*
int getRampDelay(int i, int total) {
  if (i < ramp_steps)
    return delay_max - (delay_max - delay_min) * i / ramp_steps;
  if (i > total - ramp_steps)
    return delay_max - (delay_max - delay_min) * (total - i) / ramp_steps;
  return delay_min;
}*/

int getRampDelay(int i, int total) {
  if (total < 2) return delay_max; // Mouvement trop court
  int half = total / 2;
  if (half < 1) return delay_min; // Pas assez de pas pour une rampe
  if (i < half) {
    // Accélération
    return delay_max - (delay_max - delay_min) * i / half;
  } else {
    // Décélération
    return delay_max - (delay_max - delay_min) * (total - i) / half;
  }
}

void syncMoveAllTo(int a1, int a2, int a3) {
  m1.prepareMoveTo(a1);
  m2.prepareMoveTo(a2);
  m3.prepareMoveTo(a3);

  int d1 = abs(m1.target - m1.position);
  int d2 = abs(m2.target - m2.position);
  int d3 = abs(m3.target - m3.position);

  int steps[3] = {d1, d2, d3};
  int dirs[3] = {m1.dir ? 1 : -1, m2.dir ? 1 : -1, m3.dir ? 1 : -1};
  int pos[3] = {m1.position, m2.position, m3.position};
  int targets[3] = {m1.target, m2.target, m3.target};

  int maxSteps = max(steps[0], max(steps[1], steps[2]));
  int counters[3] = {0, 0, 0};

  for (int i = 0; i < maxSteps; i++) {
    for (int m = 0; m < 3; m++) {
      counters[m] += steps[m];
      if (counters[m] >= maxSteps && pos[m] != targets[m]) {
        // Tick le moteur correspondant
        if (m == 0) m1.tick();
        if (m == 1) m2.tick();
        if (m == 2) m3.tick();
        counters[m] -= maxSteps;
      }
    }
    delayMicroseconds(getRampDelay(i, maxSteps));
  }
}

void setup() {
  Serial.begin(115200);
  m1.begin();
  m2.begin();
  m3.begin();
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW); // Active les drivers

  // Initialiser la position à -30°
  int startSteps = angleToSteps(-30.0);
  m1.position = startSteps;
  m2.position = startSteps;
  m3.position = startSteps;
}

void loop() {
  float target[3] = {0};
  char buffer[128] = {0};

  if (Serial.available()) {
    size_t len = Serial.readBytesUntil('\n', buffer, sizeof(buffer) - 1);
    buffer[len] = '\0';
    if (sscanf(buffer, "%f:%f:%f", &target[0], &target[1], &target[2]) == 3) {
        Serial.printf("ANGLES REÇUS: %.2f %.2f %.2f\n", target[0], target[1], target[2]);
        syncMoveAllTo(angleToSteps(target[0]), angleToSteps(target[1]), angleToSteps(target[2]));
        delay(100);
    } else {
        Serial.println("Format invalide");
    }
  }
}

