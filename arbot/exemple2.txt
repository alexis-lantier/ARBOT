// Brochage ESP32
#define STEP1 37
#define DIR1 38

#define STEP2 39
#define DIR2 40

#define STEP3 41
#define DIR3 42

#define ENABLE_PIN 36

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
StepperMotor motor1(STEP1, DIR1);
StepperMotor motor2(STEP2, DIR2);
StepperMotor motor3(STEP3, DIR3);

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

// --- UART reception & parsing ---
#define UART_BAUD 115200
#define UART Serial

String uartBuffer = "";

uint8_t calcCRC(const String& data) {
  uint8_t crc = 0;
  for (size_t i = 0; i < data.length(); ++i) {
    crc += (uint8_t)data[i];
  }
  return crc;
}

#define UART_BAUD 115200
#define UART Serial
#define MOTMAGIC 0xA5

void handleUART() {
  static uint8_t buf[5];
  static uint8_t idx = 0;

  while (UART.available()) {
    uint8_t b = UART.read();
    if (idx == 0 && b != MOTMAGIC) continue; // attend magic
    buf[idx++] = b;
    if (idx == 5) {
      uint8_t crc = (buf[0] + buf[1] + buf[2] + buf[3]) & 0xFF;
      if (crc == buf[4]) {
        float a1 = buf[1];
        float a2 = buf[2];
        float a3 = buf[3];
        int s1 = angleToSteps(a1);
        int s2 = angleToSteps(a2);
        int s3 = angleToSteps(a3);
        motor1.prepareMoveTo(s1);
        motor2.prepareMoveTo(s2);
        motor3.prepareMoveTo(s3);
        int totalSteps = max(abs(motor1.target - motor1.position), max(abs(motor2.target - motor2.position), abs(motor3.target - motor3.position)));
        for (int step = 0; step < totalSteps; step++) {
          if (motor1.isMoving()) motor1.tick();
          if (motor2.isMoving()) motor2.tick();
          if (motor3.isMoving()) motor3.tick();
          delayMicroseconds(getRampDelay(step, totalSteps));
        }
        UART.write("OK\n");
      } else {
        UART.write("ERR\n");
      }
      idx = 0;
    }
  }
}

// --- Fin UART ---

void setup() {
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);  // Active les drivers

  motor1.begin();
  motor2.begin();
  motor3.begin();

  UART.begin(UART_BAUD);

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
  handleUART();
}