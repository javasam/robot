// Robot v1.0 + Dock: авто-объезд с серво + упрощённая парковка в прямоугольник
// Логика: после патруля робот ищет зону парковки, и как только видит чёрную линию
// любым датчиком L/C/R — сразу заезжает внутрь и останавливается.

#include <Arduino.h>
#include <Servo.h>

// ----- Пины -----
#define leftMotorCtrl 4
#define leftMotorPwr  6
#define rightMotorCtrl 2
#define rightMotorPwr  5

#define trig 12
#define echo 13

#define leftLineTrk  11
#define midLineTrk    7
#define rightLineTrk  8

#define soundPin 9
#define servoPin 10

// ----- Состояние -----
int baseSpeed = 140;
unsigned long tPrint = 0;

Servo scanServo;
const int SERVO_CENTER = 90;
const int SERVO_LEFT   = 150;
const int SERVO_RIGHT  = 30;

bool autoMode = true;

// Таймер патруля
unsigned long patrolStartTime = 0;
const unsigned long PATROL_TIME_MS = 30000; // 30 секунд

// Режимы верхнего уровня
enum Mode {
  MODE_AUTO,
  MODE_DOCK_SEARCH,
  MODE_DOCK_PARK
};
Mode mode = MODE_AUTO;

// Для докинга
unsigned long dockPhaseStart = 0;

//По факту большее число = медленнее, поэтому докинг делаем "медленным" большим числом
const int DOCK_SPEED = 160;

// Сколько ехать вперёд после обнаружения линии, чтобы заехать внутрь прямоугольника
const unsigned long PARK_EXTRA_MS = 450;

// ----- Структуры -----
struct LineState {
  int L;
  int C;
  int R;
};

// ----- Движение -----
void driveLR(int left, int right) {
  left  = constrain(left,  -255, 255);
  right = constrain(right, -255, 255);

  if (left == 0) {
    digitalWrite(leftMotorCtrl, LOW);
    analogWrite(leftMotorPwr, 0);
  } else {
    digitalWrite(leftMotorCtrl, (left > 0) ? HIGH : LOW);
    analogWrite(leftMotorPwr, abs(left));
  }

  if (right == 0) {
    digitalWrite(rightMotorCtrl, LOW);
    analogWrite(rightMotorPwr, 0);
  } else {
    digitalWrite(rightMotorCtrl, (right > 0) ? HIGH : LOW);
    analogWrite(rightMotorPwr, abs(right));
  }
}

void stopMotors() {
  digitalWrite(leftMotorCtrl, LOW);
  digitalWrite(rightMotorCtrl, LOW);
  analogWrite(leftMotorPwr, 0);
  analogWrite(rightMotorPwr, 0);
}

// ----- Датчики -----
float readDistanceCm() {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  unsigned long duration = pulseIn(echo, HIGH, 30000UL);
  if (duration == 0) return -1.0;
  return duration / 58.20;
}

LineState readLineLCR() {
  LineState s;
  s.L = digitalRead(leftLineTrk);
  s.C = digitalRead(midLineTrk);
  s.R = digitalRead(rightLineTrk);
  return s;
}

// ----- Звук -----
void beepShort() {
  tone(soundPin, 880, 120);
}

// ----- Серво: измерить на угле -----
float measureAtAngle(int angleDeg) {
  scanServo.write(angleDeg);
  delay(400);
  float d = readDistanceCm();
  delay(50);
  return d;
}

// ----- Help -----
void printHelp() {
  Serial.println(F("\nCommands:"));
  Serial.println(F("  f/b/l/r/s  -> manual forward/back/left/right/stop (auto OFF)"));
  Serial.println(F("  + / -      -> speed up / down"));
  Serial.println(F("  p          -> beep"));
  Serial.println(F("  a          -> toggle auto-mode on/off"));
  Serial.println(F("  d          -> start docking (search line)"));
  Serial.println(F("  h          -> help"));
}

// ----- Докинг: поиск -----
void dockSearchStep() {
  LineState s = readLineLCR();

  // Как только увидели чёрную линию — сразу начинаем парковку
  if (s.L || s.C || s.R) {
    Serial.println(F("Dock: line detected -> PARK"));
    stopMotors();
    dockPhaseStart = millis();
    mode = MODE_DOCK_PARK;
    return;
  }

  // Если линии нет — ищем её, но не врезаемся в стены
  const float obstacleDist = 10.0;
  float dCenter = readDistanceCm();

  if (dCenter > 0 && dCenter < obstacleDist) {
    Serial.println(F("Dock: obstacle during search"));

    stopMotors();
    delay(150);

    // Немного назад
    driveLR(-DOCK_SPEED, -DOCK_SPEED);
    delay(180);
    stopMotors();
    delay(80);

    // Смотрим влево и вправо
    float dLeft  = measureAtAngle(SERVO_LEFT);
    float dRight = measureAtAngle(SERVO_RIGHT);
    scanServo.write(SERVO_CENTER);

    Serial.print(F("Dock search dLeft="));
    Serial.print(dLeft, 1);
    Serial.print(F("  dRight="));
    Serial.println(dRight, 1);

    // Если везде тесно — разворачиваемся
    if ((dLeft < 0 && dRight < 0) ||
        (dLeft > 0 && dLeft < obstacleDist &&
         dRight > 0 && dRight < obstacleDist)) {
      driveLR(DOCK_SPEED, -DOCK_SPEED);
      delay(400);
      stopMotors();
      delay(80);
    } else {
      if (dLeft < 0) dLeft = 0;
      if (dRight < 0) dRight = 0;

      if (dLeft > dRight) {
        driveLR(-DOCK_SPEED, DOCK_SPEED);
      } else {
        driveLR(DOCK_SPEED, -DOCK_SPEED);
      }
      delay(280);
      stopMotors();
      delay(80);
    }
  } else {
    // Свободно — едем вперёд и ищем линию
    driveLR(DOCK_SPEED, DOCK_SPEED);
  }
}

// ----- Докинг: парковка внутрь прямоугольника -----
void dockParkStep() {
  // Заезжаем внутрь ограниченное время, потом стоп
  driveLR(DOCK_SPEED, DOCK_SPEED);

  if (millis() - dockPhaseStart > PARK_EXTRA_MS) {
    stopMotors();
    Serial.println(F("Dock: parked -> STOP"));

    beepShort();
    autoMode = false;
    mode = MODE_AUTO;
    stopMotors();
  }
}

// ----- Setup -----
void setup() {
  Serial.begin(9600);

  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);

  pinMode(leftMotorCtrl, OUTPUT);
  pinMode(leftMotorPwr, OUTPUT);
  pinMode(rightMotorCtrl, OUTPUT);
  pinMode(rightMotorPwr, OUTPUT);

  pinMode(leftLineTrk, INPUT);
  pinMode(midLineTrk, INPUT);
  pinMode(rightLineTrk, INPUT);

  pinMode(soundPin, OUTPUT);

  scanServo.attach(servoPin);
  scanServo.write(SERVO_CENTER);
  delay(300);

  stopMotors();
  printHelp();
  beepShort();

  mode = MODE_AUTO;
  autoMode = true;
  patrolStartTime = millis();
}

// ----- Loop -----
void loop() {
  // Телеметрия
  if (millis() - tPrint >= 200) {
    tPrint = millis();

    float d = readDistanceCm();
    LineState s = readLineLCR();

    Serial.print(F("dist_cm="));
    Serial.print(d, 1);
    Serial.print(F("  LCR="));
    Serial.print(s.L);
    Serial.print(s.C);
    Serial.print(s.R);
    Serial.print(F("  speed="));
    Serial.print(baseSpeed);
    Serial.print(F("  auto="));
    Serial.print(autoMode ? F("ON") : F("OFF"));
    Serial.print(F("  mode="));
    if (mode == MODE_AUTO) Serial.println(F("AUTO"));
    else if (mode == MODE_DOCK_SEARCH) Serial.println(F("DOCK_SEARCH"));
    else if (mode == MODE_DOCK_PARK) Serial.println(F("DOCK_PARK"));
  }

  // Обработка команд из Serial
  if (Serial.available()) {
    char c = (char)Serial.read();

    if (c == 'f') {
      autoMode = false;
      mode = MODE_AUTO;
      driveLR(baseSpeed, baseSpeed);
    }
    else if (c == 'b') {
      autoMode = false;
      mode = MODE_AUTO;
      driveLR(-baseSpeed, -baseSpeed);
    }
    else if (c == 'l') {
      autoMode = false;
      mode = MODE_AUTO;
      driveLR(-baseSpeed, baseSpeed);
    }
    else if (c == 'r') {
      autoMode = false;
      mode = MODE_AUTO;
      driveLR(baseSpeed, -baseSpeed);
    }
    else if (c == 's') {
      autoMode = false;
      mode = MODE_AUTO;
      stopMotors();
    }
    else if (c == '-') {
      baseSpeed = min(255, baseSpeed + 20);
      Serial.print(F("baseSpeed = "));
      Serial.println(baseSpeed);
      driveLR(baseSpeed, baseSpeed);
    }
    else if (c == '+') {
      baseSpeed = max(60, baseSpeed - 20);
      Serial.print(F("baseSpeed = "));
      Serial.println(baseSpeed);
      driveLR(baseSpeed, baseSpeed);
    }
    else if (c == 'p') {
      beepShort();
    }
    else if (c == 'a') {
      autoMode = !autoMode;
      mode = MODE_AUTO;
      Serial.print(F("autoMode = "));
      Serial.println(autoMode ? F("ON") : F("OFF"));

      if (!autoMode) stopMotors();
      if (autoMode) {
        scanServo.write(SERVO_CENTER);
        patrolStartTime = millis();
      }
    }
    else if (c == 'd') {
      autoMode = false;
      stopMotors();
      mode = MODE_DOCK_SEARCH;
      dockPhaseStart = millis();
      Serial.println(F("Dock mode: SEARCH"));
    }
    else if (c == 'h') {
      printHelp();
    }
  }

  // Таймер патруля: 30 сек AUTO -> автоматический докинг
  if (mode == MODE_AUTO && autoMode &&
      millis() - patrolStartTime > PATROL_TIME_MS) {
    Serial.println(F("Patrol timeout -> start docking"));
    autoMode = false;
    stopMotors();
    mode = MODE_DOCK_SEARCH;
    dockPhaseStart = millis();
  }

  // FSM: докинг имеет приоритет над обычным авто-режимом
  if (mode == MODE_DOCK_SEARCH) {
    dockSearchStep();
  } else if (mode == MODE_DOCK_PARK) {
    dockParkStep();
  } else {
    // Обычный авто-режим с оглядкой серво
    if (autoMode) {
      float dCenter = readDistanceCm();
      const float obstacleDist = 25.0;

      if (dCenter > 0 && dCenter < obstacleDist) {
        stopMotors();
        delay(200);

        driveLR(-baseSpeed, -baseSpeed);
        delay(300);
        stopMotors();
        delay(100);

        float dLeft  = measureAtAngle(SERVO_LEFT);
        float dRight = measureAtAngle(SERVO_RIGHT);
        scanServo.write(SERVO_CENTER);

        Serial.print(F("dLeft="));
        Serial.print(dLeft, 1);
        Serial.print(F("  dRight="));
        Serial.println(dRight, 1);

        if ((dLeft < 0 && dRight < 0) ||
            (dLeft > 0 && dLeft < obstacleDist &&
             dRight > 0 && dRight < obstacleDist)) {
          driveLR(baseSpeed, -baseSpeed);
          delay(500);
          stopMotors();
          delay(100);
        } else {
          if (dLeft < 0) dLeft = 0;
          if (dRight < 0) dRight = 0;

          if (dLeft > dRight) {
            driveLR(-baseSpeed, baseSpeed);
          } else {
            driveLR(baseSpeed, -baseSpeed);
          }
          delay(400);
          stopMotors();
          delay(100);
        }
      } else {
        driveLR(baseSpeed, baseSpeed);
      }
    }
  }
}
