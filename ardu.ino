#include <Wire.h>
#include <MPU6050_tockn.h>

// --- [핀 및 상수 설정] ---
const byte IN1 = 6; const byte IN2 = 5;
const byte IN3 = 10; const byte IN4 = 9;
const byte ENCODER_L = 2; const byte ENCODER_R = 3;

const float WHEEL_D = 6.5;
const int TICKS_REV = 40;

// 출력 범위 설정
const int MIN_PWM = 90;  // 최소 출력
const int MAX_PWM = 150; // 최대 출력
const byte MANUAL_BASE_SPD = 110;

MPU6050 mpu(Wire);

// PID 설정
float Kp = 25.0, Ki = 0.05, Kd = 1.0;
float err_int = 0, last_err = 0;

// 제어 변수
long targetTick = 0;
float targetYaw = 0;
bool isMovingAuto = false;  
bool isMovingManual = false;
bool isOriented = false;    
int manualDirection = 1;     // 1: 전진, -1: 후진, 0: 제자리 회전

volatile long lTick = 0, rTick = 0;
volatile unsigned long lTime = 0, rTime = 0;
unsigned long pTime = 0;
unsigned long lastStatusTime = 0;

void cL() { unsigned long c = micros(); if (c - lTime > 500) { lTick++; lTime = c; } }
void cR() { unsigned long c = micros(); if (c - rTime > 500) { rTick++; rTime = c; } }

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(5); // 시리얼 파싱 지연 방지를 위한 타임아웃 설정
  Wire.begin();
  mpu.begin();
  mpu.calcGyroOffsets(false);
  
  pinMode(ENCODER_L, INPUT_PULLUP);
  pinMode(ENCODER_R, INPUT_PULLUP);
  attachInterrupt(0, cL, RISING);
  attachInterrupt(1, cR, RISING);
  
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  stopAll();
}

void loop() {
  mpu.update();

  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 'D') {
      float dist = Serial.parseFloat();
      if (Serial.read() == ',') {
        if (Serial.read() == 'A') {
          float angle_err = Serial.parseFloat();
          
          // 레이턴시 측정을 위한 T 파라미터 처리 (추가된 부분)
          if (Serial.read() == ',') {
            if (Serial.read() == 'T') {
              String timestamp = Serial.readStringUntil('\n');
              Serial.println(timestamp); // 받은 타임스탬프 즉시 에코
            }
          }
          
          updateTarget(dist, angle_err);
        }
      }
    } else {
      handleManualCommand(cmd);
    }
  }

  if (isMovingAuto || isMovingManual) {
    if (isMovingAuto && ((lTick + rTick) / 2 >= targetTick)) {
      stopAll();
      Serial.println("STATUS:ARRIVED");
    } else {
      // a, d로 인한 제자리 회전(manualDirection=0) 시에도 PID 로직을 태움
      applyPIDDrive();
    }
  }

  if (millis() - lastStatusTime > 100) {
    lastStatusTime = millis();
    reportStatus();
  }
}

void updateTarget(float dist, float angle_err) {
  targetYaw = mpu.getAngleZ() + angle_err;
  long currentAvg = (lTick + rTick) / 2;
  long addedTick = (long)((dist / (WHEEL_D * 3.14159)) * TICKS_REV);
  targetTick = currentAvg + addedTick;
  
  if (abs(angle_err) > 5.0) {
    isOriented = false;
    err_int = 0;
  }
  
  isMovingAuto = true;
  isMovingManual = false;
}

void applyPIDDrive() {
  unsigned long c = millis();
  float dt = (c - pTime) / 1000.0;
  if (dt <= 0) return;
  pTime = c;

  float currentYaw = mpu.getAngleZ();
  float err = targetYaw - currentYaw;
  
  if (err > 180) err -= 360;
  else if (err < -180) err += 360;

  if (abs(err) < 10.0) {
    err_int += err * dt;
    err_int = constrain(err_int, -50, 50);
  } else {
    err_int = 0;
  }

  float correction = (Kp * err) + (Ki * err_int) + (Kd * (err - last_err) / dt);
  last_err = err;

  int base = 0;
  if (isMovingAuto) {
    if (!isOriented) {
      base = 0;
      if (abs(err) > 2.0) {
        if (correction > 0 && correction < MIN_PWM) correction = MIN_PWM;
        else if (correction < 0 && correction > -MIN_PWM) correction = -MIN_PWM;
      }
      if (abs(err) < 2.5) {
        isOriented = true;
        err_int = 0;
      }
    } else {
      base = 100;
    }
  } else {
    base = MANUAL_BASE_SPD;
  }

  // manualDirection이 0이면 base(전진력)가 0이 되어 제자리 회전만 남음
  base *= manualDirection;
  int leftSpd = base + (int)correction;
  int rightSpd = base - (int)correction;

  leftSpd = applyRange(leftSpd);
  rightSpd = applyRange(rightSpd);

  moveRaw(leftSpd, rightSpd);
}

int applyRange(int speed) {
  if (speed == 0) return 0;
  bool positive = speed > 0;
  int absSpeed = abs(speed);
  if (absSpeed < MIN_PWM) absSpeed = MIN_PWM;
  if (absSpeed > MAX_PWM) absSpeed = MAX_PWM;
  return positive ? absSpeed : -absSpeed;
}

void reportStatus() {
  if (!isMovingAuto && !isMovingManual) Serial.println("STATUS:IDLE");
  else if (isMovingAuto) {
    if (!isOriented) Serial.println("STATUS:TURNING");
    else Serial.println("STATUS:DRIVING");
  }
}

void moveRaw(int left, int right) {
  if (left >= 0) { analogWrite(IN1, left); digitalWrite(IN2, LOW); }
  else { digitalWrite(IN1, LOW); analogWrite(IN2, abs(left)); }
  if (right >= 0) { analogWrite(IN3, right); digitalWrite(IN4, LOW); }
  else { digitalWrite(IN3, LOW); analogWrite(IN4, abs(right)); }
}

void stopAll() {
  isMovingAuto = false; isMovingManual = false;
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  err_int = 0;
}

// [수정된 부분] a, d 명령 시 targetYaw를 변경하여 PID 추적 방식으로 회전
void handleManualCommand(char cmd) {
  if (cmd == 'w' || cmd == 's' || cmd == 'a' || cmd == 'd') {
    lTick = 0; rTick = 0; err_int = 0; last_err = 0;
    pTime = millis();
    isOriented = true;
    isMovingManual = true;
    isMovingAuto = false;

    if (cmd == 'w' || cmd == 's') {
      targetYaw = mpu.getAngleZ(); // 현재 방향 유지
      manualDirection = (cmd == 'w') ? 1 : -1;
    }
    else if (cmd == 'a' || cmd == 'd') {
      // 15도씩 목표 각도를 틀어줌 (PID가 알아서 회전시킴)
      float turnStep = (cmd == 'a') ? 15.0 : -15.0;
      targetYaw = mpu.getAngleZ() + turnStep;
      manualDirection = 0; // 전진/후진 속도를 0으로 하여 제자리 회전 유도
    }
  }
  else if (cmd == 'x' || cmd == ' ') {
    stopAll();
  }
}
