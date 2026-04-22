#include <Wire.h>
#include <MPU6050_tockn.h>
#include <Adafruit_VL53L0X.h>

// 하드웨어 시리얼(0, 1번 핀)을 사용하므로 SoftwareSerial 라이브러리 제거
const byte IN1 = 6; const byte IN2 = 5; 
const byte IN3 = 10; const byte IN4 = 9;
const byte ENCODER_L = 2; const byte ENCODER_R = 3;
const byte XSHUT = 4;

MPU6050 mpu(Wire);
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// 주행 설정
const float TARGET_DIST = 300.0;
const byte BASE_SPD = 120;
const float WHEEL_D = 6.5;
const int TICKS_REV = 40;

// PID
float Kp = 12.0, Ki = 0.05, Kd = 1.2; 
float err_int = 0, last_err = 0;
float tYaw = 0;

volatile long lTick = 0, rTick = 0;
volatile unsigned long lTime = 0, rTime = 0;
unsigned long sTime = 0, pTime = 0;
bool isRun = false; 
long tTick = 0;   

void cL() { unsigned long c = micros(); if (c - lTime > 500) { lTick++; lTime = c; } }
void cR() { unsigned long c = micros(); if (c - rTime > 500) { rTick++; rTime = c; } }

void setup() {
  // 하드웨어 시리얼은 115200까지도 매우 안정적입니다.
  // ESP32의 Serial2.begin 속도와 일치시켜주세요.
  Serial.begin(115200); 
  Wire.begin();
  
  mpu.begin();
  mpu.calcGyroOffsets(false);

  pinMode(XSHUT, OUTPUT);
  digitalWrite(XSHUT, HIGH);
  lox.begin();

  pinMode(ENCODER_L, INPUT_PULLUP);
  pinMode(ENCODER_R, INPUT_PULLUP);
  attachInterrupt(0, cL, RISING); 
  attachInterrupt(1, cR, RISING); 

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
}

void loop() {
  mpu.update();

  // ESP32에서 오는 명령 수신
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'w') {
      tTick = (long)((TARGET_DIST / (WHEEL_D * 3.14159)) * TICKS_REV);
      lTick = 0; rTick = 0; err_int = 0; last_err = 0;
      tYaw = mpu.getAngleZ(); 
      pTime = millis();          
      isRun = true;
    } else if (c == 'x') { isRun = false; stp(); }
    else if (c == 's') { isRun = false; bck(); }
    else if (c == 'a') { isRun = false; lft(); }
    else if (c == 'd') { isRun = false; rgt(); }
  }

  if (isRun) drv();

  // 송신 주기를 100ms로 조절하여 데이터 흐름을 안정화
  if (millis() - sTime >= 50) {
    snd();
    sTime = millis();
  }
}

void drv() {
  unsigned long c = millis();
  float dt = (c - pTime) / 1000.0;
  if (dt <= 0) return;
  pTime = c;

  if ((lTick + rTick) / 2 >= tTick) { isRun = false; stp(); return; }

  float err = tYaw - mpu.getAngleZ();
  err_int += err * dt;
  float cor = (Kp * err) + (Ki * err_int) + (Kd * (err - last_err) / dt);
  last_err = err;

  analogWrite(IN1, constrain(BASE_SPD + (int)cor, 0, 255)); digitalWrite(IN2, LOW);
  analogWrite(IN3, constrain(BASE_SPD - (int)cor, 0, 255)); digitalWrite(IN4, LOW);
}

void snd() {
  VL53L0X_RangingMeasurementData_t m;
  lox.rangingTest(&m, false);
  
  // Serial로 직접 전송 (ESP32의 Serial2로 들어감)
  Serial.print(F("DATA,"));
  Serial.print((int)mpu.getAngleZ()); Serial.print(F(","));
  Serial.print(lTick); Serial.print(F(","));
  Serial.print(rTick); Serial.print(F(","));
  Serial.println((m.RangeStatus != 4) ? m.RangeMilliMeter : -1);
}

void bck() { digitalWrite(IN1, LOW); analogWrite(IN2, BASE_SPD); digitalWrite(IN3, LOW); analogWrite(IN4, BASE_SPD); }
void lft() { analogWrite(IN1, BASE_SPD); digitalWrite(IN2, LOW); digitalWrite(IN3, LOW); analogWrite(IN4, BASE_SPD); }
void rgt() { digitalWrite(IN1, LOW); analogWrite(IN2, BASE_SPD); analogWrite(IN3, BASE_SPD); digitalWrite(IN4, LOW); }
void stp() { digitalWrite(IN1, LOW); digitalWrite(IN2, LOW); digitalWrite(IN3, LOW); digitalWrite(IN4, LOW); }
