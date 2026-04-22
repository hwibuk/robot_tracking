#include <Wire.h>
#include <MPU6050_tockn.h>
#include <Adafruit_VL53L0X.h>
#include <SoftwareSerial.h>

// [최적화] 핀 번호 및 상수는 byte로 고정
const byte RX_PIN = 7;
const byte TX_PIN = 8;
const byte IN1 = 6; const byte IN2 = 5; 
const byte IN3 = 10; const byte IN4 = 9;
const byte ENCODER_L = 2; const byte ENCODER_R = 3;
const byte XSHUT = 4;

SoftwareSerial esp(RX_PIN, TX_PIN);
MPU6050 mpu(Wire);
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// 주행 설정
const float TARGET_DIST = 300.0;
const byte BASE_SPD = 120;
const float WHEEL_D = 6.5;
const int TICKS_REV = 40;

// PID (성능 유지를 위해 게인은 그대로)
float Kp = 12.0, Ki = 0.05, Kd = 1.2; 
float err_int = 0, last_err = 0;
float tYaw = 0;

volatile long lTick = 0;
volatile long rTick = 0;
volatile unsigned long lTime = 0, rTime = 0;

unsigned long sTime = 0, pTime = 0;
bool isRun = false; 
long tTick = 0;   

void cL() { unsigned long c = micros(); if (c - lTime > 500) { lTick++; lTime = c; } }
void cR() { unsigned long c = micros(); if (c - rTime > 500) { rTick++; rTime = c; } }

void setup() {
  esp.begin(115200);
  Wire.begin();
  
  mpu.begin();
  mpu.calcGyroOffsets(false); // 가급적 빠르게 셋업

  pinMode(XSHUT, OUTPUT);
  digitalWrite(XSHUT, HIGH);
  lox.begin();

  pinMode(ENCODER_L, INPUT_PULLUP);
  pinMode(ENCODER_R, INPUT_PULLUP);
  attachInterrupt(0, cL, RISING); // 2번 핀
  attachInterrupt(1, cR, RISING); // 3번 핀

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
}

void loop() {
  mpu.update();

  if (esp.available()) {
    char c = esp.read();
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

  // 부호 해결된 로직 유지
  analogWrite(IN1, constrain(BASE_SPD + (int)cor, 0, 255)); digitalWrite(IN2, LOW);
  analogWrite(IN3, constrain(BASE_SPD - (int)cor, 0, 255)); digitalWrite(IN4, LOW);
}

void snd() {
  VL53L0X_RangingMeasurementData_t m;
  lox.rangingTest(&m, false);
  
  // [최적화] 시리얼 출력 문자열 통합 및 F() 매크로
  esp.print(F("D,"));
  esp.print((int)mpu.getAngleZ()); esp.print(F(",")); // 각도는 정수로 변환해 전송 (메모리 절약)
  esp.print(lTick); esp.print(F(","));
  esp.print(rTick); esp.print(F(","));
  esp.println((m.RangeStatus != 4) ? m.RangeMilliMeter : -1);
}

void bck() { digitalWrite(IN1, LOW); analogWrite(IN2, BASE_SPD); digitalWrite(IN3, LOW); analogWrite(IN4, BASE_SPD); }
void lft() { analogWrite(IN1, BASE_SPD); digitalWrite(IN2, LOW); digitalWrite(IN3, LOW); analogWrite(IN4, BASE_SPD); }
void rgt() { digitalWrite(IN1, LOW); analogWrite(IN2, BASE_SPD); analogWrite(IN3, BASE_SPD); digitalWrite(IN4, LOW); }
void stp() { digitalWrite(IN1, LOW); digitalWrite(IN2, LOW); digitalWrite(IN3, LOW); digitalWrite(IN4, LOW); }
