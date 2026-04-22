#include <Wire.h>
#include <MPU6050_tockn.h>
#include <Adafruit_VL53L0X.h>

// --- [사용자 설정 섹션] ---
const float TARGET_DIST = 200.0;   
const float TARGET_ANGLE = 90.0;  // 목표로 삼을 절대 방위 (0~359)
const byte BASE_SPD = 130;        
const byte TURN_SPD_MAX = 110;    
const byte TURN_SPD_MIN = 75;     // 역회전 시 진동 방지를 위해 낮게 설정
const float WHEEL_D = 6.5;        
const int TICKS_REV = 40;         
// ------------------------

const byte IN1 = 6; const byte IN2 = 5; 
const byte IN3 = 10; const byte IN4 = 9;
const byte ENCODER_L = 2; const byte ENCODER_R = 3;
const byte XSHUT = 4;

MPU6050 mpu(Wire);
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

float Kp = 15.0, Ki = 0.05, Kd = 1.5; 
float err_int = 0, last_err = 0;
float tYaw = 0;
float lastYawForStop = 0; // 정지 판정용 이전 각도 저장

volatile long lTick = 0, rTick = 0;
volatile unsigned long lTime = 0, rTime = 0;
unsigned long sTime = 0, pTime = 0;

byte mode = 0; // 0: 정지, 1: 직진, 2: 회전
long tTick = 0;   

void cL() { unsigned long c = micros(); if (c - lTime > 500) { lTick++; lTime = c; } }
void cR() { unsigned long c = micros(); if (c - rTime > 500) { rTick++; rTime = c; } }

float getNormalizedAngle() {
  float angle = mpu.getAngleZ();
  float normalized = fmod(angle, 360.0);
  if (normalized < 0) normalized += 360.0;
  return normalized;
}

void setup() {
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

  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'x') { mode = 0; stp(); }
    else if (mode == 0) {
      if (c == 'w') { 
        resetVariables();
        tTick = (long)((TARGET_DIST / (WHEEL_D * 3.14159)) * TICKS_REV);
        tYaw = getNormalizedAngle(); 
        mode = 1; 
      } 
      else if (c == 'a') { 
        resetVariables(); 
        tYaw = TARGET_ANGLE; 
        mode = 2; 
      }
    }
  }

  if (mode == 1) drv();      
  else if (mode == 2) trn(); 

  if (millis() - sTime >= 100) {
    snd();
    sTime = millis();
  }
}

void resetVariables() {
  lTick = 0; rTick = 0; err_int = 0; last_err = 0; pTime = millis();
  lastYawForStop = getNormalizedAngle();
}

void drv() {
  unsigned long c = millis(); float dt = (c - pTime) / 1000.0; if (dt <= 0) return; pTime = c;
  if ((lTick + rTick) / 2 >= tTick) { mode = 0; stp(); return; }
  
  float currentYaw = getNormalizedAngle();
  float err = tYaw - currentYaw;
  if (err > 180) err -= 360;
  else if (err < -180) err += 360;

  err_int += err * dt;
  float cor = (Kp * err) + (Ki * err_int) + (Kd * (err - last_err) / dt);
  last_err = err;
  analogWrite(IN1, constrain(BASE_SPD + (int)cor, 0, 255)); digitalWrite(IN2, LOW);
  analogWrite(IN3, constrain(BASE_SPD - (int)cor, 0, 255)); digitalWrite(IN4, LOW);
}

void trn() {
  float currentYaw = getNormalizedAngle();
  float err = tYaw - currentYaw;

  // 최단 경로 보정
  if (err > 180) err -= 360;
  else if (err < -180) err += 360;

  // 정지 판정 로직: 에러가 작고 + 회전 속도(변화량)가 거의 없을 때
  float yawDiff = abs(currentYaw - lastYawForStop);
  lastYawForStop = currentYaw;

  if (abs(err) < 2.0 && yawDiff < 0.2) { 
    mode = 0;
    stp();
    return;
  }

  // 속도 수식에 TURN_SPD_MIN 변수 적용
  int spd = constrain(abs(err) * 3 + TURN_SPD_MIN, TURN_SPD_MIN, TURN_SPD_MAX);

  if (err > 0) { // 목표가 왼쪽(CCW)에 있음 -> 좌회전
    analogWrite(IN1, spd); digitalWrite(IN2, LOW); 
    digitalWrite(IN3, LOW); analogWrite(IN4, spd); 
  } else { // 목표가 오른쪽(CW)에 있음 -> 우회전 (오버슈트 시 일로 들어옴)
    digitalWrite(IN1, LOW); analogWrite(IN2, spd); 
    analogWrite(IN3, spd); digitalWrite(IN4, LOW); 
  }
}

void snd() {
  VL53L0X_RangingMeasurementData_t m;
  lox.rangingTest(&m, false);
  Serial.print(F("DATA,"));
  Serial.print((int)getNormalizedAngle()); Serial.print(F(","));
  Serial.print(lTick); Serial.print(F(","));
  Serial.print(rTick); Serial.print(F(","));
  Serial.println((m.RangeStatus != 4) ? m.RangeMilliMeter : -1);
}

void stp() { 
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW); 
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW); 
}
