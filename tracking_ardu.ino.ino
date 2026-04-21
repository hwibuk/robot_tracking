#include <Wire.h>
#include <MPU6050.h>
#include <Adafruit_VL53L0X.h>

// --- 핀 설정 ---
const int IN1 = 6; const int IN2 = 5; 
const int IN3 = 10; const int IN4 = 9;
const int ENCODER_L = 2;
const int ENCODER_R = 3;
const int XSHUT_PIN = 4;

// --- 객체 생성 ---
MPU6050 mpu;
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// --- 데이터 변수 ---
volatile long leftPulseCount = 0;
volatile long rightPulseCount = 0;
unsigned long lastTime = 0;

void countLeft() { leftPulseCount++; }
void countRight() { rightPulseCount++; }

void setup() {
  Serial.begin(115200); // ESP32와 연결됨
  Wire.begin();

  // 1. IMU 초기화
  mpu.initialize();
  if (!mpu.testConnection()) Serial.println("MPU6050 Fail");

  // 2. VL53L0X 초기화
  pinMode(XSHUT_PIN, OUTPUT);
  digitalWrite(XSHUT_PIN, HIGH);
  if (!lox.begin()) Serial.println(F("VL53L0X Fail"));

  // 3. 엔코더 설정
  pinMode(ENCODER_L, INPUT_PULLUP);
  pinMode(ENCODER_R, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_L), countLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_R), countRight, RISING);

  // 4. 모터 설정
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  stopMotors();
}

void loop() {
  // [1] ESP32를 통해 들어온 명령 처리
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    handleCommand(cmd);
  }

  // [2] 센서 데이터 주기적 전송 (약 50ms 간격)
  if (millis() - lastTime >= 50) {
    sendSensorData();
    lastTime = millis();
  }
}

void handleCommand(char c) {
  switch (c) {
    case 'w': moveForward();  break;
    case 's': moveBackward(); break;
    case 'a': turnLeft();     break;
    case 'd': turnRight();    break;
    case 'x': stopMotors();   break;
  }
}

void sendSensorData() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);
  int distance = (measure.RangeStatus != 4) ? measure.RangeMilliMeter : -1;

  // PC가 파싱하기 좋게 CSV 형식으로 출력
  Serial.print("DATA,");
  Serial.print(ax); Serial.print(",");
  Serial.print(ay); Serial.print(",");
  Serial.print(az); Serial.print(",");
  Serial.print(gx); Serial.print(",");
  Serial.print(gy); Serial.print(",");
  Serial.print(gz); Serial.print(",");
  Serial.print(leftPulseCount); Serial.print(",");
  Serial.print(rightPulseCount); Serial.print(",");
  Serial.println(distance);
}

void moveForward() {
  analogWrite(IN1, 200); digitalWrite(IN2, LOW);
  analogWrite(IN3, 200); digitalWrite(IN4, LOW);
}
void moveBackward() {
  digitalWrite(IN1, LOW);  analogWrite(IN2, 200);
  digitalWrite(IN3, LOW);  analogWrite(IN4, 200);
}
void turnRight() {
  digitalWrite(IN1, LOW);  analogWrite(IN2, 200);
  analogWrite(IN3, 200); digitalWrite(IN4, LOW);
}
void turnLeft() {
  analogWrite(IN1, 200); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);  analogWrite(IN4, 200);
}
void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}