#include <Wire.h>
#include <MPU6050.h>
#include <Adafruit_VL53L0X.h>
#include <SoftwareSerial.h>

// --- 핀 설정 ---
const int IN1 = 6; const int IN2 = 5; 
const int IN3 = 10; const int IN4 = 9;
const int ENCODER_L = 2;
const int ENCODER_R = 3;
const int XSHUT_PIN = 4;

// [수정] 소프트웨어 시리얼 핀 선언 (7: RX, 8: TX)
const int SOFT_RX = 7;    // 아두이노 D7(RX) <--- ESP32 TX2(17) 연결
const int SOFT_TX = 8;    // 아두이노 D8(TX) ---> ESP32 RX2(16) 연결 (전압 분배 필수!)

SoftwareSerial esp32Serial(SOFT_RX, SOFT_TX);

// --- 객체 생성 ---
MPU6050 mpu;
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// --- 데이터 변수 ---
volatile long leftPulseCount = 0;
volatile long rightPulseCount = 0;
unsigned long lastTime = 0;

////////////////////////////////////////////////////////////////////////////////
// PID 적용을 위한 변수 선언. PID 상수가중치 (실험을 통해 튜닝 필요)
double Kp = 1.0, Ki = 0.1, Kd = 0.01; 

// 목표 속도 및 현재 상태
int targetSpeed = 50; // 루프 주기(50ms)당 목표 펄스 수
double leftInput, leftOutput, rightInput, rightOutput;
double leftError, rightError, leftSumError, rightSumError, leftPrevError, rightPrevError;

int basePWM = 150; // 기본 출력값

bool bRun = false;
////////////////////////////////////////////////////////////////////////////////

void countLeft() { leftPulseCount++; }
void countRight() { rightPulseCount++; }

void setup() {
  Serial.begin(115200); // ESP32와 연결됨
  Wire.begin();

  // ESP32 통신용 (소프트웨어 시리얼)
  esp32Serial.begin(115200);
  Serial.println("System Ready. Communicating with ESP32...");

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

  moveForward();
  stopMotors();
}

void loop() {
  // [1] ESP32를 통해 들어온 명령 처리
  if (esp32Serial.available() > 0) {
    char cmd = esp32Serial.read();
    Serial.print("Command received: ");
    Serial.println(cmd);
    handleCommand(cmd);
  }
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    Serial.print("Command received USB: ");
    Serial.println(cmd);
    handleCommand(cmd);
  }

  // [2] 센서 데이터 주기적 전송 (약 50ms 간격)
  if (millis() - lastTime >= 50) {
    sendSensorData();
    lastTime = millis();
  }

  if (bRun == true)
  {

  }
}

void handleCommand(char c) {
  switch (c) {
    case 'W':
    case 'w': moveForward();  break;
    case 'S':
    case 's': moveBackward(); break;
    case 'A':
    case 'a': turnLeft();     break;
    case 'D':
    case 'd': turnRight();    break;
    case 'X':
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
  Serial.print(ax); Serial.print(",\t");
  Serial.print(ay); Serial.print(",\t");
  Serial.print(az); Serial.print(",\t");
  Serial.print(gx); Serial.print(",\t");
  Serial.print(gy); Serial.print(",\t");
  Serial.print(gz); Serial.print(",\t\t\t");
  Serial.print(leftPulseCount); Serial.print(",\t");
  Serial.print(rightPulseCount); Serial.print(",\t\t\t");
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

////////////////////////////////////////////////////////////////////////////////
// pid 계산, 변수 저장
void computePID() {
  // 1. 현재 속도 계산 (현재 펄스 - 이전 펄스)
  static long prevLeftPulse = 0;
  static long prevRightPulse = 0;
  
  long currentLeft = leftPulseCount - prevLeftPulse;
  long currentRight = rightPulseCount - prevRightPulse;
  
  prevLeftPulse = leftPulseCount;
  prevRightPulse = rightPulseCount;

  // 2. 왼쪽 모터 PID
  leftError = targetSpeed - currentLeft;
  leftSumError += leftError * 0.05; // 50ms 주기 반영
  double leftD = (leftError - leftPrevError) / 0.05;
  leftOutput = (Kp * leftError) + (Ki * leftSumError) + (Kd * leftD);
  leftPrevError = leftError;

  // 3. 오른쪽 모터 PID (동일 방식)
  rightError = targetSpeed - currentRight;
  rightSumError += rightError * 0.05;
  double rightD = (rightError - rightPrevError) / 0.05;
  rightOutput = (Kp * rightError) + (Ki * rightSumError) + (Kd * rightD);
  rightPrevError = rightError;

  // 4. 모터 출력 제한 (0~255) 및 적용
  applyMotorSpeed(basePWM + leftOutput, basePWM + rightOutput);
}

void applyMotorSpeed(int leftPWM, int rightPWM) {
  // 1. 최대/최소값 제한 (PWM 범위: 0 ~ 255)
  leftPWM = constrain(leftPWM, 0, 255);
  rightPWM = constrain(rightPWM, 0, 255);

  // 2. 사동대(Deadband) 처리: 모터가 실제 움직이기 시작하는 최소값 이하일 경우 정지
  // 보통 저가형 DC 모터는 PWM 40~60 이하에서 소리만 나고 돌지 않습니다.
  if (leftPWM < 40) leftPWM = 0;
  if (rightPWM < 40) rightPWM = 0;

  // 3. 하드웨어 출력 (전진 기준)
  // 'w' 명령 시 이 함수가 호출되도록 연결합니다.
  analogWrite(IN1, leftPWM);  digitalWrite(IN2, LOW);
  analogWrite(IN3, rightPWM); digitalWrite(IN4, LOW);
}
////////////////////////////////////////////////////////////////////////////////
