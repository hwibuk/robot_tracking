#include <WiFi.h>
#include <WebServer.h>

const char* ssid = "asia-edu_2G";
const char* password = "12345678";

// 아두이노 통신용 Serial2 핀
#define TX2 17
#define RX2 16

// 상태 정의
enum MotorStatus { STOP, FORWARD, BACKWARD, LEFT, RIGHT };
MotorStatus motor_status = STOP;

int current_speed = 5;
WebServer server(80);

void setup() {
  Serial.begin(115200);
  // 아두이노와 통신할 시리얼 포트 (9600 보드레이트)
  Serial2.begin(115200, SERIAL_8N1, RX2, TX2); 

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { 
    delay(500); 
    Serial.print("."); 
  }
  
  Serial.println("\nWiFi Connected. IP: ");
  Serial.println(WiFi.localIP());

  // 핸들러 등록
  server.on("/", handle_OnConnect);
  server.on("/forward", handle_forward);
  server.on("/backward", handle_backward);
  server.on("/left", handle_left);   // 좌회전 추가
  server.on("/right", handle_right); // 우회전 추가
  server.on("/stop", handle_stop);
  server.on("/speed", handle_speed);

  server.begin();
}

void loop() {
  server.handleClient();
}

// --- 시리얼 전송 함수 ---
void sendToArduino(String cmd) {
  Serial2.print(cmd); // 아두이노로 한 글자 명령 전달
  Serial.print("Sent to Arduino: "); Serial.println(cmd);
}

// --- 핸들러 함수들 ---
void handle_forward() {
  motor_status = FORWARD;
  sendToArduino("W"); // 전진: W
  server.send(200, "text/html", SendHTML());
}

void handle_backward() {
  motor_status = BACKWARD;
  sendToArduino("S"); // 후진: S
  server.send(200, "text/html", SendHTML());
}

void handle_left() {
  motor_status = LEFT;
  sendToArduino("A"); // 좌회전: A
  server.send(200, "text/html", SendHTML());
}

void handle_right() {
  motor_status = RIGHT;
  sendToArduino("D"); // 우회전: D
  server.send(200, "text/html", SendHTML());
}

void handle_stop() {
  motor_status = STOP;
  sendToArduino("X"); // 정지: X (S가 후진이므로 X로 설정)
  server.send(200, "text/html", SendHTML());
}

void handle_speed() {
  if (server.hasArg("val")) {
    String speed = server.arg("val");
    current_speed = speed.toInt();
    sendToArduino(speed); // 속도: 0~9
  }
  server.send(200, "text/html", SendHTML());
}

void handle_OnConnect() {
  server.send(200, "text/html", SendHTML());
}

// --- 웹 페이지 UI 생성 ---
String SendHTML() {
  String ptr = "<!DOCTYPE html><html><head>";
  ptr += "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">";
  ptr += "<style>html{font-family:sans-serif; text-align:center;} ";
  ptr += ".btn{display:inline-block; padding:15px 25px; margin:5px; font-size:18px; text-decoration:none; color:white; border-radius:5px; min-width:80px;}";
  ptr += ".fwd{background:#2ecc71;} .bwd{background:#e67e22;} .side{background:#34495e;} .stp{background:#e74c3c;} .spd{background:#3498db; padding:10px; min-width:30px;}";
  ptr += "</style></head><body>";
  ptr += "<h1>Robot Control</h1>";
  
  // 제어 패널 UI (상하좌우 배치)
  ptr += "<div><a class=\"btn fwd\" href=\"/forward\">FORWARD (W)</a></div>";
  ptr += "<div>";
  ptr += "<a class=\"btn side\" href=\"/left\">LEFT (A)</a>";
  ptr += "<a class=\"btn stp\" href=\"/stop\">STOP (X)</a>";
  ptr += "<a class=\"btn side\" href=\"/right\">RIGHT (D)</a>";
  ptr += "</div>";
  ptr += "<div><a class=\"btn bwd\" href=\"/backward\">BACKWARD (S)</a></div>";
  ptr += "<hr>";
  
  // 속도 버튼
  ptr += "<h3>Speed Control</h3>";
  for(int i=0; i<=9; i++) {
    ptr += "<a class=\"btn spd\" href=\"/speed?val=" + String(i) + "\">" + String(i) + "</a>";
    if(i==4) ptr += "<br>";
  }
  
  // 현재 상태 표시
  String status_text;
  switch(motor_status) {
    case FORWARD: status_text = "FORWARD"; break;
    case BACKWARD: status_text = "BACKWARD"; break;
    case LEFT: status_text = "LEFT"; break;
    case RIGHT: status_text = "RIGHT"; break;
    default: status_text = "STOP"; break;
  }
  
  ptr += "<p><strong>Status:</strong> " + status_text + " | <strong>Speed:</strong> " + String(current_speed) + "</p>";
  ptr += "</body></html>";
  return ptr;
}