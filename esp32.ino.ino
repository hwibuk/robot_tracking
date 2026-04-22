#include <WiFi.h>
#include <WiFiUdp.h>

// 1. 와이파이 설정
const char* ssid = "asia-edu_2G";
const char* password = "12345678";

// 2. UDP 설정
WiFiUDP udp;
const unsigned int localUdpPort = 12345;
IPAddress remoteIP;
unsigned int remotePort;
bool firstPacketReceived = false;

void setup() {
  Serial.begin(115200); // 디버깅용 (USB)
  Serial2.begin(115200, SERIAL_8N1, 16, 17); // 아두이노 연결용 (RX2:16, TX2:17)

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  Serial.print("ESP32 IP Address: ");
  Serial.println(WiFi.localIP());

  udp.begin(localUdpPort);
}

void loop() {
  // [A] PC(UDP) -> 아두이노(Serial2)
  int packetSize = udp.parsePacket();
  if (packetSize) {
    remoteIP = udp.remoteIP();
    remotePort = udp.remotePort();
    firstPacketReceived = true; // PC 주소 확보

    while (udp.available()) {
      char c = udp.read();
      Serial2.write(c); // UDP 데이터를 아두이노로 전달
    }
  }

  // [B] 아두이노(Serial2) -> PC(UDP)
  if (Serial2.available() && firstPacketReceived) {
    udp.beginPacket(remoteIP, remotePort);
    while (Serial2.available()) {
      udp.write(Serial2.read()); // 아두이노 데이터를 PC로 전달
    }
    udp.endPacket();
  }
}
