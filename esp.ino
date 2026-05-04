#include <WiFi.h>
#include <WiFiUdp.h>

const char* ssid = "asia-edu_2G";
const char* password = "12345678";

WiFiUDP udp;
const unsigned int localUdpPort = 12345;
IPAddress remoteIP;
unsigned int remotePort;
bool firstPacketReceived = false;

// [최적화] 시리얼 데이터를 모으기 위한 버퍼
char serialBuffer[256];
int bufferIndex = 0;
unsigned long lastSerialReadTime = 0;

void setup() {
  Serial.begin(115200);
  // ESP32의 하드웨어 시리얼 버퍼 크기를 늘려 병목 방지
  Serial2.begin(115200, SERIAL_8N1, 16, 17);
  
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println("\nWiFi connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  udp.begin(localUdpPort);
}

void loop() {
  // --- [A] PC -> 아두이노 (속도 최적화) ---
  int packetSize = udp.parsePacket();
  if (packetSize) {
    remoteIP = udp.remoteIP();
    remotePort = udp.remotePort();
    firstPacketReceived = true;

    // 한 바이트씩 write하는 대신 버퍼를 활용해 한꺼번에 전송 시도
    while (udp.available()) {
      Serial2.write(udp.read());
    }
  }

  // --- [B] 아두이노 -> PC (병목 방지: 데이터 모아 쏘기) ---
  if (firstPacketReceived && Serial2.available()) {
    while (Serial2.available() && bufferIndex < 255) {
      char c = Serial2.read();
      serialBuffer[bufferIndex++] = c;
      lastSerialReadTime = millis();
    }
  }

  // 데이터가 버퍼에 있고, 읽기가 끝난 지 5ms가 지났거나 버퍼가 꽉 찼을 때만 전송
  if (bufferIndex > 0 && (millis() - lastSerialReadTime > 5 || bufferIndex >= 250)) {
    udp.beginPacket(remoteIP, remotePort);
    udp.write((uint8_t*)serialBuffer, bufferIndex);
    udp.endPacket();
    
    bufferIndex = 0; // 버퍼 초기화
  }
}
