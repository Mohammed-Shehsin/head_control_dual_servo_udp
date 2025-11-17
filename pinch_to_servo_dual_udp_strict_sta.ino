#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESP32Servo.h>

// ======= Wi-Fi STA (router / hotspot) =======
const char* WIFI_SSID = "WIFI_ADDRR";     // <-- set
const char* WIFI_PASS = "WIFI_PASS"; // <-- set
const uint16_t UDP_PORT = 4210;

// ======= Servos (MG90S) =======
const int SERVO1_PIN = 18; // Right hand -> Servo1
const int SERVO2_PIN = 19; // Left  hand -> Servo2
const int PULSE_MIN_US = 500;
const int PULSE_MAX_US = 2400;
const int SERVO_HZ     = 50;

// ======= Failsafe =======
const uint32_t COMMAND_TIMEOUT_MS = 1200;

WiFiUDP udp;
Servo servo1, servo2;
uint32_t lastCmdMs = 0;
bool centeredOnce = true;

int clampAngle(int a){ return a<0?0:(a>180?180:a); }

void applyAngles(int a1, int a2){
  a1 = clampAngle(a1); a2 = clampAngle(a2);
  servo1.write(a1);
  servo2.write(a2);
  Serial.printf("APPLIED %d,%d\r\n", a1, a2);
}

void setup() {
  Serial.begin(115200);
  delay(200);

  // Servos
  servo1.setPeriodHertz(SERVO_HZ);
  servo2.setPeriodHertz(SERVO_HZ);
  servo1.attach(SERVO1_PIN, PULSE_MIN_US, PULSE_MAX_US);
  servo2.attach(SERVO2_PIN, PULSE_MIN_US, PULSE_MAX_US);
  applyAngles(90, 90);

  // Wi-Fi
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("\r\nConnecting to %s", WIFI_SSID);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
    if (millis() - t0 > 20000) {
      Serial.println("\r\nWi-Fi connect timeout. Rebooting...");
      ESP.restart();
    }
  }
  Serial.printf("\r\nConnected. IP: %s\r\n", WiFi.localIP().toString().c_str());

  // UDP
  udp.begin(UDP_PORT);
  Serial.printf("Listening UDP port %u\r\n", UDP_PORT);
  Serial.println("Expect lines like: 45,120");

  lastCmdMs = millis();
  centeredOnce = true;
}

void loop() {
  int pktSize = udp.parsePacket();
  if (pktSize > 0) {
    char buf[64];
    int n = udp.read(buf, sizeof(buf)-1);
    if (n > 0) {
      buf[n] = '\0';
      Serial.printf("RECEIVED: %s\r\n", buf); // show raw

      String s(buf); s.trim();
      int idx = s.indexOf(',');
      if (idx > 0) {
        String s1 = s.substring(0, idx); s1.trim();
        String s2 = s.substring(idx+1);  s2.trim();
        if (s1.length() && s2.length()) {
          int a1 = s1.toInt();
          int a2 = s2.toInt();
          applyAngles(a1, a2);
          lastCmdMs = millis();
          centeredOnce = false;
        }
      }
    }
  }

  if ((millis() - lastCmdMs) > COMMAND_TIMEOUT_MS && !centeredOnce) {
    Serial.println("TIMEOUT → CENTER 90,90");
    applyAngles(90, 90);
    centeredOnce = true;
  }
}
