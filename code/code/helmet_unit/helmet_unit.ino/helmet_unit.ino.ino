/*
 Project: Smart Helmet with Sensor-Based Ignition Control
 Module: Helmet Unit
 Platform: ESP32
 Description:
  - Detects helmet wearing using limit switch
  - Detects alcohol using MQ-3 sensor
  - Sends data wirelessly to bike unit using ESP-NOW
*/

#include <WiFi.h>
#include <esp_now.h>

// ================= PIN DEFINITIONS =================
#define MQ3_PIN        34     // Analog pin for MQ-3 sensor
#define LIMIT_PIN      13     // Digital pin for limit switch

// ================= CONSTANTS =================
const unsigned long SEND_INTERVAL = 1200; // milliseconds

// ================= DATA PACKET =================
typedef struct {
  uint8_t  helmet;     // 0 = not worn, 1 = worn
  uint16_t alcohol;    // ADC value
  uint32_t seq;        // timestamp
  uint8_t  checksum;
} HelmetPkt;

// ================= GLOBAL VARIABLES =================
uint8_t bikeMAC[6] = {0xD4, 0x8A, 0xFC, 0x3B, 0x92, 0x7C};

uint32_t lastSend    = 0;
int      lastAlcohol = -1;
uint8_t  lastHelmet  = 0xFF;

// ================= FUNCTIONS =================
uint8_t simpleChecksum(const HelmetPkt &p) {
  uint32_t s = p.helmet +
               (p.alcohol & 0xFF) + (p.alcohol >> 8) +
               (p.seq & 0xFF) + (p.seq >> 8) +
               (p.seq >> 16) + (p.seq >> 24);
  return (uint8_t)s;
}

int readMQ3Avg(uint8_t samples = 6) {
  long sum = 0;
  for (uint8_t i = 0; i < samples; i++) {
    sum += analogRead(MQ3_PIN);
    delay(20);
  }
  return (int)(sum / samples);
}

// ================= SETUP =================
void setup() {
  pinMode(LIMIT_PIN, INPUT_PULLUP);

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.begin(115200);
    Serial.println("ESP-NOW init failed");
    return;
  }

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, bikeMAC, 6);
  peer.channel = 0;
  peer.encrypt = false;

  esp_now_add_peer(&peer);
}

// ================= LOOP =================
void loop() {
  uint32_t now = millis();

  int     alcoholValue = readMQ3Avg();
  uint8_t helmetState  = (digitalRead(LIMIT_PIN) == LOW) ? 1 : 0;

  bool changed = (alcoholValue != lastAlcohol) ||
                 (helmetState  != lastHelmet);

  if (changed || (now - lastSend) > SEND_INTERVAL) {
    HelmetPkt pkt;
    pkt.helmet   = helmetState;
    pkt.alcohol  = (uint16_t)alcoholValue;
    pkt.seq      = now;
    pkt.checksum = simpleChecksum(pkt);

    esp_now_send(bikeMAC, (uint8_t *)&pkt, sizeof(pkt));

    lastSend    = now;
    lastAlcohol = alcoholValue;
    lastHelmet  = helmetState;
  }

  delay(100);
}
