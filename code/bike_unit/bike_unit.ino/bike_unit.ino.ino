/*
 Project: Smart Helmet with Sensor-Based Ignition Control
 Module: Bike Unit
 Platform: ESP32
*/

#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <TinyGPS++.h>
#include <MPU6050.h>

#define OWNER_NUMBER "+91800509xxxx"

// Pins
#define RELAY_PIN   18
#define BUZZER_PIN  25
#define START_BTN   27

#define GPS_RX_PIN  5
#define GPS_TX_PIN  4
#define GSM_RX_PIN  17
#define GSM_TX_PIN  16

// Thresholds
#define ALCOHOL_THRESHOLD 200
#define ACC_THRESHOLD_G  1.10
#define ANGLE_THRESHOLD  45.0

LiquidCrystal_I2C lcd(0x27, 16, 2);
TinyGPSPlus gps;
HardwareSerial GPSserial(2);
HardwareSerial GSMserial(1);
MPU6050 mpu;

// Helmet packet
typedef struct {
  uint8_t helmet;
  uint16_t alcohol;
  uint32_t seq;
  uint8_t checksum;
} HelmetPkt;

// Globals
volatile uint16_t lastAlcohol = 0;
volatile uint8_t lastHelmet = 0;
volatile uint32_t lastHelmetTime = 0;

bool ignitionAllowed = false;
bool bikeRunning = false;
unsigned long lastAccCheck = 0;

// ---------- Utility ----------
uint8_t checksum(const HelmetPkt &p) {
  uint32_t s = p.helmet + (p.alcohol & 0xFF) + (p.alcohol >> 8)
               + (p.seq & 0xFF) + (p.seq >> 8) + (p.seq >> 16) + (p.seq >> 24);
  return (uint8_t)s;
}

String mapLink(double lat, double lon) {
  char buf[64];
  snprintf(buf, sizeof(buf), "https://maps.google.com/?q=%.6f,%.6f", lat, lon);
  return String(buf);
}

void sendSMS(const String &msg) {
  GSMserial.println("AT");
  delay(100);
  GSMserial.println("AT+CMGF=1");
  delay(100);
  GSMserial.print("AT+CMGS=\"");
  GSMserial.print(OWNER_NUMBER);
  GSMserial.println("\"");
  delay(100);
  GSMserial.print(msg);
  GSMserial.write(26);
  delay(800);
}

bool getGPS(String &loc) {
  unsigned long t0 = millis();
  while (millis() - t0 < 1500) {
    while (GPSserial.available()) {
      gps.encode(GPSserial.read());
    }
    if (gps.location.isValid()) {
      loc = mapLink(gps.location.lat(), gps.location.lng());
      return true;
    }
  }
  loc = "GPS not fixed";
  return false;
}

// ---------- Accident Detection ----------
bool accidentDetected() {
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);

  float accX = ax / 16384.0;
  float accY = ay / 16384.0;
  float accZ = az / 16384.0;

  float acc_g = sqrt(accX * accX + accY * accY + accZ * accZ);
  float pitch = atan2(accX, sqrt(accY * accY + accZ * accZ)) * 180.0 / PI;
  float roll  = atan2(accY, sqrt(accX * accX + accZ * accZ)) * 180.0 / PI;

  return (acc_g > ACC_THRESHOLD_G &&
          (abs(pitch) > ANGLE_THRESHOLD || abs(roll) > ANGLE_THRESHOLD));
}

// ---------- ESP-NOW ----------
void onRecv(const esp_now_recv_info *info, const uint8_t *data, int len) {
  if (len != sizeof(HelmetPkt)) return;

  HelmetPkt p;
  memcpy(&p, data, sizeof(p));

  if (checksum(p) != p.checksum) return;

  lastHelmet = p.helmet;
  lastAlcohol = p.alcohol;
  lastHelmetTime = millis();
}

void initEspNow() {
  WiFi.mode(WIFI_STA);
  esp_now_init();
  esp_now_register_recv_cb(onRecv);
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);

  pinMode(RELAY_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(START_BTN, INPUT_PULLUP);

  digitalWrite(RELAY_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  Wire.begin(21, 22);
  lcd.init();
  lcd.backlight();
  lcd.print("Smart Bike");
  delay(1500);
  lcd.clear();

  mpu.initialize();

  GPSserial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  GSMserial.begin(9600, SERIAL_8N1, GSM_RX_PIN, GSM_TX_PIN);

  initEspNow();
}

// ---------- Loop ----------
void loop() {
  unsigned long now = millis();

  // Accident check
  if (now - lastAccCheck > 200) {
    lastAccCheck = now;
    if (accidentDetected()) {
      lcd.clear();
      lcd.print("Accident!");
      digitalWrite(BUZZER_PIN, HIGH);
      digitalWrite(RELAY_PIN, LOW);
      bikeRunning = false;

      String loc;
      getGPS(loc);
      sendSMS("Accident Alert! " + loc);

      delay(1500);
      digitalWrite(BUZZER_PIN, LOW);
    }
  }

  // Helmet & alcohol logic
  if (millis() - lastHelmetTime < 5000) {
    lcd.clear();
    if (lastHelmet) {
      lcd.print("Helmet OK");
      lcd.setCursor(0, 1);
      if (lastAlcohol > ALCOHOL_THRESHOLD) {
        lcd.print("Alcohol Detected");
        ignitionAllowed = false;
        digitalWrite(RELAY_PIN, LOW);
      } else {
        lcd.print("Ready to Start");
        ignitionAllowed = true;
      }
    } else {
      lcd.print("Wear Helmet");
      ignitionAllowed = false;
    }
  }

  // Start button
  if (digitalRead(START_BTN) == LOW) {
    delay(80);
    if (digitalRead(START_BTN) == LOW) {
      String loc;
      getGPS(loc);
      if (!ignitionAllowed) {
        sendSMS("Start Blocked " + loc);
      } else {
        digitalWrite(RELAY_PIN, HIGH);
        bikeRunning = true;
        sendSMS("Bike Started " + loc);
      }
      while (digitalRead(START_BTN) == LOW);
    }
  }

  // Helmet removed while running
  if (bikeRunning && lastHelmet == 0) {
    digitalWrite(RELAY_PIN, LOW);
    bikeRunning = false;
    String loc;
    getGPS(loc);
    sendSMS("Helmet Removed " + loc);
    lcd.clear();
    lcd.print("Helmet Removed");
  }

  delay(50);
}
