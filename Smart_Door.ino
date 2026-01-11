#include <SPI.h>
#include <Adafruit_PN532.h>
#include <Servo.h>

#include <WiFiS3.h>
#include <PubSubClient.h>

// ============================================================================
// PRIVATE CONFIG (GitHub-safe)
// Create secrets.h locally (DO NOT COMMIT). Commit secrets.example.h instead.
//
// secrets.h should contain:
//   #pragma once
//   #define WIFI_SSID   "YOUR_WIFI_NAME"
//   #define WIFI_PASS   "YOUR_WIFI_PASSWORD"
//   #define MQTT_SERVER "YOUR_MQTT_BROKER_IP_OR_DOMAIN"
//   #define MQTT_PORT   1883
// ============================================================================
#include "secrets.h"

static const char* WIFI_SSID_STR   = WIFI_SSID;
static const char* WIFI_PASS_STR   = WIFI_PASS;
static const char* MQTT_SERVER_STR = MQTT_SERVER;
static const int   MQTT_PORT_NUM   = MQTT_PORT;
// ============================================================================

// ================== MQTT TOPICS ==================
// event  : JSON messages for actions/results (NFC/remote/alarm/light)
// state  : door lifecycle state (ready/locking/locked/unlocking/unlocked/online)
// alarm  : alarm mode/state (armed/disarmed/alarm)
// light  : smart light state (on/off)
// =================================================
const char* TOPIC_EVENT = "smartdoor/event";
const char* TOPIC_STATE = "smartdoor/state";
const char* TOPIC_CMD   = "smartdoor/cmd";
const char* TOPIC_ALARM = "smartdoor/alarm";
const char* TOPIC_LIGHT = "smartdoor/light";

// Remote command cooldown (prevents rapid repeated MQTT commands)
const unsigned long REMOTE_COOLDOWN_MS = 5000;
unsigned long lastRemoteCmdMs = 0;

// ================== NFC + SERVO ==================
#define PN532_SS     10
#define BUZZER_PIN    7
#define SERVO_PIN     6

Adafruit_PN532 nfc(PN532_SS);
Servo doorServo;

// Continuous servo tuning (adjust to your hardware)
const int SERVO_STOP         = 91;
const int SERVO_UNLOCK_SPEED = 120;
const int SERVO_LOCK_SPEED   = 60;

// How long to spin servo to fully unlock/lock (tune)
const int UNLOCK_MOVE_MS = 600;
const int LOCK_MOVE_MS   = 480;

// Keep for other uses (not auto-close anymore)
const unsigned long UNLOCK_HOLD_MS = 3000;

// Authorized card UID
const uint8_t AUTH_UID[] = {0x69, 0x22, 0xE9, 0x18};
const uint8_t AUTH_LEN = 4;

// Track door state for toggle behavior
bool isDoorLocked = true;  // assume starts locked

// ================== PIR + LEDs ==================
#define PIR_PIN 4

// Red LED = Alarm indicator (motion while LOCKED & ARMED)
#define LED_RED_PIN 5

// White LED = Smart light (motion while UNLOCKED & DISARMED)
#define LED_WHITE_PIN 3

// Alarm mode/state
bool alarmArmed = false;
bool alarmTriggered = false;

const unsigned long ALARM_DURATION_MS = 5000;   // buzzer + red LED ON time
unsigned long alarmUntilMs = 0;

const unsigned long PIR_REARM_GAP_MS = 2000;    // avoid rapid retriggers
unsigned long lastPirTrigMs = 0;

// Smart light state
bool lightOn = false;
const unsigned long LIGHT_DURATION_MS = 8000;   // white LED ON time
unsigned long lightUntilMs = 0;
// =======================================================

// ================== ALARM BEEP PATTERN (Non-blocking) =======================
// Alarm uses tone/noTone to create a beep-beep pattern without blocking loop().
const int ALARM_FREQ_HZ = 2500;
const unsigned long BEEP_ON_MS  = 200;
const unsigned long BEEP_OFF_MS = 150;

bool alarmBeepIsOn = false;
unsigned long nextBeepChangeMs = 0;
// ============================================================================

// ---- buzzer helpers (LOW-trigger) ----
// Manual beeps use blocking PWM-like pulses (acceptable for short feedback).
void buzzerOn()  { digitalWrite(BUZZER_PIN, LOW); }
void buzzerOff() { digitalWrite(BUZZER_PIN, HIGH); }

void beep(int freq, int durationMs) {
  pinMode(BUZZER_PIN, OUTPUT);
  for (int i = 0; i < durationMs * freq / 1000; i++) {
    digitalWrite(BUZZER_PIN, LOW);
    delayMicroseconds(500000 / freq);
    digitalWrite(BUZZER_PIN, HIGH);
    delayMicroseconds(500000 / freq);
  }
}
void accessGrantedBeep() { beep(2000, 200); }
void accessDeniedBeep()  { for (int i = 0; i < 3; i++) { beep(1000, 120); delay(120);} }

// ---- UID check ----
bool uidMatches(const uint8_t *uid, uint8_t uidLen) {
  if (uidLen != AUTH_LEN) return false;
  for (uint8_t i = 0; i < uidLen; i++) if (uid[i] != AUTH_UID[i]) return false;
  return true;
}

// ---- Servo control (blocking movement, short duration) ----
void servoStop() { doorServo.write(SERVO_STOP); }

void unlockDoor() {
  doorServo.write(SERVO_UNLOCK_SPEED);
  delay(UNLOCK_MOVE_MS);
  servoStop();
  isDoorLocked = false;
}

void lockDoor() {
  doorServo.write(SERVO_LOCK_SPEED);
  delay(LOCK_MOVE_MS);
  servoStop();
  isDoorLocked = true;
}

// ---- non-blocking delay that still services MQTT ----
// Used after NFC scan to avoid repeated reads while keeping MQTT responsive.
void mqttDelay(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    mqtt.loop();
    delay(10);
  }
}

// ================== MQTT PUBLISH HELPERS ==================
void publishEvent(const char* type, const char* result, const char* uidHex) {
  char msg[160];
  snprintf(msg, sizeof(msg),
           "{\"type\":\"%s\",\"result\":\"%s\",\"uid\":\"%s\"}",
           type, result, uidHex);
  mqtt.publish(TOPIC_EVENT, msg);
}

void publishState(const char* state) {
  mqtt.publish(TOPIC_STATE, state);
}

void publishAlarmState(const char* state) {
  mqtt.publish(TOPIC_ALARM, state);
}

void publishLightState(const char* state) {
  mqtt.publish(TOPIC_LIGHT, state);
}
// ==========================================================

// ================== ALARM HELPERS =========================
// Turn off alarm output and publish current armed state.
void alarmOff() {
  digitalWrite(LED_RED_PIN, LOW);

  noTone(BUZZER_PIN);
  alarmBeepIsOn = false;
  nextBeepChangeMs = 0;

  alarmTriggered = false;

  // Optional: publish an "alarm_cleared" event for dashboard logs
  publishEvent("alarm", "cleared", "-");

  publishAlarmState(alarmArmed ? "armed" : "disarmed");
}

// Trigger alarm due to motion while armed/locked.
void alarmOn() {
  digitalWrite(LED_RED_PIN, HIGH);

  alarmTriggered = true;
  alarmUntilMs = millis() + ALARM_DURATION_MS;

  // Start beep-beep pattern immediately
  alarmBeepIsOn = false;
  nextBeepChangeMs = 0;

  publishEvent("alarm", "motion_locked", "-");
  publishAlarmState("alarm");
}
// ==========================================================

// ================== SMART LIGHT HELPERS ===================
// Smart light is only used when unlocked/disarmed.
void smartLightOff() {
  digitalWrite(LED_WHITE_PIN, LOW);
  lightOn = false;
  publishLightState("off");
}

void smartLightOn() {
  digitalWrite(LED_WHITE_PIN, HIGH);
  lightOn = true;
  lightUntilMs = millis() + LIGHT_DURATION_MS;

  publishEvent("light", "motion_unlocked", "-");
  publishLightState("on");
}
// ==========================================================

// ================== MQTT CALLBACK (SUBSCRIBE) ==============
// Commands on smartdoor/cmd:
//   unlock/open -> unlock + disarm alarm
//   lock        -> lock + arm alarm (changed to match NFC lock behavior)
void onMqttMessage(char* topic, uint8_t* payload, unsigned int length) {
  String t = String(topic);
  String cmd;
  cmd.reserve(length + 1);
  for (unsigned int i = 0; i < length; i++) cmd += (char)payload[i];
  cmd.trim();
  cmd.toLowerCase();

  Serial.print("MQTT topic: "); Serial.print(t);
  Serial.print(" | cmd: "); Serial.println(cmd);

  if (t != TOPIC_CMD) return;

  unsigned long now = millis();
  if (now - lastRemoteCmdMs < REMOTE_COOLDOWN_MS) {
    Serial.println("Remote command ignored (cooldown).");
    return;
  }
  lastRemoteCmdMs = now;

  if (cmd == "unlock" || cmd == "open") {
    Serial.println("Remote: UNLOCK (disarm alarm)...");
    accessGrantedBeep();
    publishEvent("remote", "unlock", "-");

    publishState("unlocking");
    unlockDoor();
    publishState("unlocked");

    // Disarm alarm when unlocked remotely
    alarmArmed = false;
    alarmOff();

  } else if (cmd == "lock") {
    Serial.println("Remote: LOCK (arm alarm)...");
    publishEvent("remote", "lock", "-");

    publishState("locking");
    lockDoor();
    publishState("locked");

    // CHANGED: arm alarm on remote lock (consistent with NFC lock)
    alarmArmed = true;
    publishAlarmState("armed");

    // Optional: turn off white light when locking
    smartLightOff();

  } else {
    Serial.println("Unknown command. Use: unlock/open, lock");
  }
}
// ==========================================================

// ================== MQTT RECONNECT =========================
// Ensures MQTT connection and subscribes to command topic.
void mqttReconnect() {
  while (!mqtt.connected()) {
    Serial.print("MQTT connecting... ");

    String clientId = "UNO_R4_SmartDoor_";
    clientId += String((uint32_t)millis(), HEX);

    if (mqtt.connect(clientId.c_str())) {
      Serial.println("OK");

      publishState("online");
      mqtt.subscribe(TOPIC_CMD);

      Serial.print("Subscribed to: ");
      Serial.println(TOPIC_CMD);
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqtt.state());
      Serial.println(" retry in 2s");
      delay(2000);
    }
  }
}
// ==========================================================

void setup() {
  Serial.begin(115200);
  delay(500);

  // Buzzer (LOW-trigger relay style)
  pinMode(BUZZER_PIN, OUTPUT);
  buzzerOff();

  // PIR + LEDs
  pinMode(PIR_PIN, INPUT); // if you see noise/false triggers, consider INPUT_PULLDOWN (if supported)
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_WHITE_PIN, OUTPUT);
  digitalWrite(LED_RED_PIN, LOW);
  digitalWrite(LED_WHITE_PIN, LOW);

  // Servo init
  doorServo.attach(SERVO_PIN);
  servoStop();

  // Wi-Fi connect
  Serial.print("Connecting WiFi: ");
  Serial.println(WIFI_SSID_STR);

  WiFi.begin(WIFI_SSID_STR, WIFI_PASS_STR);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  // MQTT init
  mqtt.setServer(MQTT_SERVER_STR, MQTT_PORT_NUM);
  mqtt.setCallback(onMqttMessage);

  // Ensure MQTT is connected BEFORE publishing initial states (fix)
  mqttReconnect();

  // NFC init
  Serial.println("NFC Door Lock (PN532 SPI + MQTT + Alarm + Smart Light)");
  nfc.begin();

  uint32_t versiondata = nfc.getFirmwareVersion();
  if (!versiondata) {
    Serial.println("PN532 not found. Check SPI mode + wiring + CS pin.");
    while (1) { beep(500, 500); }
  }
  nfc.SAMConfig();
  Serial.println("Ready. Tap a card...");

  // Publish initial states (now MQTT is connected)
  publishState("ready");
  publishState(isDoorLocked ? "locked" : "unlocked");
  publishAlarmState("disarmed");
  publishLightState("off");
}

void loop() {
  if (!mqtt.connected()) mqttReconnect();
  mqtt.loop();

  unsigned long now = millis();

  // Alarm beep-beep pattern (non-blocking)
  if (alarmTriggered) {
    if (nextBeepChangeMs == 0 || (long)(now - nextBeepChangeMs) >= 0) {
      if (alarmBeepIsOn) {
        noTone(BUZZER_PIN);
        alarmBeepIsOn = false;
        nextBeepChangeMs = now + BEEP_OFF_MS;
      } else {
        tone(BUZZER_PIN, ALARM_FREQ_HZ);
        alarmBeepIsOn = true;
        nextBeepChangeMs = now + BEEP_ON_MS;
      }
    }
  }

  // Alarm timeout
  if (alarmTriggered && (long)(now - alarmUntilMs) >= 0) {
    alarmOff();
  }

  // Smart light timeout
  if (lightOn && (long)(now - lightUntilMs) >= 0) {
    smartLightOff();
  }

  // PIR logic
  int pir = digitalRead(PIR_PIN);

  // Motion while LOCKED & ARMED -> Alarm
  if (alarmArmed && isDoorLocked && !alarmTriggered) {
    if (pir == HIGH && (now - lastPirTrigMs > PIR_REARM_GAP_MS)) {
      lastPirTrigMs = now;
      Serial.println("Motion detected while LOCKED/ARMED! Alarm triggered.");
      alarmOn();
    }
  }

  // Motion while UNLOCKED & DISARMED -> Smart Light
  if (!alarmArmed && !isDoorLocked && !lightOn) {
    if (pir == HIGH && (now - lastPirTrigMs > PIR_REARM_GAP_MS)) {
      lastPirTrigMs = now;
      Serial.println("Motion detected while UNLOCKED/DISARMED! White light ON.");
      smartLightOn();
    }
  }

  // ================== NFC READ ==================
  uint8_t uid[7];
  uint8_t uidLength;
  const uint16_t NFC_TIMEOUT_MS = 50;

  if (nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength, NFC_TIMEOUT_MS)) {

    // Build UID string for logs/MQTT (e.g., "69:22:E9:18")
    char uidHex[24] = {0};
    char* p = uidHex;
    for (uint8_t i = 0; i < uidLength; i++) {
      sprintf(p, "%02X", uid[i]);
      p += 2;
      if (i < uidLength - 1) { *p = ':'; p++; }
    }

    Serial.print("UID: ");
    Serial.println(uidHex);

    if (uidMatches(uid, uidLength)) {
      Serial.println("Authorized card");
      accessGrantedBeep();
      publishEvent("nfc", "granted", uidHex);

      // NFC toggles lock state
      if (isDoorLocked) {
        Serial.println("Unlocking (NFC)...");
        publishState("unlocking");
        unlockDoor();
        publishState("unlocked");

        // Disarm alarm when unlocked
        alarmArmed = false;
        alarmOff();

      } else {
        Serial.println("Locking (NFC)...");
        publishState("locking");
        lockDoor();
        publishState("locked");

        // Arm alarm when locked via NFC
        alarmArmed = true;
        publishAlarmState("armed");

        // Turn off white light when locking (optional)
        smartLightOff();
      }

    } else {
      Serial.println("Access denied");
      accessDeniedBeep();
      publishEvent("nfc", "denied", uidHex);
    }

    // Short delay to avoid repeated reads of the same card while keeping MQTT alive
    mqttDelay(1500);
  }
}
