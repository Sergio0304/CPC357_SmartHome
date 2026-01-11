#define EIDSP_QUANTIZE_FILTERBANK 0

#include <Arduino.h>
#include <ESP32Servo.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s.h"

#include <DHT.h>
#include <WiFi.h>
#include <PubSubClient.h>

#include <Audio_Classification_-_Keyword_Spotting_inferencing.h>

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

// ================== INMP441 I2S PINS (ESP32-S3) =============================
// INMP441 wiring (example):
//   BCLK/SCK -> GPIO14
//   WS/LRCLK -> GPIO21
//   SD/DOUT  -> GPIO47
//   L/R pin  -> GND  (forces LEFT channel)
// ============================================================================
#define I2S_BCLK  14
#define I2S_WS    21
#define I2S_DIN   47
// ============================================================================

// ================== USER BUTTON (Manual Toggle) =============================
// Button on GPIO3 (D3). Uses INPUT_PULLUP, so:
//   - released = HIGH
//   - pressed  = LOW
// Debounce prevents multiple triggers from switch bounce.
// ============================================================================
#define BUTTON_PIN  3
static const uint32_t BTN_DEBOUNCE_MS = 40;
static bool last_btn_level = HIGH;
static uint32_t last_btn_change_ms = 0;
// ============================================================================

// ================== RAIN SENSOR (Digital Output) ============================
// Uses a DO pin from rain module.
// If module outputs LOW when wet -> set RAIN_ACTIVE_LOW = true.
// RAIN_CONFIRM_MS: how long wet must persist before "raining" is latched
// RAIN_CLEAR_MS  : how long dry must persist before rain latch is cleared
// ============================================================================
#define RAIN_DO_PIN  4
static const bool RAIN_ACTIVE_LOW = true;
static const uint32_t RAIN_CONFIRM_MS = 500;
static const uint32_t RAIN_CLEAR_MS   = 1500;

static bool rain_latched = false;
static uint32_t rain_since_ms = 0;
static uint32_t dry_since_ms  = 0;
// ============================================================================

// ================== DHT22 (Temperature Automation) ==========================
// Reads temperature every DHT_INTERVAL_MS.
// TEMP_OPEN_AT: auto-open threshold (if temp automation enabled).
// temp_open_latched avoids repeated open commands while temp stays high.
// ============================================================================
#define DHT_PIN   6
#define DHT_TYPE  DHT22
DHT dht(DHT_PIN, DHT_TYPE);

static const float TEMP_OPEN_AT = 40.0f;
static const uint32_t DHT_INTERVAL_MS = 2000;
static uint32_t last_dht_ms = 0;
static bool temp_open_latched = false;
// ============================================================================

// ================== SERVO (Window Open/Close) ===============================
// Continuous rotation servo control using pulse width:
//   NEUTRAL_US: stop
//   OPEN_US   : rotate toward open direction
//   CLOSE_US  : rotate toward close direction
// OPEN_MOVE_MS / CLOSE_MOVE_MS: movement duration tuning for your mechanism.
// Servo is controlled non-blocking using a stop timer.
// ============================================================================
#define SERVO_PIN  38
Servo windowServo;

static const int NEUTRAL_US = 1500;
static const int OPEN_US    = 1900;
static const int CLOSE_US   = 1100;

static const uint32_t OPEN_MOVE_MS  = 200;
static const uint32_t CLOSE_MOVE_MS = 140;

static bool servo_moving = false;
static uint32_t servo_stop_at_ms = 0;

static inline void servoStop() {
  windowServo.writeMicroseconds(NEUTRAL_US);
  servo_moving = false;
}
// ============================================================================

// ================== VOICE DETECTION (Edge Impulse) ==========================
// Hysteresis thresholds:
// - Trigger when confidence >= TRIGGER_ON
// - Re-arm only after confidence <= TRIGGER_OFF for HOLD_OFF_MS
// This prevents repeated triggers while confidence stays high.
// COOLDOWN_MS adds a minimum time gap between actions (safety + stability).
// ============================================================================
static const float TRIGGER_ON  = 0.70f;
static const float TRIGGER_OFF = 0.50f;
static const uint32_t COOLDOWN_MS = 1500;
static const uint32_t HOLD_OFF_MS = 800;

static uint32_t last_toggle_ms = 0;
static uint32_t last_high_ms = 0;
static bool waiting_for_drop = false;

static bool window_is_open = false;
// ============================================================================

// ================== WIFI + MQTT (Cloud Connectivity) =========================
// Publishes:
//   aiwindow/state  -> retained state snapshot (open/rain/temp + automation flags)
//   aiwindow/event  -> non-retained events (actions, sensor events, boot)
// Subscribes:
//   aiwindow/cmd    -> commands: open/close/toggle/voice_on/off/temp_on/off/rain_on/off
// ============================================================================
const char* TOPIC_STATE = "aiwindow/state";
const char* TOPIC_EVENT = "aiwindow/event";
const char* TOPIC_CMD   = "aiwindow/cmd";

WiFiClient espClient;
PubSubClient mqtt(espClient);

// Automation switches (toggled by MQTT commands)
static bool voice_auto = true;   // enable/disable voice actions
static bool temp_auto  = true;   // enable/disable temperature auto-open
static bool rain_auto  = true;   // enable/disable rain auto-close (and voice lockout while raining)

static uint32_t last_state_publish_ms = 0;
static const uint32_t STATE_PUBLISH_INTERVAL_MS = 1000;

// Print-once latches to avoid repeating the same "ignored" logs every loop
static bool voice_ignored_latched = false;
static bool rain_voice_ignored_latched = false;
// ============================================================================

// ================== AUDIO INFERENCE BUFFERS (Double Buffer) =================
// Audio samples are captured via I2S into a ring-style double buffer.
// When one buffer is full, it is marked ready for EI inference.
// ============================================================================
typedef struct {
  int16_t *buffers[2];
  uint8_t  buf_select;
  volatile uint8_t buf_ready;
  uint32_t buf_count;
  uint32_t n_samples;
} inference_t;

static inference_t inference;
static bool debug_nn = false;
static volatile bool record_status = true;

// I2S DMA read buffers
static const uint32_t sample_buffer_size_bytes = 2048;
static int32_t i2s_read_buf32[sample_buffer_size_bytes / 4];
static int16_t sampleBuffer16[sample_buffer_size_bytes / 2];

// Copy converted 16-bit samples into the active inference buffer.
// When full, swap buffer and mark ready.
static void audio_inference_callback(uint32_t n_samples_16) {
  for (uint32_t i = 0; i < n_samples_16; i++) {
    inference.buffers[inference.buf_select][inference.buf_count++] = sampleBuffer16[i];

    if (inference.buf_count >= inference.n_samples) {
      inference.buf_select ^= 1;
      inference.buf_count = 0;
      inference.buf_ready = 1;
    }
  }
}

// Initialize I2S for INMP441 input (RX only, 32-bit samples, left channel).
static int i2s_init(uint32_t sampling_rate) {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = sampling_rate,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 256,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_BCLK,
    .ws_io_num = I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_DIN
  };

  esp_err_t ret = i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  if (ret != ESP_OK) return ret;

  ret = i2s_set_pin(I2S_NUM_0, &pin_config);
  if (ret != ESP_OK) return ret;

  ret = i2s_zero_dma_buffer(I2S_NUM_0);
  return ret;
}

// FreeRTOS task: continuously read I2S samples, convert 32->16-bit, apply gain, feed EI buffer.
static void capture_samples(void *arg) {
  (void)arg;

  while (record_status) {
    size_t bytes_read = 0;

    esp_err_t err = i2s_read(I2S_NUM_0, (void*)i2s_read_buf32, sample_buffer_size_bytes,
                             &bytes_read, portMAX_DELAY);

    if (err != ESP_OK || bytes_read == 0) continue;

    // INMP441 outputs 24-bit (packed in 32). SHIFT and GAIN tune signal level.
    const int SHIFT = 15;
    const int GAIN  = 8;

    uint32_t samples_32 = bytes_read / 4;
    if (samples_32 > (sample_buffer_size_bytes / 4)) samples_32 = (sample_buffer_size_bytes / 4);

    for (uint32_t i = 0; i < samples_32; i++) {
      int16_t v = (int16_t)(i2s_read_buf32[i] >> SHIFT);

      int32_t boosted = (int32_t)v * GAIN;
      if (boosted > 32767) boosted = 32767;
      if (boosted < -32768) boosted = -32768;

      sampleBuffer16[i] = (int16_t)boosted;
    }

    audio_inference_callback(samples_32);
  }

  vTaskDelete(NULL);
}

// Allocate inference buffers, start I2S, and launch capture task.
static bool microphone_inference_start(uint32_t n_samples) {
  inference.buffers[0] = (int16_t*)malloc(n_samples * sizeof(int16_t));
  if (!inference.buffers[0]) return false;

  inference.buffers[1] = (int16_t*)malloc(n_samples * sizeof(int16_t));
  if (!inference.buffers[1]) {
    free(inference.buffers[0]);
    return false;
  }

  inference.buf_select = 0;
  inference.buf_count = 0;
  inference.n_samples = n_samples;
  inference.buf_ready = 0;

  if (i2s_init(EI_CLASSIFIER_FREQUENCY) != ESP_OK) return false;

  record_status = true;
  xTaskCreate(capture_samples, "CaptureSamples", 1024 * 10, NULL, 10, NULL);
  return true;
}

// Wait until a full slice is ready for inference.
static bool microphone_inference_record(void) {
  while (inference.buf_ready == 0) delay(1);
  inference.buf_ready = 0;
  return true;
}

// Edge Impulse callback: provide float samples for the requested slice segment.
static int microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr) {
  numpy::int16_to_float(&inference.buffers[inference.buf_select ^ 1][offset], out_ptr, length);
  return 0;
}

// ================== MQTT HELPERS ============================================
// Publish retained device state (periodically and after actions).
// 'reason' helps dashboard/debugging (why state changed).
static void mqttPublishState(const char* reason) {
  float t = dht.readTemperature();
  if (isnan(t)) t = -999;

  char msg[280];
  snprintf(msg, sizeof(msg),
           "{\"open\":%d,\"rain\":%d,\"temp\":%.1f,"
           "\"voice\":%d,\"tempAuto\":%d,\"rainAuto\":%d,"
           "\"reason\":\"%s\"}",
           window_is_open ? 1 : 0,
           rain_latched ? 1 : 0,
           t,
           voice_auto ? 1 : 0,
           temp_auto ? 1 : 0,
           rain_auto ? 1 : 0,
           reason);
  mqtt.publish(TOPIC_STATE, msg, true);
}

// Publish non-retained events (actions/sensor transitions).
static void mqttPublishEvent(const char* event, const char* detail = "") {
  char msg[240];
  snprintf(msg, sizeof(msg),
           "{\"event\":\"%s\",\"detail\":\"%s\",\"open\":%d,\"rain\":%d}",
           event, detail,
           window_is_open ? 1 : 0,
           rain_latched ? 1 : 0);
  mqtt.publish(TOPIC_EVENT, msg, false);
}
// ============================================================================

// ================== SERVO ACTIONS (Non-Blocking) ============================
// Starts an open/close movement then stops automatically after OPEN_MOVE_MS / CLOSE_MOVE_MS.
// 'reason' is logged and published to MQTT for debugging.
static void startOpenMove(const char* reason) {
  if (servo_moving) {
    Serial.printf(">>> OPEN IGNORED (servo moving) | reason=%s\n", reason);
    return;
  }

  Serial.printf(">>> OPEN WINDOW | reason=%s\n", reason);
  windowServo.writeMicroseconds(OPEN_US);
  servo_moving = true;
  servo_stop_at_ms = millis() + OPEN_MOVE_MS;
  window_is_open = true;

  mqttPublishEvent("action", (String("open: ") + reason).c_str());
  mqttPublishState(reason);
}

static void startCloseMove(const char* reason) {
  if (servo_moving) {
    Serial.printf(">>> CLOSE IGNORED (servo moving) | reason=%s\n", reason);
    return;
  }

  Serial.printf(">>> CLOSE WINDOW | reason=%s\n", reason);
  windowServo.writeMicroseconds(CLOSE_US);
  servo_moving = true;
  servo_stop_at_ms = millis() + CLOSE_MOVE_MS;
  window_is_open = false;

  mqttPublishEvent("action", (String("close: ") + reason).c_str());
  mqttPublishState(reason);
}

// Stop the servo when its movement time has elapsed (non-blocking timing).
static void updateServo() {
  if (servo_moving && (int32_t)(millis() - servo_stop_at_ms) >= 0) {
    servoStop();
    Serial.println(">>> SERVO STOP");
    mqttPublishEvent("action", "servo_stop");
    mqttPublishState("servo_stop");
  }
}

// Manual/remote toggle (used by button + MQTT "toggle").
static void toggleWindowNow(uint32_t now, const char* reason) {
  if (now - last_toggle_ms < COOLDOWN_MS) {
    Serial.printf(">>> TOGGLE IGNORED (cooldown) | reason=%s\n", reason);
    return;
  }

  if (!window_is_open) startOpenMove(reason);
  else                 startCloseMove(reason);

  last_toggle_ms = now;
}
// ============================================================================

// ================== MQTT CALLBACK / CONNECT ================================
// Parse incoming commands from TOPIC_CMD and update device control states.
static void mqttCallback(char* topic, byte* payload, unsigned int length) {
  (void)topic; // topic not used (single subscription)
  String cmd;
  cmd.reserve(length + 1);
  for (unsigned int i = 0; i < length; i++) cmd += (char)payload[i];
  cmd.trim();

  Serial.print(">>> MQTT CMD: ");
  Serial.println(cmd);

  if (cmd == "open") {
    startOpenMove("mqtt:open");
  } else if (cmd == "close") {
    startCloseMove("mqtt:close");
  } else if (cmd == "toggle") {
    toggleWindowNow(millis(), "mqtt:toggle");

  } else if (cmd == "voice_on") {
    voice_auto = true;
    voice_ignored_latched = false;
    Serial.println(">>> VOICE AUTO ON (mqtt)");
    mqttPublishEvent("cmd", "voice_on");
    mqttPublishState("voice_on");

  } else if (cmd == "voice_off") {
    voice_auto = false;
    voice_ignored_latched = false;
    Serial.println(">>> VOICE AUTO OFF (mqtt)");
    mqttPublishEvent("cmd", "voice_off");
    mqttPublishState("voice_off");

  } else if (cmd == "temp_on") {
    temp_auto = true;
    Serial.println(">>> TEMP AUTO ON (mqtt)");
    mqttPublishEvent("cmd", "temp_on");
    mqttPublishState("temp_on");

  } else if (cmd == "temp_off") {
    temp_auto = false;
    Serial.println(">>> TEMP AUTO OFF (mqtt)");
    mqttPublishEvent("cmd", "temp_off");
    mqttPublishState("temp_off");

  } else if (cmd == "rain_on") {
    rain_auto = true;
    rain_voice_ignored_latched = false;
    Serial.println(">>> RAIN AUTO ON (mqtt)");
    mqttPublishEvent("cmd", "rain_on");
    mqttPublishState("rain_on");

  } else if (cmd == "rain_off") {
    rain_auto = false;
    rain_voice_ignored_latched = false;
    Serial.println(">>> RAIN AUTO OFF (mqtt)");
    mqttPublishEvent("cmd", "rain_off");
    mqttPublishState("rain_off");

  } else if (cmd == "auto_on") {
    voice_auto = true;
    temp_auto  = true;
    voice_ignored_latched = false;
    Serial.println(">>> AUTO MODE ON (legacy mqtt) -> voice+temp ON");
    mqttPublishEvent("cmd", "auto_on");
    mqttPublishState("auto_on");

  } else if (cmd == "auto_off") {
    voice_auto = false;
    temp_auto  = false;
    voice_ignored_latched = false;
    Serial.println(">>> AUTO MODE OFF (legacy mqtt) -> voice+temp OFF");
    mqttPublishEvent("cmd", "auto_off");
    mqttPublishState("auto_off");
  }
}

// Connect to Wi-Fi (blocking until connected).
// NOTE: For production, you could add a timeout + retry strategy.
static void wifiConnect() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID_STR, WIFI_PASS_STR);

  Serial.print("WiFi connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(400);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}

// Ensure MQTT stays connected; subscribe to commands after connect.
static void mqttEnsureConnected() {
  if (mqtt.connected()) return;

  while (!mqtt.connected()) {
    Serial.print("MQTT connecting... ");
    String clientId = "aiwindow-" + String((uint32_t)ESP.getEfuseMac(), HEX);

    if (mqtt.connect(clientId.c_str())) {
      Serial.println("OK");
      mqtt.subscribe(TOPIC_CMD);
      mqttPublishEvent("boot", "mqtt_connected");
      mqttPublishState("boot");
    } else {
      Serial.print("FAIL rc=");
      Serial.print(mqtt.state());
      Serial.println(" retry in 2s");
      delay(2000);
    }
  }
}
// ============================================================================

// ================== BUTTON HANDLER =========================================
// Debounced manual toggle. Button is active-low (INPUT_PULLUP).
static void updateButton(uint32_t now) {
  bool level = digitalRead(BUTTON_PIN);

  if (level != last_btn_level) {
    last_btn_level = level;
    last_btn_change_ms = now;
    return;
  }

  if (now - last_btn_change_ms < BTN_DEBOUNCE_MS) return;

  static bool pressed_latched = false;
  if (level == LOW && !pressed_latched) {
    pressed_latched = true;
    Serial.println(">>> D3 BUTTON PRESS");
    toggleWindowNow(now, "button:D3");
  }
  if (level == HIGH) pressed_latched = false;
}
// ============================================================================

// ================== RAIN HANDLER ===========================================
// Converts the digital pin into a boolean "raining" signal.
static bool isRainingNow() {
  bool lvl = digitalRead(RAIN_DO_PIN);
  return RAIN_ACTIVE_LOW ? (lvl == LOW) : (lvl == HIGH);
}

// Latches rain state with confirm/clear timers to reject sensor noise.
// If rain_auto is enabled and window is open, it will auto-close for safety.
static void updateRain(uint32_t now) {
  bool raining = isRainingNow();

  if (raining) {
    dry_since_ms = 0;

    if (rain_since_ms == 0) rain_since_ms = now;

    if (!rain_latched && (now - rain_since_ms >= RAIN_CONFIRM_MS)) {
      rain_latched = true;
      Serial.println(">>> RAIN DETECTED");
      mqttPublishEvent("rain", "detected");
      mqttPublishState("rain_detected");

      if (rain_auto && window_is_open) {
        Serial.println(">>> AUTO CLOSE DUE TO RAIN (rain_auto=ON)");
        startCloseMove("rain:auto_close");
        last_toggle_ms = now;
      } else if (!rain_auto) {
        Serial.println(">>> RAIN AUTO-CLOSE DISABLED (rain_auto=OFF)");
      }
    }
  } else {
    rain_since_ms = 0;

    if (dry_since_ms == 0) dry_since_ms = now;

    if (rain_latched && (now - dry_since_ms >= RAIN_CLEAR_MS)) {
      rain_latched = false;
      Serial.println(">>> RAIN CLEARED (re-armed)");
      mqttPublishEvent("rain", "cleared");
      mqttPublishState("rain_cleared");

      // Allow voice/rain "ignored" log to print again next time it rains
      rain_voice_ignored_latched = false;

      // Re-arm temperature trigger after rain clears (optional design choice)
      temp_open_latched = false;
    }
  }
}
// ============================================================================

// ================== DHT22 HANDLER ==========================================
// Periodically read temperature and auto-open if threshold is exceeded.
// Note: temp automation is skipped while rain is latched and rain_auto is enabled.
static void updateDHT(uint32_t now) {
  if (now - last_dht_ms < DHT_INTERVAL_MS) return;
  last_dht_ms = now;

  float t = dht.readTemperature();
  if (isnan(t)) {
    Serial.println(">>> DHT22 read failed");
    mqttPublishEvent("dht", "read_failed");
    return;
  }

  Serial.print(">>> Temp: ");
  Serial.print(t, 1);
  Serial.println(" C");

  if (rain_latched && rain_auto) return;
  if (!temp_auto) return;

  if (!temp_open_latched && t >= TEMP_OPEN_AT) {
    temp_open_latched = true;

    if (!window_is_open) {
      Serial.println(">>> TEMP >= 40C: FORCE OPEN");
      startOpenMove("temp>=40C");
      last_toggle_ms = now;
    }
  }

  // Small hysteresis for temperature to avoid rapid re-triggering near 40C
  if (temp_open_latched && t < (TEMP_OPEN_AT - 1.0f)) {
    temp_open_latched = false;
  }
}
// ============================================================================

// ================== SETUP ==================================================
// Initializes sensors, network, servo, and starts the audio inference pipeline.
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n=== AI Window (Voice + Button + Rain + Temp) + MQTT ===");

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(RAIN_DO_PIN, INPUT_PULLUP);

  dht.begin();

  windowServo.setPeriodHertz(50);
  windowServo.attach(SERVO_PIN, 500, 2500);
  servoStop();

  wifiConnect();
  mqtt.setServer(MQTT_HOST_STR, MQTT_PORT_NUM);
  mqtt.setCallback(mqttCallback);
  mqttEnsureConnected();

  // Edge Impulse model init
  run_classifier_init();

  // Start microphone capture + double-buffer inference
  if (!microphone_inference_start(EI_CLASSIFIER_SLICE_SIZE)) {
    Serial.println("ERR: Failed to start microphone inference");
    while (1) delay(1000);
  }

  last_toggle_ms = millis();
  last_high_ms = 0;

  Serial.println("Ready. Speak / press D3 / rain closes (if enabled) / temp>=40 opens (if enabled)...");
  mqttPublishEvent("boot", "ready");
  mqttPublishState("ready");
}
// ============================================================================

// ================== LOOP ===================================================
// Main loop tasks:
// 1) Keep MQTT connection alive + publish periodic state
// 2) Update servo timing + read sensors (button/rain/temp)
// 3) Run Edge Impulse continuous inference and trigger open/close on keywords
void loop() {
  uint32_t now = millis();

  mqttEnsureConnected();
  mqtt.loop();

  if (now - last_state_publish_ms >= STATE_PUBLISH_INTERVAL_MS) {
    last_state_publish_ms = now;
    mqttPublishState("periodic");
  }

  updateServo();
  updateButton(now);
  updateRain(now);
  updateDHT(now);

  // Wait for next audio slice for inference
  if (!microphone_inference_record()) return;

  signal_t signal;
  signal.total_length = EI_CLASSIFIER_SLICE_SIZE;
  signal.get_data = &microphone_audio_signal_get_data;

  ei_impulse_result_t result = {0};
  EI_IMPULSE_ERROR r = run_classifier_continuous(&signal, &result, debug_nn);
  if (r != EI_IMPULSE_OK) return;

  // Safety: ignore voice while raining if rain_auto is enabled
  if (rain_latched && rain_auto) {
    if (!rain_voice_ignored_latched) {
      Serial.println(">>> VOICE IGNORED (raining & rain_auto=ON)");
      rain_voice_ignored_latched = true;
    }
    return;
  } else {
    rain_voice_ignored_latched = false;
  }

  // If voice automation is disabled, do not act on EI keywords
  if (!voice_auto) {
    if (!voice_ignored_latched) {
      Serial.println(">>> VOICE IGNORED (voice_auto=OFF)");
      voice_ignored_latched = true;
    }
    return;
  } else {
    voice_ignored_latched = false;
  }

  // Extract confidence scores from EI output
  float open_conf = 0.0f, close_conf = 0.0f;
  for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
    const char* lbl = result.classification[i].label;
    if (strcmp(lbl, "open window") == 0 || strcmp(lbl, "open_window") == 0) {
      open_conf = result.classification[i].value;
    } else if (strcmp(lbl, "close window") == 0 || strcmp(lbl, "close_window") == 0) {
      close_conf = result.classification[i].value;
    }
  }

  // Decide which command is stronger (open vs close)
  bool want_open = (open_conf > close_conf);
  float best_conf = want_open ? open_conf : close_conf;

  // Trigger with hysteresis + cooldown to avoid repeated actions
  if (best_conf >= TRIGGER_ON) {
    last_high_ms = now;

    if (!waiting_for_drop && (now - last_toggle_ms >= COOLDOWN_MS)) {
      if (want_open) {
        if (!window_is_open) startOpenMove("voice:open");
        else Serial.println(">>> VOICE OPEN IGNORED (already open)");
      } else {
        if (window_is_open) startCloseMove("voice:close");
        else Serial.println(">>> VOICE CLOSE IGNORED (already closed)");
      }

      last_toggle_ms = now;
      waiting_for_drop = true;
    }
  }

  // Re-arm after confidence drops below TRIGGER_OFF for HOLD_OFF_MS
  if (waiting_for_drop) {
    if (best_conf <= TRIGGER_OFF && (now - last_high_ms >= HOLD_OFF_MS)) {
      waiting_for_drop = false;
    }
  }
}
