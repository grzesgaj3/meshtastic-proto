#include <Arduino.h>
#include <Meshtastic.h>
#include <esp_sleep.h>

#define SERIAL_RX_PIN 16
#define SERIAL_TX_PIN 17
#define SERIAL_BAUD 9600
#define WAKE_PIN 27
#define BUILTIN_LED_PIN 2

constexpr uint32_t SENSOR_SEND_WAIT_TIMEOUT_MS = 10000;
constexpr uint32_t SENSOR_TRANSMISSION_SETTLE_MS = 1000;
constexpr uint8_t SENSOR_MESSAGES_TO_SEND = 2;
constexpr uint32_t LED_BLINK_DURATION_MS = 5000;
constexpr uint32_t LED_BLINK_INTERVAL_MS = 250;
constexpr size_t MESSAGE_BUFFER_SIZE = 256;

bool message_received = false;
bool message_is_valid = false;
bool led_blinking = false;
uint32_t led_blink_started_at = 0;
uint32_t led_last_toggle_at = 0;
char received_message[MESSAGE_BUFFER_SIZE] = {};

void connected_callback(mt_node_t *node, mt_nr_progress_t progress) {
  (void)node;
  (void)progress;
  Serial.println("Connected to Meshtastic device.");
}

bool validate_message(const char *text) {
  return text != nullptr && strcmp(text, "sensor1;c;") == 0;
}

void text_message_callback(uint32_t from, uint32_t to, uint8_t channel, const char *text) {
  (void)from;
  (void)to;
  (void)channel;

  if (text == nullptr) {
    return;
  }

  strncpy(received_message, text, MESSAGE_BUFFER_SIZE - 1);
  received_message[MESSAGE_BUFFER_SIZE - 1] = '\0';
  message_is_valid = validate_message(received_message);
  message_received = true;

  Serial.print("Received message: ");
  Serial.println(received_message);
}

void enter_deep_sleep() {
  while (digitalRead(WAKE_PIN) == HIGH) {
    mt_loop(millis());
    delay(10);
  }

  Serial.println("Entering deep sleep.");
  Serial.flush();
  esp_sleep_enable_ext0_wakeup(static_cast<gpio_num_t>(WAKE_PIN), 1);
  esp_deep_sleep_start();
}

void setup() {
  pinMode(WAKE_PIN, INPUT_PULLDOWN);
  pinMode(BUILTIN_LED_PIN, OUTPUT);
  digitalWrite(BUILTIN_LED_PIN, LOW);

  Serial.begin(9600);
  delay(100);
  randomSeed(micros());

  esp_sleep_wakeup_cause_t wakeup_cause = esp_sleep_get_wakeup_cause();
  Serial.print("Wake-up cause: ");
  Serial.println(static_cast<int>(wakeup_cause));

  mt_serial_init(SERIAL_RX_PIN, SERIAL_TX_PIN, SERIAL_BAUD);
  mt_request_node_report(connected_callback);
  set_text_message_callback(text_message_callback);

  Serial.println("Module initialized.");

  if (wakeup_cause != ESP_SLEEP_WAKEUP_EXT0) {
    enter_deep_sleep();
  }
}

void loop() {
  static const uint32_t cycle_started_at = millis();
  static uint8_t sensor_messages_sent = 0;
  static uint32_t sensor_message_sent_at = 0;
  uint32_t now = millis();

  bool can_send = mt_loop(now);

  if (can_send && sensor_messages_sent < SENSOR_MESSAGES_TO_SEND) {
    char sensor_message[24];
    uint8_t sensor_value = static_cast<uint8_t>(random(0, 21));
    snprintf(sensor_message, sizeof(sensor_message), "sensor1;%u", sensor_value);
    mt_send_text(sensor_message, BROADCAST_ADDR, 0);
    Serial.print("Sent sensor message: ");
    Serial.println(sensor_message);
    sensor_messages_sent++;
    sensor_message_sent_at = now;
  }

  if (message_received) {
    if (message_is_valid) {
      Serial.println("Valid sensor message received. Blinking LED for 5 seconds.");
      digitalWrite(BUILTIN_LED_PIN, LOW);
      led_blinking = true;
      led_blink_started_at = now;
      led_last_toggle_at = now;
      message_received = false;
    } else {
      Serial.println("Message rejected.");
      message_received = false;
    }
  }

  if (led_blinking) {
    if (now - led_blink_started_at >= LED_BLINK_DURATION_MS) {
      digitalWrite(BUILTIN_LED_PIN, LOW);
      led_blinking = false;
      enter_deep_sleep();
    }

    if (now - led_last_toggle_at >= LED_BLINK_INTERVAL_MS) {
      digitalWrite(BUILTIN_LED_PIN, !digitalRead(BUILTIN_LED_PIN));
      led_last_toggle_at = now;
    }
  }

  if (!led_blinking && sensor_messages_sent == SENSOR_MESSAGES_TO_SEND &&
      now - sensor_message_sent_at >= SENSOR_TRANSMISSION_SETTLE_MS) {
    Serial.println("Sensor transmission complete.");
    enter_deep_sleep();
  }

  if (sensor_messages_sent < SENSOR_MESSAGES_TO_SEND &&
      now - cycle_started_at >= SENSOR_SEND_WAIT_TIMEOUT_MS) {
    Serial.println("Meshtastic send timeout.");
    enter_deep_sleep();
  }
}

