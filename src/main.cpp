#include <Arduino.h>
#include <EEPROM.h>
#include <Meshtastic.h>
#include <esp_sleep.h>

constexpr uint8_t SERIAL_RX_PIN = 16;
constexpr uint8_t SERIAL_TX_PIN = 17;
constexpr uint32_t SERIAL_BAUD = 9600;
constexpr uint8_t WAKE_PIN = 27;
constexpr uint8_t BUILTIN_LED_PIN = 2;

constexpr uint32_t SENSOR_SEND_WAIT_TIMEOUT_MS = 10000;
constexpr uint32_t SENSOR_RETRY_DELAY_MS = 5000;
constexpr uint32_t SENSOR_TRANSMISSION_SETTLE_MS = 1000;
constexpr uint8_t SENSOR_MESSAGES_PER_CYCLE = 1;
constexpr uint32_t LED_BLINK_DURATION_MS = 5000;
constexpr uint32_t LED_BLINK_INTERVAL_MS = 250;
constexpr size_t MESSAGE_BUFFER_SIZE = 256;
constexpr size_t SENSOR_MESSAGE_SIZE = 24;
constexpr size_t EEPROM_SIZE = 64;
constexpr uint32_t PENDING_MESSAGE_MAGIC = 0x53454E53;

struct PendingSensorMessage {
  uint32_t magic;
  char text[SENSOR_MESSAGE_SIZE];
};

bool led_blinking = false;
uint32_t led_blink_started_at = 0;
uint32_t led_last_toggle_at = 0;
char received_message[MESSAGE_BUFFER_SIZE] = {};
uint8_t sensor_messages_sent = 0;
uint32_t sensor_message_sent_at = 0;
uint32_t cycle_started_at = 0;
char sensor_message[SENSOR_MESSAGE_SIZE] = {};
bool pending_message_loaded = false;
bool sending_pending_message = false;
uint8_t sensor_send_attempts = 0;
uint8_t sensor_messages_to_send = SENSOR_MESSAGES_PER_CYCLE;
uint32_t next_sensor_send_at = 0;

void clear_pending_sensor_message() {
  PendingSensorMessage empty_message = {};
  EEPROM.put(0, empty_message);
  EEPROM.commit();
  pending_message_loaded = false;
}

void save_pending_sensor_message(const char *text) {
  PendingSensorMessage pending_message = {PENDING_MESSAGE_MAGIC, {}};
  strncpy(pending_message.text, text, SENSOR_MESSAGE_SIZE - 1);
  EEPROM.put(0, pending_message);
  EEPROM.commit();
  pending_message_loaded = true;
}

void load_pending_sensor_message() {
  PendingSensorMessage pending_message = {};
  EEPROM.get(0, pending_message);

  if (pending_message.magic != PENDING_MESSAGE_MAGIC ||
      pending_message.text[0] == '\0') {
    return;
  }

  strncpy(sensor_message, pending_message.text, SENSOR_MESSAGE_SIZE - 1);
  sensor_message[SENSOR_MESSAGE_SIZE - 1] = '\0';
  pending_message_loaded = true;
}

void connected_callback(mt_node_t *node, mt_nr_progress_t progress) {
  (void)node;
  (void)progress;
  Serial.println("Connected to Meshtastic device.");
}

bool validate_message(const char *text) {
  return text != nullptr && strcmp(text, "sensor1;c;") == 0;
}

void handle_received_message(const char *text) {
  if (validate_message(text)) {
    Serial.println("Valid sensor message received. Blinking LED for 5 seconds.");
    digitalWrite(BUILTIN_LED_PIN, LOW);
    led_blinking = true;
    led_blink_started_at = millis();
    led_last_toggle_at = led_blink_started_at;
  } else {
    Serial.println("Message rejected.");
  }
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

  Serial.print("Received message: ");
  Serial.println(received_message);
  handle_received_message(received_message);
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

void send_sensor_message(uint32_t now) {
  if (sensor_messages_sent == 0 && pending_message_loaded) {
    sending_pending_message = true;
  } else {
    uint8_t sensor_value = static_cast<uint8_t>(random(0, 21));
    snprintf(sensor_message, sizeof(sensor_message), "sensor1;%u", sensor_value);
    sending_pending_message = false;
  }

  Serial.print("Sending sensor message: ");
  Serial.println(sensor_message);

  if (mt_send_text(sensor_message, BROADCAST_ADDR, 0)) {
    Serial.println("Sensor message accepted by Meshtastic device.");
    if (sending_pending_message) {
      clear_pending_sensor_message();
    }
    sensor_messages_sent++;
    sensor_message_sent_at = now;
    sensor_send_attempts = 0;
    next_sensor_send_at = now;
    return;
  }

  sensor_send_attempts++;
  if (sensor_send_attempts < 2) {
    Serial.println("Sensor message was not accepted. Retrying in 5 seconds.");
    next_sensor_send_at = now + SENSOR_RETRY_DELAY_MS;
    return;
  }

  Serial.println("Sensor message failed twice. Saving it to EEPROM.");
  save_pending_sensor_message(sensor_message);
  enter_deep_sleep();
}

void update_led(uint32_t now) {
  if (!led_blinking) {
    return;
  }

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

void setup() {
  pinMode(WAKE_PIN, INPUT_PULLDOWN);
  pinMode(BUILTIN_LED_PIN, OUTPUT);
  digitalWrite(BUILTIN_LED_PIN, LOW);

  Serial.begin(9600);
  delay(100);
  randomSeed(micros());
  EEPROM.begin(EEPROM_SIZE);
  load_pending_sensor_message();
  sensor_messages_to_send = pending_message_loaded
                                ? SENSOR_MESSAGES_PER_CYCLE + 1
                                : SENSOR_MESSAGES_PER_CYCLE;

  esp_sleep_wakeup_cause_t wakeup_cause = esp_sleep_get_wakeup_cause();
  Serial.print("Wake-up cause: ");
  Serial.println(static_cast<int>(wakeup_cause));

  mt_serial_init(SERIAL_RX_PIN, SERIAL_TX_PIN, SERIAL_BAUD);
  mt_request_node_report(connected_callback);
  set_text_message_callback(text_message_callback);

  Serial.println("Module initialized.");

  cycle_started_at = millis();

  if (wakeup_cause != ESP_SLEEP_WAKEUP_EXT0) {
    enter_deep_sleep();
  }
}

void loop() {
  uint32_t now = millis();

  if (mt_loop(now) && sensor_messages_sent < sensor_messages_to_send &&
      now >= next_sensor_send_at) {
    send_sensor_message(now);
  }

  update_led(now);

  if (!led_blinking && sensor_messages_sent == sensor_messages_to_send &&
      now - sensor_message_sent_at >= SENSOR_TRANSMISSION_SETTLE_MS) {
    Serial.println("Sensor transmission complete.");
    enter_deep_sleep();
  }

  if (sensor_messages_sent < sensor_messages_to_send &&
      now - cycle_started_at >= SENSOR_SEND_WAIT_TIMEOUT_MS) {
    Serial.println("Meshtastic send timeout.");
    enter_deep_sleep();
  }
}

