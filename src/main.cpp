#include <Arduino.h>
#include <EEPROM.h>
#include <ESP32Servo.h>
#include <Meshtastic.h>
#include <esp_sleep.h>

constexpr uint8_t SERIAL_RX_PIN = 32;
constexpr uint8_t SERIAL_TX_PIN = 33;
constexpr uint32_t SERIAL_BAUD = 9600;
constexpr uint8_t SENSOR_PIN = 14;
constexpr uint8_t MESSAGE_WAKE_PIN = 27;
constexpr uint8_t BUILTIN_LED_PIN = 2;

constexpr uint32_t SENSOR_MEASUREMENT_WINDOW_MS = 10000;
constexpr uint32_t SENSOR_SEND_WAIT_TIMEOUT_MS = 10000;
constexpr uint32_t SENSOR_RETRY_DELAY_MS = 5000;
constexpr uint32_t SENSOR_TRANSMISSION_SETTLE_MS = 1000;
constexpr uint32_t SENSOR_PULSE_DEBOUNCE_US = 1000;
constexpr uint8_t SENSOR_MESSAGES_PER_CYCLE = 1;
constexpr uint32_t LED_BLINK_DURATION_MS = 5000;
constexpr uint32_t LED_BLINK_INTERVAL_MS = 250;
constexpr size_t MESSAGE_BUFFER_SIZE = 256;
constexpr size_t SENSOR_MESSAGE_SIZE = 24;
constexpr size_t EEPROM_SIZE = 64;
constexpr uint32_t PENDING_MESSAGE_MAGIC = 0x53454E53;

constexpr int VERTICAL_HOME = 0;
constexpr int VERTICAL_TARGET = 180;
constexpr uint32_t VERTICAL_MOVE_MS = 2000;
constexpr int BRUSH_HOME = 0;
constexpr int BRUSH_TARGET = 90;
constexpr uint32_t BRUSH_MOVE_MS = 1000;
constexpr int BRUSH_SPEED_US = 1700;
constexpr int SERVO_STOP_US = 1500;
constexpr int HORIZONTAL_FORWARD_US = 1800;
constexpr int HORIZONTAL_BACKWARD_US = 1200;
constexpr uint32_t CLEANING_TIME_MS = 24000;
constexpr int HORIZONTAL_REPEATS = 2;
constexpr uint32_t CLEANING_MARGIN_MS = 2000;
constexpr uint32_t CLEANING_STEP_MS =
  CLEANING_TIME_MS / (2 * HORIZONTAL_REPEATS);

struct PendingSensorMessage {
  uint32_t magic;
  char text[SENSOR_MESSAGE_SIZE];
};

bool led_blinking = false;
bool cleaning_requested = false;
bool measurement_complete = false;
bool sensor_measurement_enabled = false;
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
uint32_t measurement_started_at = 0;
volatile uint32_t sensor_pulse_count = 0;
volatile uint32_t last_sensor_pulse_at_us = 0;
uint32_t measured_pulse_count = 0;

Servo horizontal_servo;
Servo vertical_servo;
Servo brush_position_servo;
Servo brush_rotation_servo;

void IRAM_ATTR sensor_pulse_isr() {
  uint32_t now_us = micros();
  if (now_us - last_sensor_pulse_at_us >= SENSOR_PULSE_DEBOUNCE_US) {
    sensor_pulse_count++;
    last_sensor_pulse_at_us = now_us;
  }
}

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
  return text != nullptr &&
         (strcmp(text, "sensor1;c") == 0 || strcmp(text, "sensor1;c;") == 0);
}

void handle_received_message(const char *text) {
  if (validate_message(text)) {
    Serial.println("Valid sensor message received. Blinking LED for 5 seconds.");
    cleaning_requested = true;
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

  Serial.print("ODEBRANO WIADOMOSC RAW: ");

  if (text == nullptr) {
    Serial.println("<NULL>");
    return;
  }

  strncpy(received_message, text, MESSAGE_BUFFER_SIZE - 1);
  received_message[MESSAGE_BUFFER_SIZE - 1] = '\0';

  Serial.println(received_message);
  Serial.print("DLUGOSC WIADOMOSCI: ");
  Serial.println(strlen(received_message));
  handle_received_message(received_message);
}

void portnum_callback(uint32_t from, uint32_t to, uint8_t channel,
                     meshtastic_PortNum port,
                     meshtastic_Data_payload_t *payload) {
  (void)to;
  (void)channel;

  Serial.print("ODEBRANO INNY PAKIET; FROM=0x");
  Serial.print(from, HEX);
  Serial.print("; PORT=");
  Serial.print(static_cast<int>(port));

  if (payload != nullptr) {
    Serial.print("; SIZE=");
    Serial.print(payload->size);
    Serial.print("; HEX=");
    for (size_t i = 0; i < payload->size && i < 64; i++) {
      if (payload->bytes[i] < 0x10) {
        Serial.print('0');
      }
      Serial.print(payload->bytes[i], HEX);
      Serial.print(' ');
    }
  }

  Serial.println();
}

void encrypted_callback(uint32_t from, uint32_t to, uint8_t channel,
                       meshtastic_MeshPacket_public_key_t pubKey,
                       meshtastic_MeshPacket_encrypted_t *payload) {
  (void)to;
  (void)channel;
  (void)pubKey;
  (void)payload;

  Serial.print("ODEBRANO ZASZYFROWANY PAKIET; FROM=0x");
  Serial.println(from, HEX);
}

void enter_deep_sleep() {
  while (digitalRead(MESSAGE_WAKE_PIN) == HIGH) {
    mt_loop(millis());
    delay(10);
  }

  Serial.println("Entering deep sleep.");
  Serial.flush();
  esp_sleep_enable_ext0_wakeup(static_cast<gpio_num_t>(MESSAGE_WAKE_PIN), 1);
  uint64_t sensor_wake_mask = 1ULL << SENSOR_PIN;
  esp_sleep_enable_ext1_wakeup(sensor_wake_mask, ESP_EXT1_WAKEUP_ALL_LOW);
  esp_deep_sleep_start();
}

void stop_rotation() {
  horizontal_servo.writeMicroseconds(SERVO_STOP_US);
  brush_rotation_servo.writeMicroseconds(SERVO_STOP_US);
}

void clean_rain_gauge() {
  Serial.println("Cleaning cycle started.");

  vertical_servo.write(VERTICAL_TARGET);
  delay(VERTICAL_MOVE_MS);

  brush_position_servo.write(BRUSH_TARGET);
  delay(BRUSH_MOVE_MS);

  brush_rotation_servo.writeMicroseconds(BRUSH_SPEED_US);
  for (int repeat = 0; repeat < HORIZONTAL_REPEATS; repeat++) {
    horizontal_servo.writeMicroseconds(HORIZONTAL_FORWARD_US);
    delay(CLEANING_STEP_MS);
    horizontal_servo.writeMicroseconds(HORIZONTAL_BACKWARD_US);
    delay(CLEANING_STEP_MS);
  }

  stop_rotation();
  delay(CLEANING_MARGIN_MS);

  brush_position_servo.write(BRUSH_HOME);
  delay(BRUSH_MOVE_MS);
  vertical_servo.write(VERTICAL_HOME);
  delay(VERTICAL_MOVE_MS);

  stop_rotation();
  Serial.println("Cleaning cycle complete.");
}

void finish_sensor_measurement() {
  noInterrupts();
  measured_pulse_count = sensor_pulse_count;
  interrupts();
  detachInterrupt(digitalPinToInterrupt(SENSOR_PIN));
  measurement_complete = true;

  Serial.print("Sensor measurement complete. Pulses: ");
  Serial.println(measured_pulse_count);
}

void send_sensor_message(uint32_t now) {
  if (sensor_messages_sent == 0 && pending_message_loaded) {
    sending_pending_message = true;
  } else {
    snprintf(sensor_message, sizeof(sensor_message), "sensor1;%lu",
             static_cast<unsigned long>(measured_pulse_count));
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
    return;
  }

  if (now - led_last_toggle_at >= LED_BLINK_INTERVAL_MS) {
    digitalWrite(BUILTIN_LED_PIN, !digitalRead(BUILTIN_LED_PIN));
    led_last_toggle_at = now;
  }
}

void setup() {
  pinMode(SENSOR_PIN, INPUT_PULLUP);
  pinMode(MESSAGE_WAKE_PIN, INPUT_PULLDOWN);
  pinMode(BUILTIN_LED_PIN, OUTPUT);
  digitalWrite(BUILTIN_LED_PIN, LOW);

  Serial.begin(9600);
  delay(100);
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
  set_portnum_callback(portnum_callback);
  set_encrypted_callback(encrypted_callback);

  Serial.println("Module initialized.");

  if (wakeup_cause != ESP_SLEEP_WAKEUP_EXT0 &&
      wakeup_cause != ESP_SLEEP_WAKEUP_EXT1) {
    enter_deep_sleep();
  }

  sensor_measurement_enabled =
      wakeup_cause == ESP_SLEEP_WAKEUP_EXT1;

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  horizontal_servo.setPeriodHertz(50);
  vertical_servo.setPeriodHertz(50);
  brush_position_servo.setPeriodHertz(50);
  brush_rotation_servo.setPeriodHertz(50);
  horizontal_servo.attach(21, 500, 2400);
  vertical_servo.attach(19, 500, 2400);
  brush_position_servo.attach(18, 500, 2400);
  brush_rotation_servo.attach(22, 500, 2400);
  stop_rotation();
  vertical_servo.write(VERTICAL_HOME);
  brush_position_servo.write(BRUSH_HOME);

  measurement_complete = !sensor_measurement_enabled;

  if (sensor_measurement_enabled) {
    sensor_pulse_count = 0;
    last_sensor_pulse_at_us = micros();
    measurement_started_at = millis();
    cycle_started_at = measurement_started_at;
    attachInterrupt(digitalPinToInterrupt(SENSOR_PIN), sensor_pulse_isr, FALLING);
    Serial.println("Sensor measurement started.");
  } else {
    measurement_complete = true;
    measured_pulse_count = 0;
    cycle_started_at = millis();
    Serial.println("Message check started; sensor measurement skipped.");
  }
}

void loop() {
  uint32_t now = millis();

  if (sensor_measurement_enabled && !measurement_complete &&
      now - measurement_started_at >= SENSOR_MEASUREMENT_WINDOW_MS) {
    finish_sensor_measurement();
  }

  if (mt_loop(now) && measurement_complete &&
      sensor_messages_sent < sensor_messages_to_send &&
      now >= next_sensor_send_at) {
    if (sensor_measurement_enabled) {
      send_sensor_message(now);
    } else if (pending_message_loaded) {
      send_sensor_message(now);
    } else {
      sensor_messages_sent = sensor_messages_to_send;
    }
  }

  update_led(now);

  if (!led_blinking && cleaning_requested) {
    cleaning_requested = false;
    clean_rain_gauge();
    enter_deep_sleep();
  }

  if (sensor_measurement_enabled &&
      !led_blinking && sensor_messages_sent == sensor_messages_to_send &&
      now - sensor_message_sent_at >= SENSOR_TRANSMISSION_SETTLE_MS) {
    Serial.println("Sensor transmission complete.");
    if (cleaning_requested) {
      cleaning_requested = false;
      clean_rain_gauge();
    }
    enter_deep_sleep();
  }

  if (!sensor_measurement_enabled &&
      now - cycle_started_at >= SENSOR_SEND_WAIT_TIMEOUT_MS) {
    Serial.println("Message check complete.");
    enter_deep_sleep();
  }

  if (sensor_measurement_enabled &&
      sensor_messages_sent < sensor_messages_to_send &&
      now - cycle_started_at >=
          SENSOR_MEASUREMENT_WINDOW_MS + SENSOR_SEND_WAIT_TIMEOUT_MS) {
    Serial.println("Meshtastic send timeout.");
    enter_deep_sleep();
  }
}

