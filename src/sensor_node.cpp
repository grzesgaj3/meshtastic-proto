#include "sensor_node.h"

#include <Meshtastic.h>
#include <cstring>
#include <esp_attr.h>
#include <esp_random.h>

namespace {
constexpr uint8_t SERIAL_RX_PIN = 16;
constexpr uint8_t SERIAL_TX_PIN = 17;
constexpr uint32_t SERIAL_BAUD = 9600;
constexpr uint8_t LED_PIN = 2;

constexpr uint32_t SEND_WAIT_TIMEOUT_MS = 10000;
constexpr uint32_t RETRY_DELAY_MS = 5000;
constexpr uint32_t TRANSMISSION_SETTLE_MS = 1000;
constexpr uint32_t LED_BLINK_DURATION_MS = 5000;
constexpr uint32_t LED_BLINK_INTERVAL_MS = 250;
constexpr size_t MESSAGE_SIZE = 24;
constexpr char VALID_MESSAGE[] = "sensor1;c;";

enum class Phase : uint8_t { Sending, Done, Failed };

RTC_DATA_ATTR char pendingMessage[MESSAGE_SIZE] = {};
RTC_DATA_ATTR bool pendingMessageValid = false;

Phase phase = Phase::Sending;
char messageBuffer[MESSAGE_SIZE] = {};
uint32_t cycleStartedAt = 0;
uint32_t nextSendAt = 0;
uint32_t lastSentAt = 0;
uint32_t ledStartedAt = 0;
uint32_t ledLastToggleAt = 0;
uint8_t sentCount = 0;
uint8_t targetCount = 1;
uint8_t sendAttempts = 0;
bool ledActive = false;

void formatSensorMessage(uint8_t value) {
  memcpy(messageBuffer, "sensor1;", 8);
  char *p = messageBuffer + 8;
  if (value >= 10) {
    *p++ = static_cast<char>('0' + value / 10);
  }
  *p++ = static_cast<char>('0' + value % 10);
  *p = '\0';
}

void startLedBlink(uint32_t now) {
  digitalWrite(LED_PIN, LOW);
  ledActive = true;
  ledStartedAt = now;
  ledLastToggleAt = now;
}

void updateLed(uint32_t now) {
  if (!ledActive) {
    return;
  }
  if (now - ledStartedAt >= LED_BLINK_DURATION_MS) {
    digitalWrite(LED_PIN, LOW);
    ledActive = false;
    return;
  }
  if (now - ledLastToggleAt >= LED_BLINK_INTERVAL_MS) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    ledLastToggleAt = now;
  }
}

void textMessageCallback(uint32_t from, uint32_t to, uint8_t channel, const char *text) {
  (void)from;
  (void)to;
  (void)channel;
  if (text != nullptr && strcmp(text, VALID_MESSAGE) == 0) {
    startLedBlink(millis());
  }
}

void savePendingMessage() {
  strncpy(pendingMessage, messageBuffer, MESSAGE_SIZE - 1);
  pendingMessage[MESSAGE_SIZE - 1] = '\0';
  pendingMessageValid = true;
}
} // namespace

void sensorNodeBegin() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  randomSeed(esp_random());

  targetCount = pendingMessageValid ? 2 : 1;
  phase = Phase::Sending;
  sentCount = 0;
  sendAttempts = 0;
  cycleStartedAt = millis();
  nextSendAt = cycleStartedAt;

  mt_serial_init(SERIAL_RX_PIN, SERIAL_TX_PIN, SERIAL_BAUD);
  set_text_message_callback(textMessageCallback);
}

bool sensorNodeSend() {
  const bool sendingPending = sentCount == 0 && pendingMessageValid;
  if (sendingPending) {
    memcpy(messageBuffer, pendingMessage, MESSAGE_SIZE);
  } else {
    formatSensorMessage(static_cast<uint8_t>(esp_random() % 21));
  }

  if (!mt_send_text(messageBuffer, BROADCAST_ADDR, 0)) {
    sendAttempts++;
    return false;
  }

  if (sendingPending) {
    pendingMessageValid = false;
  }
  sentCount++;
  sendAttempts = 0;
  nextSendAt = millis();
  return true;
}

SensorNodeStatus sensorNodeUpdate(uint32_t now) {
  const bool ready = mt_loop(now);
  now = millis();
  updateLed(now);

  if (phase != Phase::Sending) {
    return phase == Phase::Done ? SensorNodeStatus::Done : SensorNodeStatus::Failed;
  }

  if (sentCount >= targetCount) {
    if (!ledActive && now - lastSentAt >= TRANSMISSION_SETTLE_MS) {
      phase = Phase::Done;
      return SensorNodeStatus::Done;
    }
    return SensorNodeStatus::Busy;
  }

  if (now - cycleStartedAt >= SEND_WAIT_TIMEOUT_MS) {
    phase = Phase::Failed;
    return SensorNodeStatus::Failed;
  }

  if (!ready || now < nextSendAt) {
    return SensorNodeStatus::Busy;
  }

  if (sensorNodeSend()) {
    lastSentAt = now;
    return SensorNodeStatus::Busy;
  }

  if (sendAttempts >= 2) {
    savePendingMessage();
    phase = Phase::Failed;
    return SensorNodeStatus::Failed;
  }

  nextSendAt = now + RETRY_DELAY_MS;
  return SensorNodeStatus::Busy;
}
