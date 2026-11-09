#include <Arduino.h>
#include <Meshtastic.h>
#include <esp_sleep.h>

#include "sensor_node.h"

namespace {
constexpr uint8_t WAKE_PIN = 27;
constexpr uint32_t WAKE_POLL_DELAY_MS = 10;
}

void enterDeepSleep() {
  while (digitalRead(WAKE_PIN) == HIGH) {
    mt_loop(millis());
    delay(WAKE_POLL_DELAY_MS);
  }

  esp_sleep_enable_ext0_wakeup(static_cast<gpio_num_t>(WAKE_PIN), 1);
  esp_deep_sleep_start();
}

void setup() {
  pinMode(WAKE_PIN, INPUT_PULLDOWN);
  Serial.begin(9600);
  sensorNodeBegin();

  if (esp_sleep_get_wakeup_cause() != ESP_SLEEP_WAKEUP_EXT0) {
    enterDeepSleep();
  }
}

void loop() {
  if (sensorNodeUpdate(millis()) != SensorNodeStatus::Busy) {
    enterDeepSleep();
  }
}
