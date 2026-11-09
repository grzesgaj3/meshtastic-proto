#include <Arduino.h>
#include <Meshtastic.h>
#include <cstdarg>
#include <cstdio>
#include <esp_sleep.h>

#include "sensor_node.h"

// Core/SDK logs are not used; stubbing the printf family drops newlib
// formatting (~45 KB). Use Serial.print or manual formatting instead.
extern "C" {
int __wrap_printf(const char *, ...) { return 0; }
int __wrap_fprintf(FILE *, const char *, ...) { return 0; }
int __wrap_sprintf(char *buffer, const char *, ...) {
  if (buffer != nullptr) {
    buffer[0] = '\0';
  }
  return 0;
}
int __wrap_snprintf(char *buffer, size_t size, const char *, ...) {
  if (buffer != nullptr && size > 0) {
    buffer[0] = '\0';
  }
  return 0;
}
int __wrap_vprintf(const char *, va_list) { return 0; }
int __wrap_vsnprintf(char *buffer, size_t size, const char *, va_list) {
  if (buffer != nullptr && size > 0) {
    buffer[0] = '\0';
  }
  return 0;
}
}

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
