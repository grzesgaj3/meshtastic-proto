#ifndef SENSOR_NODE_H
#define SENSOR_NODE_H

#include <Arduino.h>

enum class SensorNodeStatus : uint8_t { Busy, Done, Failed };

void sensorNodeBegin();
bool sensorNodeSend();
SensorNodeStatus sensorNodeUpdate(uint32_t now);

#endif
