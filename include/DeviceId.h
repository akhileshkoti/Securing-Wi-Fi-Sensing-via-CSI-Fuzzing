#pragma once
#include <Arduino.h>
#include <WiFi.h>

// Short, stable, unique-per-chip ID from the factory eFuse MAC.
// Same value on every boot; differs between boards. e.g. "3F2A1C"
inline String deviceId() {
  uint64_t mac = ESP.getEfuseMac();
  char buf[7];
  snprintf(buf, sizeof(buf), "%06X", (uint32_t)(mac & 0xFFFFFF));
  return String(buf);
}