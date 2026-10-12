#pragma once
#include <Arduino.h>
#include "esp_mac.h"

// This board's Wi-Fi station MAC -- the source MAC on its ESP-NOW/CSI
// frames, i.e. the MAC the controller dashboard identifies it by. Read from
// eFuse rather than via WiFi.macAddress(), so it is correct in any Wi-Fi
// mode (including the SoftAP-only captive portal).
inline void stationMac(uint8_t mac[6]) {
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
}

// Station MAC as "AA:BB:CC:DD:EE:FF".
inline String stationMacString() {
  uint8_t mac[6];
  stationMac(mac);
  char buf[18];
  snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(buf);
}

// Short, stable, unique-per-chip ID: the last 3 bytes of the station MAC,
// e.g. "3FA2D0" for B4:3A:45:3F:A2:D0. The first 3 bytes are the vendor
// OUI, shared by every board from the same manufacturer, so they must not
// be used here.
inline String deviceId() {
  uint8_t mac[6];
  stationMac(mac);
  char buf[7];
  snprintf(buf, sizeof(buf), "%02X%02X%02X", mac[3], mac[4], mac[5]);
  return String(buf);
}
