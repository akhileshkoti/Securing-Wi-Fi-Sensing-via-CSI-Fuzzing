#include "WifiStation.h"
#include <WiFi.h>

bool WifiStation::connect(const NetConfig &cfg, uint32_t timeoutMs) {
  Serial.printf("[WiFi] Connecting to '%s'...\n", cfg.ssid.c_str());
  WiFi.mode(WIFI_STA);
  WiFi.begin(cfg.ssid.c_str(), cfg.pass.c_str());

  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < timeoutMs) {
    delay(250);
    Serial.print('.');
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("[WiFi] Connected. IP=%s\n", WiFi.localIP().toString().c_str());
    return true;
  }
  Serial.println("[WiFi] Connection failed.");
  return false;
}