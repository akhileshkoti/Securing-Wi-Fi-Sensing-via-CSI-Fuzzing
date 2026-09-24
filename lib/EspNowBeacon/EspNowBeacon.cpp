#include "EspNowBeacon.h"
#include "Settings.h"
#include "DeviceId.h"
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

namespace {
  const uint8_t BROADCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  volatile uint32_t s_fail = 0;

  void onSent(const uint8_t *mac, esp_now_send_status_t status) {
    if (status != ESP_NOW_SEND_SUCCESS) s_fail++;
  }
}

bool EspNowBeacon::begin() {
  // WiFi is already started (STA connected), which keeps us on the AP channel.
  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESPNOW] init failed");
    return false;
  }
  esp_now_register_send_cb(onSent);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, BROADCAST, 6);
  peer.channel = 0;                 // 0 = use current (STA) channel
  peer.ifidx   = WIFI_IF_STA;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("[ESPNOW] add broadcast peer failed");
    return false;
  }

  // ESP-NOW defaults to 1 Mbps 802.11b (DSSS) which has NO OFDM LTF, so the
  // receiver can't compute CSI from it. Use the lowest *legacy OFDM* rate:
  // it's a basic rate (broadcast is reliably received) and carries L-LTF.
  esp_err_t rerr = esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_6M);
  if (rerr != ESP_OK) {
    Serial.printf("[ESPNOW] set rate failed: %d\n", rerr);
  }

  _ready = true;
  Serial.printf("[ESPNOW] beacon ready, MAC=%s\n", WiFi.macAddress().c_str());
  return true;
}

void EspNowBeacon::loop() {
  if (!_ready) return;
  uint32_t now = millis();
  if (now - _last < Settings::BEACON_INTERVAL_MS) return;
  _last = now;

  char payload[48];
  int n = snprintf(payload, sizeof(payload), "ILU,%s,%lu",
                   deviceId().c_str(), (unsigned long)_seq++);
  esp_now_send(BROADCAST, (const uint8_t *)payload, n);

  static uint32_t lastLog = 0, cnt = 0;
  cnt++;
  if (now - lastLog >= 1000) {
    Serial.printf("[ESPNOW] sent %lu/s (failed %lu)\n",
                  (unsigned long)cnt, (unsigned long)s_fail);
    cnt = 0; s_fail = 0; lastLog = now;
  }
}