#include "GuardianBeacon.h"
#include "GuardianSettings.h"
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <time.h>
#include <cstring>

namespace {
  const uint8_t BROADCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

  // Deterministic 32-bit mix (splitmix32-style). This exact function is
  // reimplemented in controller/fuzzing_eval.py's slot_hash() -- it IS the
  // shared schedule: the defender (who knows `seed`) and nobody else can
  // reproduce it. The two implementations must stay byte-for-byte
  // equivalent; if you change one, change the other.
  uint32_t slotHash(uint32_t seed, uint32_t slot) {
    uint32_t h = seed ^ slot;
    h ^= h >> 16; h *= 0x7feb352dU;
    h ^= h >> 15; h *= 0x846ca68bU;
    h ^= h >> 16;
    return h;
  }

  bool isBurstSlot(uint32_t seed, uint32_t slot) {
    // slotHash() is ~uniform over [0, 2^32); compare against a fraction of
    // that range (integer math) rather than modulo + floating point, so
    // both language implementations agree exactly.
    uint64_t threshold = (uint64_t)(GuardianSettings::BURST_PROBABILITY * 4294967296.0);
    return (uint64_t)slotHash(seed, slot) < threshold;
  }
}

void GuardianBeacon::begin(uint32_t seed) {
  _seed = seed;

  // One-time SNTP sync so this board's notion of "current slot" lines up
  // with the controller's (which uses wall-clock Unix time from the CSV
  // log). Clock drift over a short capture session is negligible; this is
  // a prototype, not a production time-sync scheme.
  Serial.print("[Guardian] syncing time via SNTP");
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  time_t now = 0;
  uint32_t start = millis();
  while (now < 1700000000 && millis() - start < 15000) {  // sanity floor: after ~2023
    delay(200);
    time(&now);
    Serial.print(".");
  }
  Serial.println();
  if (now < 1700000000) {
    Serial.println("[Guardian] SNTP sync failed -- schedule will not line up with the controller");
    return;
  }
  _epochMsAtBoot = (int64_t)now * 1000 - (int64_t)millis();
  Serial.printf("[Guardian] synced, epoch_ms_at_boot=%lld seed=%lu\n",
                (long long)_epochMsAtBoot, (unsigned long)seed);

  if (esp_now_init() != ESP_OK) {
    Serial.println("[Guardian] ESP-NOW init failed");
    _epochMsAtBoot = 0;
    return;
  }
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, BROADCAST, 6);
  peer.channel = 0;          // 0 = use current (STA) channel
  peer.ifidx   = WIFI_IF_STA;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("[Guardian] add broadcast peer failed");
  }

  // Same rate-forcing trick as the Illuminators: ESP-NOW defaults to 1Mbps
  // 802.11b DSSS, which carries no OFDM long training field and produces no
  // CSI at any listener -- legitimate receiver or eavesdropper alike.
  esp_err_t rerr = esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_6M);
  if (rerr != ESP_OK) {
    Serial.printf("[Guardian] set rate failed: %d\n", rerr);
  }
}

void GuardianBeacon::loop() {
  if (_epochMsAtBoot == 0) return;   // not synced / init failed

  int64_t epochMs = _epochMsAtBoot + (int64_t)millis();
  uint32_t slot = (uint32_t)(epochMs / GuardianSettings::SLOT_MS);
  if (slot == _lastSlot) return;     // already handled this slot
  _lastSlot = slot;

  if (!isBurstSlot(_seed, slot)) return;

  for (uint8_t i = 0; i < GuardianSettings::BURST_FRAMES; i++) {
    char payload[32];
    int n = snprintf(payload, sizeof(payload), "GRD,%lu,%lu",
                      (unsigned long)slot, (unsigned long)_seq++);
    esp_now_send(BROADCAST, (const uint8_t *)payload, n);
    delay(GuardianSettings::BURST_FRAME_GAP_MS);
  }
  Serial.printf("[Guardian] burst slot=%lu\n", (unsigned long)slot);
}
