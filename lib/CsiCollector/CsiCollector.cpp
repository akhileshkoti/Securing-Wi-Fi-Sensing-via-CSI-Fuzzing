#include "CsiCollector.h"
#include "Settings.h"
#include <WiFi.h>
#include "esp_wifi.h"
#include <esp_now.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

namespace {
  volatile uint32_t s_espnowRx = 0;   // ESP-NOW frames received

  // Accept the Illuminators' ESP-NOW frames so the stack processes them.
  // CSI is captured separately by the PHY; this just keeps ESP-NOW alive.
  void onEspNowRecv(const uint8_t *mac, const uint8_t *data, int len) {
    (void)mac; (void)data; (void)len;
    s_espnowRx++;
  }

  struct CsiRecord {
    uint8_t  mac[6];        // source MAC = which illuminator
    int8_t   rssi;
    int8_t   noise_floor;
    uint8_t  sig_mode;      // 0=legacy(11g), 1=HT(11n), 3=VHT
    uint8_t  rate;          // PHY rate index (constant => consistent CSI)
    uint8_t  mcs;
    uint8_t  cwb;           // 0=20MHz, 1=40MHz
    uint8_t  channel;
    uint8_t  secondary;     // secondary channel
    uint16_t len;
    int8_t   data[Settings::MAX_CSI_LEN];
  };

  QueueHandle_t s_queue = nullptr;
  volatile uint32_t s_csiCount = 0;   // CSI callback invocations

  // Wi-Fi task context: keep minimal, just copy + enqueue.
  void onCsi(void *ctx, wifi_csi_info_t *info) {
    if (!s_queue || !info || !info->buf || info->len == 0) return;
    s_csiCount++;
    const wifi_pkt_rx_ctrl_t &rx = info->rx_ctrl;
    CsiRecord r;
    memcpy(r.mac, info->mac, 6);
    r.rssi        = rx.rssi;
    r.noise_floor = rx.noise_floor;
    r.sig_mode    = rx.sig_mode;
    r.rate        = rx.rate;
    r.mcs         = rx.mcs;
    r.cwb         = rx.cwb;
    r.channel     = rx.channel;
    r.secondary   = rx.secondary_channel;
    r.len         = info->len > Settings::MAX_CSI_LEN ? Settings::MAX_CSI_LEN : info->len;
    memcpy(r.data, info->buf, r.len);
    xQueueSend(s_queue, &r, 0);   // drop if full rather than block
  }
}

void CsiCollector::begin(IPAddress controllerIp, uint16_t port) {
  _controller = controllerIp;
  _port       = port;

  s_queue = xQueueCreate(24, sizeof(CsiRecord));

  wifi_csi_config_t cfg = {};
  cfg.lltf_en           = true;
  cfg.htltf_en          = true;
  cfg.stbc_htltf2_en    = true;
  cfg.ltf_merge_en      = true;
  cfg.channel_filter_en = true;
  cfg.manu_scale        = false;
  cfg.shift             = 0;

  esp_wifi_set_csi_config(&cfg);
  esp_wifi_set_csi_rx_cb(&onCsi, nullptr);
  esp_wifi_set_csi(true);

  // Receive side of ESP-NOW so the Illuminators' frames are accepted.
  if (esp_now_init() == ESP_OK) {
    esp_now_register_recv_cb(onEspNowRecv);
  }

  Serial.printf("[CSI] enabled, forwarding (binary) to %s:%u\n",
                _controller.toString().c_str(), _port);
}

void CsiCollector::loop() {
  if (!s_queue) return;

  CsiRecord r;
  uint8_t pkt[18 + Settings::MAX_CSI_LEN];
  int sent = 0;

  while (xQueueReceive(s_queue, &r, 0) == pdTRUE) {
    // v2 record:
    // 'C' | ver(1) | mac[6] | rssi | noise | sig_mode | rate | mcs | cwb |
    //   channel | secondary | len(u16 LE) | data[len]
    size_t off = 0;
    pkt[off++] = 'C';
    pkt[off++] = 2;                       // version
    memcpy(pkt + off, r.mac, 6); off += 6;
    pkt[off++] = (uint8_t)r.rssi;
    pkt[off++] = (uint8_t)r.noise_floor;
    pkt[off++] = r.sig_mode;
    pkt[off++] = r.rate;
    pkt[off++] = r.mcs;
    pkt[off++] = r.cwb;
    pkt[off++] = r.channel;
    pkt[off++] = r.secondary;
    pkt[off++] = (uint8_t)(r.len & 0xFF);
    pkt[off++] = (uint8_t)((r.len >> 8) & 0xFF);
    memcpy(pkt + off, r.data, r.len); off += r.len;

    _udp.beginPacket(_controller, _port);
    _udp.write(pkt, off);
    if (_udp.endPacket() == 0) break;   // buffers full: leave rest for next loop()
    if (++sent >= 8) break;             // cap burst per loop() to ease lwIP
  }

  // Per-second diagnostics: captured = CSI fired, forwarded = sent to laptop.
  static uint32_t lastLog = 0, fwd = 0;
  fwd += sent;
  uint32_t now = millis();
  if (now - lastLog >= 1000) {
    Serial.printf("[CSI] espnow_rx %lu/s  captured %lu/s  forwarded %lu/s\n",
                  (unsigned long)s_espnowRx, (unsigned long)s_csiCount, (unsigned long)fwd);
    s_espnowRx = 0; s_csiCount = 0; fwd = 0; lastLog = now;
  }
}