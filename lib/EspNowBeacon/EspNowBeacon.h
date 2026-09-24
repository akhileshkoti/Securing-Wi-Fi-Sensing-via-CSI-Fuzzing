#pragma once
#include <Arduino.h>

// Illuminator transmitter using ESP-NOW broadcast. No IP/ARP/AP-relay needed:
// frames go peer-to-peer over the air, so the Receiver's CSI callback fires
// with THIS board's MAC. Requires both boards on the same Wi-Fi channel
// (true automatically when both are connected to the same AP).
class EspNowBeacon {
public:
  bool begin();   // init ESP-NOW + add broadcast peer
  void loop();    // send at Settings::BEACON_INTERVAL_MS

private:
  uint32_t _last  = 0;
  uint32_t _seq   = 0;
  bool     _ready = false;
};