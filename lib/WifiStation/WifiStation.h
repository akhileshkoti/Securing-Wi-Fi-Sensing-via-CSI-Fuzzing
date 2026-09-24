#pragma once
#include "NetConfig.h"

// Connects the ESP to an existing Wi-Fi network (station mode).
class WifiStation {
public:
  // Returns true once connected, false on timeout.
  bool connect(const NetConfig &cfg, uint32_t timeoutMs);
};