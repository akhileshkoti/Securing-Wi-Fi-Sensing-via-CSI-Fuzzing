#pragma once
#include "NetConfig.h"

// Persists NetConfig to ESP32 NVS (flash) via the Preferences library.
class ConfigStore {
public:
  NetConfig load();
  void      save(const NetConfig &cfg);
  void      clear();
};