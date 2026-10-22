#pragma once
#include <Arduino.h>

// Transmits short ESP-NOW broadcast bursts on a schedule derived from a
// shared PRNG seed, so only someone who knows the seed can tell which time
// slots carried "guardian noise" versus genuine Illuminator traffic -- the
// core trick behind the CSI-fuzzing defense (Objective 3).
//
// Selected like any other role via the captive portal (Role::Guardian);
// main.cpp calls begin()/loop() the same way it does for EspNowBeacon and
// CsiCollector. It does not touch those classes' own logic. See
// controller/fuzzing_eval.py for the matching schedule function and the
// protection/degradation evaluation this feeds.
class GuardianBeacon {
public:
  void begin(uint32_t seed);
  void loop();

private:
  uint32_t _seed = 0;
  int64_t  _epochMsAtBoot = 0;   // unix ms at millis()==0, from one-time SNTP sync; 0 = not yet synced
  uint32_t _lastSlot = 0xFFFFFFFF;
  uint32_t _seq = 0;
};
