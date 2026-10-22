#pragma once
#include <cstdint>

// Tunables for the Guardian role (CSI-fuzzing defense, Objective 3).
// Wi-Fi credentials come from the captive portal / NetConfig like every
// other role -- only the PRNG schedule parameters live here.
namespace GuardianSettings {
  // Shared secret: must match the --seed given to controller/fuzzing_eval.py
  // for the same capture session. Whoever does not know this value cannot
  // predict which slots carry guardian noise -- that asymmetry is the
  // entire defense. Change it per deployment/experiment.
  constexpr uint32_t PRNG_SEED = 1234567;

  constexpr uint32_t SLOT_MS            = 20;    // schedule granularity, matches Illuminator beacon interval
  constexpr double   BURST_PROBABILITY  = 0.15;  // fraction of slots that carry a guardian burst
  constexpr uint8_t  BURST_FRAMES       = 4;      // frames sent per burst slot
  constexpr uint32_t BURST_FRAME_GAP_MS = 3;      // spacing between frames within one burst
}
