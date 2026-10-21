// Standalone entry point for the Guardian prototype board (CSI-fuzzing
// defense, Objective 3). Built only by `pio run -e guardian` -- excluded
// from the main Illuminator/Receiver build (see platformio.ini) so this
// experimental code can never affect the already-tested pipeline that
// `src/main.cpp` builds.
//
// Flash: edit include/GuardianSettings.h (Wi-Fi credentials + PRNG seed),
// then `pio run -e guardian -t upload`.
#include <Arduino.h>
#include <WiFi.h>
#include "GuardianBeacon.h"
#include "GuardianSettings.h"

GuardianBeacon guardian;

void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.printf("[Guardian] connecting to %s...\n", GuardianSettings::WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(GuardianSettings::WIFI_SSID, GuardianSettings::WIFI_PASS);
  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
    delay(300);
    Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[Guardian] Wi-Fi connect failed -- check GuardianSettings.h");
  } else {
    Serial.printf("[Guardian] connected, channel=%d\n", WiFi.channel());
  }
  WiFi.setSleep(false);

  guardian.begin(GuardianSettings::PRNG_SEED);
}

void loop() {
  guardian.loop();
}
