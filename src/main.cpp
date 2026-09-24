#include <Arduino.h>
#include "Settings.h"
#include "NetConfig.h"
#include "ConfigStore.h"
#include "WifiStation.h"
#include "CaptivePortal.h"
#include "Discovery.h"
#include "EspNowBeacon.h"
#include "CsiCollector.h"

// --- Collaborators (composition root) ---
ConfigStore   store;
WifiStation   station;
CaptivePortal portal(store);

Discovery  discovery;
IPAddress  g_serverIp;
uint16_t   g_serverPort = Settings::DATA_PORT;

NetConfig g_cfg;
bool      g_portalMode = false;

EspNowBeacon beacon;
CsiCollector csi;

// Wipe config if the reset pin is held LOW at boot.
static bool resetRequested() {
  pinMode(Settings::RESET_PIN, INPUT_PULLUP);
  delay(50);
  return digitalRead(Settings::RESET_PIN) == LOW;
}

// Runs once we are online in station mode.
static void startApplication() {
  Serial.printf("[App] Online. Role=%s\n", roleToString(g_cfg.role));
  WiFi.setSleep(false);                         // important: no modem sleep for CSI/ESP-NOW

  if (g_cfg.role == Role::Receiver) {
    IPAddress controller;
    controller.fromString(g_cfg.serverIp);
    csi.begin(controller, Settings::CONTROLLER_PORT);
  } else {  // Illuminator
    beacon.begin();                             // ESP-NOW broadcast; no IP/mDNS needed
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);

  if (resetRequested()) {
    Serial.println("[Setup] Reset held -> clearing config");
    store.clear();
  }

  g_cfg = store.load();

  if (g_cfg.isValid() && station.connect(g_cfg, Settings::STA_TIMEOUT_MS)) {
    startApplication();
  } else {
    portal.begin();
    g_portalMode = true;
  }
}

void loop() {
  if (g_portalMode) { portal.loop(); return; }
  if (g_cfg.role == Role::Receiver) csi.loop();
  else                              beacon.loop();
}