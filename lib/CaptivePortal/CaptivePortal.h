#pragma once
#include <WebServer.h>
#include <DNSServer.h>
#include <IPAddress.h>
#include "NetConfig.h"
#include "ConfigStore.h"
#include "Settings.h"

// Brings up a SoftAP + DNS catch-all + web form so the user can enter config.
// On submit it saves via ConfigStore and reboots into station mode.
class CaptivePortal {
public:
  explicit CaptivePortal(ConfigStore &store);

  void begin();   // start AP, DNS and web server
  void loop();    // call from Arduino loop() while in portal mode

private:
  void handleRoot();
  void handleSave();
  void handleNotFound();

  ConfigStore &_store;
  WebServer    _server{Settings::WEB_PORT};
  DNSServer    _dns;
  IPAddress    _apIP{192, 168, 4, 1};
};