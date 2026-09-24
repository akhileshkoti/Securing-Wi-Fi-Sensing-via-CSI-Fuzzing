#include "Discovery.h"
#include "Settings.h"
#include "DeviceId.h"
#include <ESPmDNS.h>

bool Discovery::startResponder() {
  String host = String(Settings::MDNS_HOSTNAME) + "-" + deviceId();
  if (!MDNS.begin(host.c_str())) {
    Serial.println("[mDNS] responder start failed");
    return false;
  }
  MDNS.addService(Settings::MDNS_SERVICE, Settings::MDNS_PROTO, Settings::DATA_PORT);
  Serial.printf("[mDNS] advertising %s.local  _%s._%s :%u\n",
                host.c_str(), Settings::MDNS_SERVICE,
                Settings::MDNS_PROTO, Settings::DATA_PORT);
  return true;
}

bool Discovery::resolveServer(IPAddress &outIp, uint16_t &outPort, uint32_t timeoutMs) {
  String host = String("esp-illuminator-") + deviceId();
  if (!MDNS.begin(host.c_str())) {
    Serial.println("[mDNS] local init failed");
    return false;
  }
  uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    int n = MDNS.queryService(Settings::MDNS_SERVICE, Settings::MDNS_PROTO);
    if (n > 0) {
      outIp   = MDNS.IP(0);
      outPort = MDNS.port(0);
      Serial.printf("[mDNS] found receiver at %s:%u\n",
                    outIp.toString().c_str(), outPort);
      return true;
    }
    Serial.println("[mDNS] receiver not found, retrying...");
    delay(500);
  }
  Serial.println("[mDNS] discovery timed out");
  return false;
}