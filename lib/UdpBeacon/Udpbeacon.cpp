#include "UdpBeacon.h"
#include "Settings.h"
#include "DeviceId.h"
#include <WiFi.h>

void UdpBeacon::begin(IPAddress serverIp, uint16_t port) {
  _ip   = serverIp;
  _port = port;
  Serial.printf("[Beacon] sending to %s:%u every %lums\n",
                _ip.toString().c_str(), _port,
                (unsigned long)Settings::BEACON_INTERVAL_MS);
}

void UdpBeacon::loop() {
  if (_port == 0) return;                       // not configured yet
  uint32_t now = millis();
  if (now - _last < Settings::BEACON_INTERVAL_MS) return;
  _last = now;

  // Payload: ILU,<id>,<mac>,<seq>  -> lets the Receiver tie a MAC to an id.
  _udp.beginPacket(_ip, _port);
  _udp.printf("ILU,%s,%s,%lu",
              deviceId().c_str(),
              WiFi.macAddress().c_str(),
              (unsigned long)_seq++);
  _udp.endPacket();
}