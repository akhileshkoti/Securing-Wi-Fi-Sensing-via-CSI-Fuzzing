#pragma once
#include <Arduino.h>
#include <WiFiUdp.h>
#include <IPAddress.h>

// Illuminator side: periodically sends small UDP packets to the Receiver so
// the Receiver's PHY produces CSI. Payload carries this board's identity (id
// + MAC) so the Receiver can map a source MAC to a friendly id if it wants.
class UdpBeacon {
public:
  void begin(IPAddress serverIp, uint16_t port);
  void loop();   // call every loop(); sends at BEACON_INTERVAL_MS

private:
  WiFiUDP   _udp;
  IPAddress _ip;
  uint16_t  _port = 0;     // 0 = not configured -> loop() is a no-op
  uint32_t  _last = 0;
  uint32_t  _seq  = 0;
};