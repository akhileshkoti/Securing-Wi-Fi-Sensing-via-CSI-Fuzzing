#pragma once
#include <Arduino.h>
#include <IPAddress.h>

// Service discovery over mDNS.
//   Receiver    -> startResponder(): advertise <hostname>.local + _<service>._<proto>
//   Illuminator -> resolveServer():  find that service and return its IP + port
class Discovery {
public:
  bool startResponder();                                   // call on the Receiver
  bool resolveServer(IPAddress &outIp, uint16_t &outPort,  // call on the Illuminator
                     uint32_t timeoutMs);
};