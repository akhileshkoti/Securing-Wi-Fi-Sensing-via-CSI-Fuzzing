#pragma once
#include <Arduino.h>
#include <WiFiUdp.h>
#include <IPAddress.h>

// Receiver side: enables ESP32 CSI capture. The hardware callback fires for
// each received frame and includes the SOURCE MAC, which separates packets
// per illuminator. Records are queued and forwarded to the controller in
// loop() (the callback itself must stay light, so it only enqueues).
class CsiCollector {
public:
  void begin(IPAddress controllerIp, uint16_t port);
  void loop();   // drains the queue and forwards each record

private:
  WiFiUDP   _udp;
  IPAddress _controller;
  uint16_t  _port = 0;
};