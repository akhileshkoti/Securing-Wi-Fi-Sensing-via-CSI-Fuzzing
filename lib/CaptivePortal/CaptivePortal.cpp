#include "CaptivePortal.h"
#include "Settings.h"
#include "portal_html.h"
#include <WiFi.h>
#include "DeviceId.h"

CaptivePortal::CaptivePortal(ConfigStore &store) : _store(store) {}

void CaptivePortal::begin() {
  String apName = String(Settings::AP_SSID_PREFIX) + "-" + deviceId();
  Serial.println("[Portal] Starting SoftAP + captive portal...");
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(_apIP, _apIP, IPAddress(255, 255, 255, 0));
  WiFi.softAP(apName.c_str(), Settings::AP_PASS);
  _dns.start(Settings::DNS_PORT, "*", _apIP);  // route every domain to us
  _server.on("/",     [this]() { handleRoot(); });
  _server.on("/save", HTTP_POST, [this]() { handleSave(); });
  _server.onNotFound([this]() { handleNotFound(); });
  _server.begin();
  Serial.printf("[Portal] Join '%s' then open http://%s\n",
                apName.c_str(), _apIP.toString().c_str());
}

void CaptivePortal::loop() {
  _dns.processNextRequest();
  _server.handleClient();
}

void CaptivePortal::handleRoot() {
  _server.send_P(200, "text/html", PORTAL_FORM_HTML);
}

void CaptivePortal::handleSave() {
  NetConfig cfg;
  cfg.ssid     = _server.arg("ssid");
  cfg.pass     = _server.arg("pass");
  cfg.role     = roleFromString(_server.arg("role"));
  cfg.serverIp = _server.arg("server");

  if (!cfg.isValid()) {
    _server.send(400, "text/html", "<h3>Missing fields. Go back and retry.</h3>");
    return;
  }

  _store.save(cfg);
  _server.send(200, "text/html",
    "<h3>Saved. Rebooting...</h3><p>SSID: " + cfg.ssid +
    "<br>Role: " + roleToString(cfg.role) +
    "<br>Server: " + cfg.serverIp + "</p>");
  delay(1500);
  ESP.restart();
}

void CaptivePortal::handleNotFound() {
  // Redirect unknown hosts to the form so phones pop the captive sheet.
  _server.sendHeader("Location", "http://" + _apIP.toString() + "/", true);
  _server.send(302, "text/plain", "");
}