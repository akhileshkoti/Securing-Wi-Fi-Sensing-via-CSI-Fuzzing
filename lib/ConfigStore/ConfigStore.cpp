#include "ConfigStore.h"
#include "Settings.h"
#include <Preferences.h>

NetConfig ConfigStore::load() {
  Preferences p;
  p.begin(Settings::NVS_NAMESPACE, /*readOnly=*/true);
  NetConfig cfg;
  cfg.ssid     = p.getString("ssid", "");
  cfg.pass     = p.getString("pass", "");
  cfg.role     = roleFromString(p.getString("role", ""));
  cfg.serverIp = p.getString("server", "");
  p.end();
  return cfg;
}

void ConfigStore::save(const NetConfig &cfg) {
  Preferences p;
  p.begin(Settings::NVS_NAMESPACE, /*readOnly=*/false);
  p.putString("ssid",   cfg.ssid);
  p.putString("pass",   cfg.pass);
  p.putString("role",   roleToString(cfg.role));
  p.putString("server", cfg.serverIp);
  p.end();
}

void ConfigStore::clear() {
  Preferences p;
  p.begin(Settings::NVS_NAMESPACE, /*readOnly=*/false);
  p.clear();
  p.end();
}