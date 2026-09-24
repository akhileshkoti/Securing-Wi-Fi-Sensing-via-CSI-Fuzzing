#pragma once
#include <cstdint>

// Central place for all tunable constants. No logic here.
namespace Settings {
  // SoftAP shown when device is unconfigured
  constexpr char AP_SSID_PREFIX[] = "ESP-Setup";   // becomes ESP-Setup-XXXXXX
  constexpr char     AP_PASS[]       = "12345678";   // >=8 chars, or "" for open AP
  constexpr uint16_t WEB_PORT        = 80;
  constexpr uint8_t  DNS_PORT        = 53;

  // Hold this pin LOW at boot to wipe stored config (GPIO0 = BOOT button)
  constexpr uint8_t  RESET_PIN       = 0;

  // How long to wait for the home Wi-Fi before falling back to the portal
  constexpr uint32_t STA_TIMEOUT_MS  = 15000;

  // Preferences namespace key
  constexpr char     NVS_NAMESPACE[] = "netcfg";

  // mDNS / discovery
  constexpr char     MDNS_HOSTNAME[]       = "esp-receiver"; // -> esp-receiver.local
  constexpr char     MDNS_SERVICE[]        = "sensing";      // advertises _sensing._tcp
  constexpr char     MDNS_PROTO[]          = "tcp";
  constexpr uint16_t DATA_PORT             = 4210;
  constexpr uint32_t DISCOVERY_TIMEOUT_MS  = 8000;

  // CSI / data flow
  constexpr uint32_t BEACON_INTERVAL_MS = 20;    // illuminator TX rate (~50 packets/s)
  constexpr uint16_t CONTROLLER_PORT    = 1201;  // receiver -> controller (CSI out)
  constexpr uint16_t MAX_CSI_LEN        = 384;   // cap per-packet CSI bytes
}