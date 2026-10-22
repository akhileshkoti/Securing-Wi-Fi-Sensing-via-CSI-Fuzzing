#pragma once
#include <Arduino.h>

// The data the user enters in the captive portal.
enum class Role { Unknown, Illuminator, Receiver, Guardian };

inline Role roleFromString(const String &s) {
  if (s == "Illuminator") return Role::Illuminator;
  if (s == "Receiver")    return Role::Receiver;
  if (s == "Guardian")    return Role::Guardian;
  return Role::Unknown;
}

inline const char *roleToString(Role r) {
  switch (r) {
    case Role::Illuminator: return "Illuminator";
    case Role::Receiver:    return "Receiver";
    case Role::Guardian:    return "Guardian";
    default:                return "Unknown";
  }
}

struct NetConfig {
  String ssid;
  String pass;
  Role   role = Role::Unknown;
  String serverIp;

  bool isValid() const {
    if (ssid.length() == 0 || role == Role::Unknown) return false;
    if (role == Role::Receiver && serverIp.length() == 0) return false; // controller IP
    return true;
  }
};