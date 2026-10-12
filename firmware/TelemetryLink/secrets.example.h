// Wi-Fi networks the car tries, in order (WiFiMulti picks the strongest available).
//
// 1. Copy this file to "secrets.h" in the same sketch folder.
// 2. Fill in your own networks.
// secrets.h is listed in .gitignore, so your passwords never reach GitHub.
#pragma once

struct WifiNetwork {
  const char* ssid;
  const char* password;
};

static const WifiNetwork WIFI_NETWORKS[] = {
  {"YOUR_SSID", "YOUR_PASSWORD"},
  // {"PHONE_HOTSPOT", "ITS_PASSWORD"},   // optional fallback
};
