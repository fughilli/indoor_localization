// WiFi + host configuration for the tag. The checked-in defaults are
// placeholders; put real credentials in config_local.h (gitignored) — it
// overrides this file when present in the build.
#ifndef APPS_TAG_CONFIG_H
#define APPS_TAG_CONFIG_H

#if __has_include("apps/tag/config_local.h")
#include "apps/tag/config_local.h"
#endif

#ifndef UWB_WIFI_SSID
#define UWB_WIFI_SSID "your-ssid"
#endif
#ifndef UWB_WIFI_PASSWORD
#define UWB_WIFI_PASSWORD "your-password"
#endif
#ifndef UWB_HOST_IP
#define UWB_HOST_IP "192.168.0.100" // The machine running host/server.py
#endif
#ifndef UWB_HOST_PORT
#define UWB_HOST_PORT 7007
#endif

#endif  // APPS_TAG_CONFIG_H
