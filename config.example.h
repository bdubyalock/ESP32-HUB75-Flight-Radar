#pragma once

// Copy this file to config.h and fill in your values.
// config.h is gitignored and will not be committed.

// WiFi credentials
#define WIFI_SSID "your-ssid"
#define WIFI_PASSWORD "your-password"

// Receiver location (decimal degrees)
#define RECEIVER_LAT 40.7128
#define RECEIVER_LON -74.0060

// Search radius in kilometers (adsb.lol API requires an integer)
#define SEARCH_RADIUS_KM 50

// HUB75 panel configuration
#define PANEL_RES_X 64
#define PANEL_RES_Y 32
#define PANEL_CHAIN 1

// How often to refresh flight data (milliseconds)
#define REFRESH_INTERVAL_MS 30000
