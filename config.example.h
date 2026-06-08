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

// HUB75 panel configuration (64x32 on ESP32 Trinity)
#define PANEL_RES_X 64
#define PANEL_RES_Y 32
#define PANEL_CHAIN 1
#define PANEL_BRIGHTNESS 100

// ESP32 Trinity settings — https://esp32trinity.com
#define TRINITY_CLKPHASE false
#define TRINITY_E_PIN -1

// This panel has swapped green/blue data lines.
#define SWAP_GREEN_BLUE 1

// US Eastern: UTC-5 standard, +1h daylight (Mar–Nov)
#define GMT_OFFSET_SEC (-5 * 3600)
#define DAYLIGHT_OFFSET_SEC 3600

// No API polling during these local hours [start, end)
#define QUIET_HOUR_START 0
#define QUIET_HOUR_END 6

// Poll interval during active hours (milliseconds)
#define REFRESH_INTERVAL_MS 300000  // 5 minutes

// Display brightness during quiet hours (0-255)
#define QUIET_BRIGHTNESS 20
