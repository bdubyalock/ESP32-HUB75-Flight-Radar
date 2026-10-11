#pragma once

// Copy this file to include/config.h and fill in your values.
// config.h is gitignored and will not be committed.

// WiFi credentials
#define WIFI_SSID "your-ssid"
#define WIFI_PASSWORD "your-password"

// Receiver location (decimal degrees)
#define RECEIVER_LAT 40.747384
#define RECEIVER_LON -73.957026

// Aircraft search radius in km (~5.4nm at 10). Keep <=15 for ESP32 memory.
#define TRACK_RADIUS_KM 10

// Skip parked/taxiing aircraft (alt_baro ground or <=0)
#define TRACK_AIRBORNE_ONLY 1

// Refresh aircraft list from API (milliseconds)
#define REFRESH_INTERVAL_MS 150000  // 2.5 minutes

// Rotate displayed aircraft every N ms (no extra API call)
#define DISPLAY_CYCLE_MS 20000

// HUB75 panel configuration (64x32 on ESP32 Trinity)
#define PANEL_RES_X 64
#define PANEL_RES_Y 32
#define PANEL_CHAIN 1
#define PANEL_BRIGHTNESS 100

// ESP32 Trinity settings — https://esp32trinity.com
// Adafruit 64x32 (2277): Trinity PCB swaps G/B data lines — swap must be on.
#define TRINITY_CLKPHASE false
#define TRINITY_E_PIN -1
#define SWAP_GREEN_BLUE 1

// Set to 1 if colors are still wrong (some newer panels use FM6126A chips).
#define USE_FM6126A_DRIVER 0

// 1-4; try 2-3 if top rows flicker or disappear (library default is 1).
#define PANEL_LATCH_BLANKING 2

// Set to 1 only if green/yellow text channels are bad on your panel
#define USE_RED_TEXT_ONLY 0

// US Eastern: UTC-5 standard, +1h daylight (Mar–Nov)
#define GMT_OFFSET_SEC (-5 * 3600)
#define DAYLIGHT_OFFSET_SEC 3600

// No API polling during these local hours [start, end). May wrap past midnight.
#define QUIET_HOUR_START 22
#define QUIET_HOUR_END 6

// Display brightness during quiet hours: 20% of normal brightness
#define QUIET_BRIGHTNESS (PANEL_BRIGHTNESS / 5)
