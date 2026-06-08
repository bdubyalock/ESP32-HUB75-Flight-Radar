#pragma once

// ESP32 Trinity — https://esp32trinity.com
#define USE_TRINITY_BOARD 1
#define TRINITY_CLKPHASE false

// 64x32: E not needed. 64x64: use 18.
#define TRINITY_E_PIN -1

// Required for this panel — green/blue data lines are swapped.
#define SWAP_GREEN_BLUE 1

// Keep OFF — turning this on broke red+green on your panel.
#define USE_FM6126A_DRIVER 0

#define PANEL_BRIGHTNESS 100
