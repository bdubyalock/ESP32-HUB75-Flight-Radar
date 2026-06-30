#pragma once

// ESP32 Trinity — https://esp32trinity.com
#define USE_TRINITY_BOARD 1
#define TRINITY_CLKPHASE true

// 64x32: E not needed. 64x64: use 18.
#define TRINITY_E_PIN -1

// 0 = try first on Adafruit; 1 = Trinity FAQ fix if colors are swapped.
// This panel: G/B lines swapped + blue pixel dead, so swap must be ON.
#define SWAP_GREEN_BLUE 1

// FM6126A scrambles this panel (blue shows as dim green) — keep off.
#define USE_FM6126A_DRIVER 0

// 1-4; try 2-3 if top rows flicker or disappear (library default is 1).
#define PANEL_LATCH_BLANKING 2

#define PANEL_BRIGHTNESS 100

// DIAGNOSTIC ONLY — routes the "blue" software channel through the red
// GPIOs (known-good) instead of the normal blue GPIOs (26/27/12/13).
// Set SWAP_GREEN_BLUE to 0 before using this. If FULL BLUE then renders
// RED, blue data is fine and 26/27/12/13 are the problem (hardware).
// If FULL BLUE is still 100% black, the bug is in software, not wiring.
#define DIAGNOSTIC_SWAP_RED_BLUE 0
