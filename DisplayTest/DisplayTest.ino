// 64x32 HUB75 display test for ESP32 Trinity.
// Uses the library's native drawing calls (same style as Trinity examples).

#include <ESP32-HUB75-MatrixPanel-I2S-DMA.h>
#include "panel_config.h"

#define PANEL_RES_X 64
#define PANEL_RES_Y 32
#define PANEL_CHAIN 1

MatrixPanel_I2S_DMA *display = nullptr;

uint16_t colBlack, colRed, colGreen, colBlue, colYellow, colWhite;

enum TestScreen {
  SCREEN_FULL_RED,
  SCREEN_FULL_GREEN,
  SCREEN_FULL_BLUE,
  SCREEN_FULL_YELLOW,
  SCREEN_ROWS,
  SCREEN_COLORS,
  SCREEN_BORDER,
  SCREEN_TEXT,
  SCREEN_FLIGHT,
  SCREEN_GRID,
  SCREEN_COUNT
};

uint8_t currentScreen = 0;
unsigned long screenStartedMs = 0;
const unsigned long HOLD_MS = 3500;

void setupDisplay() {
  HUB75_I2S_CFG mxconfig(PANEL_RES_X, PANEL_RES_Y, PANEL_CHAIN);

#if USE_TRINITY_BOARD
  if (TRINITY_E_PIN >= 0) {
    mxconfig.gpio.e = TRINITY_E_PIN;
  }
  mxconfig.clkphase = TRINITY_CLKPHASE;
#endif

#if SWAP_GREEN_BLUE
  mxconfig.gpio.b1 = 26;
  mxconfig.gpio.b2 = 12;
  mxconfig.gpio.g1 = 27;
  mxconfig.gpio.g2 = 13;
#endif

#if USE_FM6126A_DRIVER
  mxconfig.driver = HUB75_I2S_CFG::FM6126A;
#endif

  display = new MatrixPanel_I2S_DMA(mxconfig);
  display->begin();
  display->setBrightness8(PANEL_BRIGHTNESS);
  display->clearScreen();

  colBlack = display->color565(0, 0, 0);
  colRed = display->color565(255, 0, 0);
  colGreen = display->color565(0, 255, 0);
  colBlue = display->color565(0, 0, 255);
  colYellow = display->color565(255, 255, 0);
  colWhite = display->color565(255, 255, 255);
}

void drawHeader(const char *title) {
  display->fillRect(0, 0, PANEL_RES_X, 10, colBlack);
  display->setTextSize(1);
  display->setTextWrap(false);
  display->setTextColor(colYellow);
  display->setCursor(1, 1);
  display->print(title);
  display->drawFastHLine(0, 9, PANEL_RES_X, display->color565(40, 40, 40));
}

void showFullRed() {
  display->fillScreen(colRed);
}

void showFullGreen() {
  display->fillScreen(colGreen);
}

void showFullBlue() {
  display->fillScreen(colBlue);
}

void showFullYellow() {
  display->fillScreen(colYellow);
}

void showRows() {
  display->fillScreen(colBlack);
  drawHeader("ROWS");
  display->fillRect(0, 10, PANEL_RES_X, 11, colRed);
  display->fillRect(0, 21, PANEL_RES_X, 11, colGreen);
}

void showColors() {
  display->fillScreen(colBlack);
  drawHeader("COLORS");
  display->fillRect(1, 11, 14, 9, colRed);
  display->fillRect(17, 11, 14, 9, colGreen);
  display->fillRect(33, 11, 14, 9, colBlue);
  display->fillRect(49, 11, 14, 9, colYellow);
}

void showBorder() {
  display->fillScreen(colBlack);
  drawHeader("BORDER");
  display->drawRect(0, 11, PANEL_RES_X, PANEL_RES_Y - 11, colGreen);
  display->drawRect(4, 15, PANEL_RES_X - 8, PANEL_RES_Y - 19, colYellow);
  display->drawLine(0, 11, PANEL_RES_X - 1, PANEL_RES_Y - 1, colRed);
  display->drawLine(PANEL_RES_X - 1, 11, 0, PANEL_RES_Y - 1, colRed);
}

void showText() {
  display->fillScreen(colBlack);
  drawHeader("TEXT");
  display->setTextColor(colGreen);
  display->setCursor(1, 12);
  display->print("ABCDEFGH");
  display->setTextColor(colYellow);
  display->setCursor(1, 22);
  display->print("01234567");
}

void showFlight() {
  display->fillScreen(colBlack);
  drawHeader("FLIGHT");
  display->setTextColor(colGreen);
  display->setCursor(1, 12);
  display->print("UAL123");
  display->setTextColor(colYellow);
  display->setCursor(1, 22);
  display->print("ALT 32000");
}

void showGrid() {
  display->fillScreen(colBlack);
  drawHeader("GRID");
  for (int x = 0; x < PANEL_RES_X; x += 8) {
    display->drawFastVLine(x, 10, PANEL_RES_Y - 10, colGreen);
  }
  for (int y = 10; y < PANEL_RES_Y; y += 4) {
    display->drawFastHLine(0, y, PANEL_RES_X, colRed);
  }
}

const char *screenName(uint8_t screen) {
  switch (screen) {
    case SCREEN_FULL_RED: return "FULL RED";
    case SCREEN_FULL_GREEN: return "FULL GREEN";
    case SCREEN_FULL_BLUE: return "FULL BLUE";
    case SCREEN_FULL_YELLOW: return "FULL YELLOW";
    case SCREEN_ROWS: return "ROWS";
    case SCREEN_COLORS: return "COLORS";
    case SCREEN_BORDER: return "BORDER";
    case SCREEN_TEXT: return "TEXT";
    case SCREEN_FLIGHT: return "FLIGHT";
    case SCREEN_GRID: return "GRID";
    default: return "UNKNOWN";
  }
}

void showScreen(uint8_t screen) {
  switch (screen % SCREEN_COUNT) {
    case SCREEN_FULL_RED: showFullRed(); break;
    case SCREEN_FULL_GREEN: showFullGreen(); break;
    case SCREEN_FULL_BLUE: showFullBlue(); break;
    case SCREEN_FULL_YELLOW: showFullYellow(); break;
    case SCREEN_ROWS: showRows(); break;
    case SCREEN_COLORS: showColors(); break;
    case SCREEN_BORDER: showBorder(); break;
    case SCREEN_TEXT: showText(); break;
    case SCREEN_FLIGHT: showFlight(); break;
    case SCREEN_GRID: showGrid(); break;
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("DisplayTest: ESP32 Trinity 64x32");
  Serial.println("Reverted config — FM6126A off, E pin off, G/B swap on");

  setupDisplay();

  screenStartedMs = millis();
  showScreen(currentScreen);
  Serial.printf("Screen: %s\n", screenName(currentScreen));
}

void loop() {
  if (millis() - screenStartedMs >= HOLD_MS) {
    currentScreen = (currentScreen + 1) % SCREEN_COUNT;
    screenStartedMs = millis();
    showScreen(currentScreen);
    Serial.printf("Screen: %s\n", screenName(currentScreen));
  }
}
