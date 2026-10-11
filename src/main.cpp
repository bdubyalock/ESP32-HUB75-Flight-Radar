#include <WiFi.h>
#include <Arduino.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <ESP32-HUB75-MatrixPanel-I2S-DMA.h>
#include <time.h>
#include <math.h>
#include <cstring>
#include <strings.h>

#include "config.h"

enum LayoutSlotKind { LAYOUT_TEXT, LAYOUT_COMPASS };

struct LayoutSlot {
  LayoutSlotKind kind;
  const char *text;
  uint16_t color;
};

MatrixPanel_I2S_DMA *display = nullptr;

uint16_t colBlack, colGreen, colYellow, colRed, colWhite;
uint16_t colCyan, colAmber;
uint16_t colTextPrimary, colTextSecondary;
uint16_t colCallsign, colType, colRoute, colStats;
uint16_t colStatsCaution, colGroundDim;
uint16_t colRadarDim, colRadarSweep;

unsigned long lastQuietAnimMs = 0;
uint8_t radarSweep = 0;

const unsigned long RADAR_ANIM_MS = 70;

unsigned long lastFetchMs = 0;
bool screensaverActive = false;
bool wifiEverConnected = false;
unsigned long lastWifiAttemptMs = 0;
String nearestCallsign = "----";
String nearestAircraftType = "----";
String cachedRouteCallsign = "";
char routeOrigin[4] = "---";
char routeDest[4] = "---";
bool hasRoute = false;
float nearestBearing = 0.0;
float nearestTrack = -1.0f;
float nearestDistanceKm = 0.0;
float nearestGroundSpeedKts = 0.0;
int nearestAltitudeFt = 0;
int nearestVerticalRateFpm = 0;
bool nearestOnGround = false;
bool nearestIsHelicopter = false;

const float KM_TO_NM = 0.539957f;
const int LOW_ALT_CAUTION_FT = 3000;  // altitude below this shows amber
int lastApiHttpCode = 0;
bool lastApiParseFailed = false;
bool hasFlightData = false;
int consecutiveFetchFailures = 0;
const int API_ERROR_FAIL_THRESHOLD = 3;

const int FLIGHT_FETCH_RETRIES = 3;
const int ROUTE_FETCH_RETRIES = 3;
const int MAX_TRACKED_AIRCRAFT = 8;
const unsigned long ROUTE_FETCH_DELAY_MS = 250;
const unsigned long FETCH_RETRY_ON_FAIL_MS = 10000;

struct AircraftSnapshot {
  char callsign[12];
  char type[8];
  float distanceKm;
  float bearing;
  float track;
  float groundSpeedKts;
  int altitudeFt;
  int verticalRateFpm;
  bool onGround;
  bool isHelicopter;
};

AircraftSnapshot trackedAircraft[MAX_TRACKED_AIRCRAFT];
int trackedAircraftCount = 0;
int displayAircraftIndex = 0;
unsigned long lastDisplayCycleMs = 0;
unsigned long aircraftDisplayStartedMs = 0;
bool bottomShowsRoutePhase = false;
String lastRenderedCallsign = "";

void refreshFlightDisplay();
void showFlight();
uint8_t activeBrightness();
void clearRoute();
void resetAircraftDisplayTimer();
void copyStringField(char *dest, size_t destSize, const char *src);

void setupDisplay() {
  HUB75_I2S_CFG mxconfig(PANEL_RES_X, PANEL_RES_Y, PANEL_CHAIN);

  if (TRINITY_E_PIN >= 0) {
    mxconfig.gpio.e = TRINITY_E_PIN;
  }
  mxconfig.clkphase = TRINITY_CLKPHASE;

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
#if PANEL_LATCH_BLANKING > 1
  display->setLatBlanking(PANEL_LATCH_BLANKING);
#endif
  display->setBrightness8(PANEL_BRIGHTNESS);
  display->clearScreen();

  colBlack = display->color565(0, 0, 0);
  colGreen = display->color565(0, 255, 0);
  colYellow = display->color565(255, 255, 0);
  colRed = display->color565(255, 0, 0);
  colWhite = display->color565(255, 255, 255);
  colCyan = display->color565(0, 200, 255);
  colAmber = display->color565(255, 140, 0);

  // Semantic stats colors (applied per-value on the stats row).
  colStatsCaution = colAmber;                   // low altitude
  colGroundDim = display->color565(110, 110, 110);  // on-ground "GND"

#if USE_RED_TEXT_ONLY
  colCallsign = colType = colRoute = colStats = colRed;
  colTextPrimary = colRed;
  colTextSecondary = colRed;
#else
  // Glass-cockpit scheme: who / what / where / performance.
  colCallsign = colWhite;       // row 1: callsign
  colType = colCyan;            // row 2: aircraft type
  colRoute = colAmber;          // row 3: route / position
  colStats = colGreen;          // row 4: speed / altitude (nominal)
  colTextPrimary = colGreen;    // boot / status / error screens
  colTextSecondary = colCyan;
#endif
  colRadarDim = display->color565(0, 35, 0);
  colRadarSweep = display->color565(0, 200, 0);
}

const int TEXT_CHAR_W = 6;
const int TEXT_CHAR_H = 8;

bool textRowFits(int y) {
  return y >= 0 && (y + TEXT_CHAR_H) <= PANEL_RES_Y;
}

int maxCharsThatFit(int startX, int endX) {
  if (endX > PANEL_RES_X) {
    endX = PANEL_RES_X;
  }
  int available = endX - startX;
  if (available <= 0) {
    return 0;
  }
  return available / TEXT_CHAR_W;
}

void copyTruncated(char *dest, size_t destSize, const char *src, int maxChars) {
  if (destSize == 0) {
    return;
  }
  if (maxChars <= 0) {
    dest[0] = '\0';
    return;
  }
  size_t len = strlen(src);
  if (len > (size_t)maxChars) {
    len = (size_t)maxChars;
  }
  strncpy(dest, src, len);
  dest[len] = '\0';
}

void printClipped(int x, int y, const char *text, uint16_t color, int endX) {
  if (!textRowFits(y)) {
    return;
  }
  char buf[40];
  int maxChars = maxCharsThatFit(x, endX);
  copyTruncated(buf, sizeof(buf), text, maxChars);
  display->setTextSize(1);
  display->setTextWrap(false);
  display->setTextColor(color);
  display->setCursor(x, y);
  display->print(buf);
}

void printClipped(int x, int y, const char *text, uint16_t color) {
  printClipped(x, y, text, color, PANEL_RES_X);
}

void printAtColumn(int x, int y, const char *text, uint16_t color) {
  if (!textRowFits(y)) {
    return;
  }
  display->setTextSize(1);
  display->setTextWrap(false);
  display->setTextColor(color);
  display->setCursor(x, y);
  display->print(text);
}

bool formatFits(const char *text, int maxChars) {
  return maxChars > 0 && text != nullptr &&
         (int)strlen(text) <= maxChars;
}

void copyFirstFittingFormat(char *dest, size_t destSize, int maxChars,
                            const char *const *options, int optionCount) {
  for (int i = 0; i < optionCount; i++) {
    if (formatFits(options[i], maxChars)) {
      copyStringField(dest, destSize, options[i]);
      return;
    }
  }
  if (optionCount > 0) {
    copyStringField(dest, destSize, options[optionCount - 1]);
  } else {
    dest[0] = '\0';
  }
}

void printClippedCentered(int areaX, int areaW, int y, const char *text,
                          uint16_t color) {
  char buf[40];
  int maxChars = areaW / TEXT_CHAR_W;
  copyTruncated(buf, sizeof(buf), text, maxChars);
  int textW = (int)strlen(buf) * TEXT_CHAR_W;
  int x = areaX;
  if (textW < areaW) {
    x = areaX + (areaW - textW) / 2;
  }
  printClipped(x, y, buf, color, areaX + areaW);
}

void showStatus(const char *line1, const char *line2) {
  display->fillScreen(colBlack);
  printClipped(1, 2, line1, colTextSecondary);
  printClipped(1, 14, line2, colTextPrimary);
}

#if COLOR_TEST_ON_BOOT
void runColorTest() {
  const uint16_t colBlue = display->color565(0, 0, 255);
  const uint16_t cols[] = {colRed, colGreen, colBlue, colYellow, colWhite};
  const char *names[] = {"RED", "GREEN", "BLUE", "YELLOW", "WHITE"};
  for (int i = 0; i < 5; i++) {
    display->fillScreen(cols[i]);
    display->setTextSize(1);
    display->setTextWrap(false);
    display->setTextColor(colBlack);
    display->setCursor(2, 2);
    display->print(names[i]);
    Serial.printf("Color test: screen says %s\n", names[i]);
    delay(2500);
  }
}
#endif

void drawRadarIdle(uint8_t sweep) {
  display->fillScreen(colBlack);

  const int cx = 31;
  const int cy = 12;
  const int radius = 11;

  display->drawCircle(cx, cy, 4, colRadarDim);
  display->drawCircle(cx, cy, 8, colRadarDim);
  display->drawCircle(cx, cy, radius, colRadarDim);
  display->drawLine(cx - radius, cy, cx + radius, cy, colRadarDim);
  display->drawLine(cx, cy - radius, cx, cy + radius, colRadarDim);

  float angle = (sweep % 256) * 2.0f * PI / 256.0f;
  int x2 = cx + (int)(cosf(angle) * radius);
  int y2 = cy + (int)(sinf(angle) * radius);
  display->drawLine(cx, cy, x2, y2, colRadarSweep);

  printClipped(2, 24, "NO TRAFFIC", colRadarDim);
}

void initQuietHours() {
  display->setBrightness8(activeBrightness());
  lastQuietAnimMs = millis();
  radarSweep = 0;
}

void updateQuietHoursDisplay() {
  display->setBrightness8(activeBrightness());

  if (millis() - lastQuietAnimMs < RADAR_ANIM_MS) {
    return;
  }
  lastQuietAnimMs = millis();
  radarSweep += 2;
  drawRadarIdle(radarSweep);
}

void showApiError() {
  char detail[12] = "Retrying";
  if (lastApiParseFailed) {
    strncpy(detail, "Parse err", sizeof(detail) - 1);
  } else if (lastApiHttpCode == 429) {
    strncpy(detail, "Rate lim", sizeof(detail) - 1);
  } else if (lastApiHttpCode > 0) {
    snprintf(detail, sizeof(detail), "HTTP %d", lastApiHttpCode);
  } else if (lastApiHttpCode < 0) {
    snprintf(detail, sizeof(detail), "Conn %d", lastApiHttpCode);
  }
  showStatus("API ERR", detail);
}

void uppercasePrefix(char *dest, const char *src, size_t len) {
  size_t i = 0;
  for (; i < len - 1 && src[i] != '\0'; i++) {
    char c = src[i];
    if (c >= 'a' && c <= 'z') {
      c -= 32;
    }
    dest[i] = c;
  }
  dest[i] = '\0';
}

void drawAirlinePrefixInBox(int boxX, int boxY, int boxW, int boxH,
                            const char *prefix, uint16_t color) {
  int prefixLen = (int)strlen(prefix);
  int maxChars = boxW / TEXT_CHAR_W;
  if (prefixLen > maxChars) {
    prefixLen = maxChars;
  }
  if (prefixLen <= 0) {
    return;
  }

  // Each glyph cell is TEXT_CHAR_W wide but the last column is blank spacing.
  // Center on the actual ink width (drop the final trailing column) so the
  // text doesn't sit on the left border with a phantom gap on the right.
  int textW = prefixLen * TEXT_CHAR_W - 1;
  int textX = boxX + (boxW - textW) / 2;
  int textY = boxY + (boxH - TEXT_CHAR_H) / 2 + 1;

  display->setTextSize(1);
  display->setTextWrap(false);
  display->setTextColor(color);
  display->setCursor(textX, textY);
  for (int i = 0; i < prefixLen; i++) {
    display->write(prefix[i]);
  }
}

// Logo tile starts at column 0 and is 19x15 so its center lands exactly on
// (x+9, y+7) — that odd-sized box is what keeps every emblem truly centered.
const int FLIGHT_ICON_X = 0;
const int FLIGHT_ICON_Y = 0;
const int FLIGHT_ICON_W = 19;
const int FLIGHT_ICON_H = 15;
const int FLIGHT_TEXT_X = 20;
const int FLIGHT_TEXT_W = PANEL_RES_X - FLIGHT_TEXT_X;
// 32px-tall panel layout (8px text rows, y + 8 must be <= 32):
// y=0 callsign | y=8 type | y=15 divider | y=16 detail row | y=24 stats row
// Detail + stats rows use dynamic centered columns with even gaps.
const int FLIGHT_ROW_CALLSIGN = 0;
const int FLIGHT_ROW_TYPE = 8;
const int FLIGHT_ROW_DIVIDER = 15;
const int FLIGHT_ROW_BOTTOM = 16;
const int FLIGHT_ROW_STATS = 24;

const int ROW_GAP_MIN = 4;
const int COMPASS_ARROW_W = 8;

int textWidthPx(const char *text) {
  if (text == nullptr) {
    return 0;
  }
  return (int)strlen(text) * TEXT_CHAR_W;
}

int layoutSlotWidth(const LayoutSlot &slot) {
  switch (slot.kind) {
    case LAYOUT_TEXT:
      return textWidthPx(slot.text);
    case LAYOUT_COMPASS:
      return COMPASS_ARROW_W;
  }
  return 0;
}

bool rowLayoutFitsEven(const int *widths, int count) {
  int total = 0;
  for (int i = 0; i < count; i++) {
    total += widths[i];
  }
  return total <= PANEL_RES_X;
}

bool rowLayoutFits(const int *widths, int count, int minGap) {
  int total = 0;
  for (int i = 0; i < count; i++) {
    total += widths[i];
  }
  return total + minGap * (count + 1) <= PANEL_RES_X;
}

int rowTextBudgetPx(int textFieldCount, bool hasCompass) {
  int itemCount = textFieldCount + (hasCompass ? 1 : 0);
  int fixedW = 0;
  if (hasCompass) {
    fixedW += COMPASS_ARROW_W;
  }
  int gapSpace = ROW_GAP_MIN * (itemCount + 1);
  int budget = PANEL_RES_X - fixedW - gapSpace;
  if (budget < textFieldCount * TEXT_CHAR_W) {
    return textFieldCount * TEXT_CHAR_W;
  }
  return budget;
}

int fitMaxCharsPerTextField(int textFieldCount, bool hasCompass) {
  return rowTextBudgetPx(textFieldCount, hasCompass) /
         textFieldCount / TEXT_CHAR_W;
}

bool statsRowFits(const char *speed, const char *alt) {
  int widths[2] = {textWidthPx(speed), textWidthPx(alt)};
  return rowLayoutFitsEven(widths, 2);
}

bool positionRowFits(const char *heading, const char *dist) {
  int widths[3] = {textWidthPx(heading), textWidthPx(dist), COMPASS_ARROW_W};
  return rowLayoutFitsEven(widths, 3);
}

void drawBitmapRW(int ox, int oy, const char *const *rows, int rowCount,
                  uint16_t redColor, uint16_t whiteColor) {
  for (int row = 0; row < rowCount; row++) {
    for (int col = 0; rows[row][col] != '\0'; col++) {
      char pixel = rows[row][col];
      if (pixel == 'R') {
        display->drawPixel(ox + col, oy + row, redColor);
      } else if (pixel == 'W') {
        display->drawPixel(ox + col, oy + row, whiteColor);
      }
    }
  }
}

void drawBitmapGW(int ox, int oy, const char *const *rows, int rowCount,
                  uint16_t greenColor, uint16_t whiteColor) {
  for (int row = 0; row < rowCount; row++) {
    for (int col = 0; rows[row][col] != '\0'; col++) {
      char pixel = rows[row][col];
      if (pixel == 'G') {
        display->drawPixel(ox + col, oy + row, greenColor);
      } else if (pixel == 'W') {
        display->drawPixel(ox + col, oy + row, whiteColor);
      }
    }
  }
}

void drawBitmapBWR(int ox, int oy, const char *const *rows, int rowCount,
                   uint16_t blueColor, uint16_t whiteColor, uint16_t redColor) {
  for (int row = 0; row < rowCount; row++) {
    for (int col = 0; rows[row][col] != '\0'; col++) {
      uint16_t color;
      switch (rows[row][col]) {
        case 'B':
          color = blueColor;
          break;
        case 'W':
          color = whiteColor;
          break;
        case 'R':
          color = redColor;
          break;
        default:
          continue;
      }
      display->drawPixel(ox + col, oy + row, color);
    }
  }
}

// Air Canada rondelle — 19x17 source art, center-cropped to 15 rows for the
// 19x15 logo tile (R = red, W = white).
void drawAcCanadaLogo(int x, int y) {
  uint16_t acRed = display->color565(217, 0, 27);
  static const char *rows[] = {
      "RWWWWWWWWWWWWWWWWWR",
      "RWWWWWWWWRWWWWWWWWR",
      "RWWWRWWWRRRWWWRWWWR",
      "RWWRRRRWRRRWRRRRWWR",
      "RWWWRRRWRRRWRRRWWWR",
      "RWWWRRRRRRRRRRRWWWR",
      "RWWWRRRRRRRRRRRWWWR",
      "RWWWWWRRRRRRRWWWWWR",
      "RWWRRRRRRRRRRRRRWWR",
      "RWRRRRRRRRRRRRRRRWR",
      "RWWRRRRWWRWWRRRRWWR",
      "RWWWWWWWWRWWWWWWWWR",
      "RWWWWWWWWRWWWWWWWWR",
      "RWWWWWWWWRWWWWWWWWR",
      "RWWWWWWWWWWWWWWWWWR",
  };
  const int logoH = (int)(sizeof(rows) / sizeof(rows[0]));
  drawBitmapRW(x, y, rows, logoH, acRed, colWhite);
}

// Endeavor Air — 19x17 source art, top-cropped to 15 rows for the logo tile
// (B = blue border, W = white, R = red swoosh).
void drawEdvLogo(int x, int y) {
  uint16_t edvBlue = display->color565(40, 110, 200);
  uint16_t edvRed = display->color565(220, 0, 40);
  static const char *rows[] = {
      "BBBBBBBBBBBBBBBBBBB",
      "BWWWWWWWWWWWWWWWWWB",
      "BWWWWWWWWWWWWWWWWWB",
      "BWWWWWWWWWWWWWWWWWB",
      "BWWWWWWWWWWWWWWWWWB",
      "BWWWWWWWWWWWWWWWRRB",
      "BWWWWWWWWWWWWWWRRWB",
      "BWWWWWWWWWWWWWRRWWB",
      "BWWWWWWWWWWWRRRWWWB",
      "BWWWWWWWWWRRRWWWRRB",
      "BRRWWWWWWRRWWWRRRWB",
      "BWRRRWWWRRWWRRRWWWB",
      "BWWWRRRRRRRRRWWWWWB",
      "BWWWWWWWWWWWWWWWWWB",
      "BWWWWWWWWWWWWWWWWWB",
  };
  const int logoH = (int)(sizeof(rows) / sizeof(rows[0]));
  drawBitmapBWR(x, y, rows, logoH, edvBlue, colWhite, edvRed);
}

// Piedmont Air — PDT Partners emblem (19x15, B = blue, W = white, R = red border).
void drawPdtLogo(int x, int y) {
  uint16_t pdtBlue = display->color565(30, 90, 180);
  static const char *rows[] = {
      "RRRRRRRRRRRRRRRRRRR",
      "RWWWWWWWWWWWWWWWWWR",
      "RWWWWWWWWWWWWWWBBWR",
      "RWWWWWWWWWWWWWBWWWR",
      "RWWWWWWWWWWWWBWWBWR",
      "RWWWWWWWWWWWBWWBWWR",
      "RWWWWWWWWWWBWWBWWWR",
      "RWWWWWWWWWBWWBWWBWR",
      "RWWWWWWWWBWWBWWBWWR",
      "RWWWWWWWBWWBWWBWWWR",
      "RWWBBBBBBBBBBBBBBBR",
      "RWBBBBBBBBBBBBBBBWR",
      "RWWWWWWWWWWWWWWWWWR",
      "RWWWWWWWWWWWWWWWWWR",
      "RRRRRRRRRRRRRRRRRRR",
  };
  const int logoH = (int)(sizeof(rows) / sizeof(rows[0]));
  drawBitmapBWR(x, y, rows, logoH, pdtBlue, colWhite, colRed);
}

// Aer Lingus — shamrock on white (19x15, G = green, W = white).
void drawEinLogo(int x, int y) {
  uint16_t eiGreen = display->color565(0, 115, 75);
  static const char *rows[] = {
      "WWWWWWWWWWWWWWWWWWW",
      "WWWWWWWWGWGWWWWWWWW",
      "WWWWWWWGGGGGWWWWWWW",
      "WWWWWWWGGGGGWWWWWWW",
      "WWWWWWWWGGGWWWWWWWW",
      "WWWWGGWWWGWWWGGWWWW",
      "WWWGGGGGWGWGGGGGWWW",
      "WWWWGGGGGGGGGGGWWWW",
      "WWWGGGGGGGGGGGGGWWW",
      "WWWWGGWWWGWWWGGWWWW",
      "WWWWWWWWWGWWWWWWWWW",
      "WWWWWWWWWGWWWWWWWWW",
      "WWWWWWWWGWWWWWWWWWW",
      "WWWWWWWWWWWWWWWWWWW",
      "WWWWWWWWWWWWWWWWWWW",
  };
  const int logoH = (int)(sizeof(rows) / sizeof(rows[0]));
  drawBitmapGW(x, y, rows, logoH, eiGreen, colWhite);
}

// Saudia — Saudi emblem (palm tree + crossed swords) on white, green border
// (G = green, W = white).
void drawSvaLogo(int x, int y) {
  uint16_t svaGreen = display->color565(0, 132, 61);
  static const char *rows[] = {
      "GGGGGGGGGGGGGGGGGGG",
      "GWWWWWWWWWWWWWWWWWG",
      "GWWWWWGGGWGGGWWWWWG",
      "GWWWWGWWWGWWWGWWWWG",
      "GWWWWWWGGGGGWWWWWWG",
      "GWWWWWGWGWGWGWWWWWG",
      "GWWWWWWWWWWWWWWWWWG",
      "GWWWGGGWWWWWGGGWWWG",
      "GWWWWGGGGWGGGGWWWWG",
      "GWWWWWGGGGGGGWWWWWG",
      "GWWWWWWGGGGGWWWWWWG",
      "GWWWWWWWGGGWWWWWWWG",
      "GWWWWWWWWGWWWWWWWWG",
      "GWWWWWWWWWWWWWWWWWG",
      "GGGGGGGGGGGGGGGGGGG",
  };
  const int logoH = (int)(sizeof(rows) / sizeof(rows[0]));
  drawBitmapGW(x, y, rows, logoH, svaGreen, colWhite);
}

// Monochrome bitmap (X = foreground, . = background).
void drawBitmapMono(int ox, int oy, const char *const *rows, int rowCount,
                    uint16_t fgColor, uint16_t bgColor) {
  for (int row = 0; row < rowCount; row++) {
    for (int col = 0; rows[row][col] != '\0'; col++) {
      char pixel = rows[row][col];
      if (pixel == 'X') {
        display->drawPixel(ox + col, oy + row, fgColor);
      } else if (pixel == '.') {
        display->drawPixel(ox + col, oy + row, bgColor);
      }
    }
  }
}

// Breeze Airways — dark blue tile with light blue checkmark + "EZ" (19x15 art).
void drawMxyLogo(int x, int y) {
  uint16_t mxyDark = display->color565(0, 25, 91);
  uint16_t mxyLight = display->color565(198, 226, 244);
  static const char *rows[] = {
      "...................",
      "...................",
      "............XX.....",
      "...........XX......",
      ".....XX....XX......",
      "......XX.XX........",
      ".......XXX.........",
      "...................",
      "...XXXX......XXXX..",
      "...X..X.........X..",
      "...XXXX.......X....",
      "...X..........X....",
      "...XXXX......XXXX..",
      "...................",
      "...................",
  };
  const int logoH = (int)(sizeof(rows) / sizeof(rows[0]));
  drawBitmapMono(x, y, rows, logoH, mxyLight, mxyDark);
}

// Republic Airways — white "R" with seven dots on black (17x15 art).
void drawRpaLogo(int x, int y) {
  static const char *rows[] = {
      ".................",
      ".......X.........",
      "............X....",
      "...X.............",
      ".................",
      "..........XXX....",
      "..........X..X...",
      ".X........XXX....",
      "..........X.X....",
      "..........X..X...",
      "..........X..X...",
      "...X......X..X...",
      "............X....",
      "........X........",
      ".................",
  };
  const int logoW = 17;
  const int logoH = (int)(sizeof(rows) / sizeof(rows[0]));
  display->fillRect(x, y, FLIGHT_ICON_W, FLIGHT_ICON_H, colBlack);
  drawBitmapMono(x + (FLIGHT_ICON_W - logoW) / 2, y, rows, logoH, colWhite,
                 colBlack);
}

// Caribbean Airlines — coral hummingbird on white (19x15 art).
void drawBwaLogo(int x, int y) {
  uint16_t bwaCoral = display->color565(230, 95, 85);
  static const char *rows[] = {
      "...................",
      "............X......",
      "...........X.......",
      "..........XX.......",
      "....XXX..XX........",
      "..XXXXXX.XX........",
      ".....XXXXX.........",
      "......XXXX.........",
      ".......XXXX........",
      "........XXX........",
      "........XXX........",
      ".........XXX.......",
      "...........X.......",
      "...................",
      "...................",
  };
  const int logoH = (int)(sizeof(rows) / sizeof(rows[0]));
  drawBitmapMono(x, y, rows, logoH, bwaCoral, colWhite);
}

// United Airlines — globe lattice on blue (19x15, B = blue, W = white).
void drawUalLogo(int x, int y) {
  uint16_t ualBlue = display->color565(0, 55, 130);
  static const char *rows[] = {
      "BBBBBBBBBBBBBBBBBBB",
      "WWWWWBBBBBBBBBBBBBB",
      "BBWBWWWWBBBBBBBBBBB",
      "BWBBBWBBWWBBBBBBBBB",
      "WBWBWBBWBBWBBBBBBBB",
      "BBBWBBBWBBBWBBBBBBB",
      "WBWBWBWBWBWBWBBBBBB",
      "BWBBBWBBBWBBBWBBBBB",
      "WBWBWBWBWBWBWBWBBBB",
      "BBBWBBBWBBBWBBBWBBB",
      "WBWBWBWBWBWBWBWWWBB",
      "BWBBBWBBBWBBBWBBWBB",
      "WBWBWBWBWBWBWBWBBBB",
      "BBBWBBBWBBBWBBBWBWB",
      "WBWBWBWBWBWBWBWBWBB",
  };
  const int logoH = (int)(sizeof(rows) / sizeof(rows[0]));
  drawBitmapBWR(x, y, rows, logoH, ualBlue, colWhite, colRed);
}

// Jazz Aviation — red "jazz" script on white (19x15, R = red, W = white).
void drawJzaLogo(int x, int y) {
  uint16_t jazzRed = display->color565(217, 0, 27);
  static const char *rows[] = {
      "WWWWWWWWWWWWWWWWWWW",
      "WWWWWWWWWWWWWWWWWWW",
      "WWWWWWWWWWWWWWWWWWW",
      "WWRWWWWWWWWWWWWWWWW",
      "WWWWWWWWWWWWWWWWWWW",
      "WWWRWWRRWWRRRWRRRWW",
      "WWWRWRWWRWWWWRWWWRW",
      "WWRRRWWWRWWWRWWWRWW",
      "WRWRWWRRRRWRWWWRWWW",
      "RRWRWWWWWWWRRRWRRRW",
      "WRRWWWWWWWWWWWWWWWW",
      "WWWWWWWWWWWWWWWWWWW",
      "WWWWWWWWWWWWWWWWWWW",
      "WWWWWWWWWWWWWWWWWWW",
      "WWWWWWWWWWWWWWWWWWW",
  };
  const int logoH = (int)(sizeof(rows) / sizeof(rows[0]));
  drawBitmapRW(x, y, rows, logoH, jazzRed, colWhite);
}

void drawHelicopterLogo(int x, int y) {
  const int w = FLIGHT_ICON_W;
  const int h = FLIGHT_ICON_H;
  const int cx = x + w / 2;
  const int cy = y + h / 2;
  display->fillRect(x, y, w, h, colBlack);
  display->drawCircle(cx, cy, 5, colWhite);
  drawAirlinePrefixInBox(x, y, w, h, "H", colWhite);
}

void drawAirlineSymbol(const char *callsign, bool isHelicopter) {
  const int x = FLIGHT_ICON_X;
  const int y = FLIGHT_ICON_Y;
  const int w = FLIGHT_ICON_W;
  const int h = FLIGHT_ICON_H;

  char prefix[4];
  uppercasePrefix(prefix, callsign, sizeof(prefix));

  // Every emblem is built around the box center so it stays centered.
  const int cx = x + w / 2;  // 9
  const int cy = y + h / 2;  // 7

  if (isHelicopter) {
    drawHelicopterLogo(x, y);
    return;
  }

  // Delta — navy tail with the 3D red "widget" (two-tone triangle).
  if (strncmp(prefix, "DAL", 3) == 0) {
    display->fillRect(x, y, w, h, display->color565(0, 32, 91));
    display->fillTriangle(cx, y + 2, x + 2, y + h - 3, cx, y + h - 3,
                          display->color565(198, 12, 48));
    display->fillTriangle(cx, y + 2, cx, y + h - 3, x + w - 3, y + h - 3,
                          display->color565(110, 0, 28));
    return;
  }
  // Copa — deep blue with a white swoosh.
  if (strncmp(prefix, "CMP", 3) == 0) {
    display->fillRect(x, y, w, h, display->color565(0, 56, 150));
    display->drawLine(x + 2, y + h - 4, x + w - 3, y + 3, colWhite);
    display->drawLine(x + 2, y + h - 3, x + w - 3, y + 4, colWhite);
    return;
  }
  // JetBlue — navy tail with the white "blueberry" dot pattern.
  if (strncmp(prefix, "JBU", 3) == 0) {
    display->fillRect(x, y, w, h, display->color565(0, 51, 153));
    display->fillRect(x + 3, y + 3, 2, 2, colWhite);
    display->fillRect(cx - 1, y + 6, 2, 2, colWhite);
    display->fillRect(x + w - 5, y + 3, 2, 2, colWhite);
    display->fillRect(x + 5, y + h - 4, 2, 2, colWhite);
    display->fillRect(x + w - 7, y + h - 4, 2, 2, colWhite);
    return;
  }
  // Jazz Aviation — red "jazz" script on white (19x15 art).
  if (strncmp(prefix, "JZA", 3) == 0) {
    drawJzaLogo(x, y);
    return;
  }
  // Southwest — bold blue with the warm-red heart.
  if (strncmp(prefix, "SWA", 3) == 0) {
    uint16_t heart = display->color565(216, 32, 57);
    display->fillRect(x, y, w, h, display->color565(0, 75, 145));
    display->fillCircle(cx - 2, y + 5, 2, heart);
    display->fillCircle(cx + 2, y + 5, 2, heart);
    display->fillTriangle(cx - 4, y + 6, cx + 4, y + 6, cx, y + 11, heart);
    return;
  }
  // United Airlines — globe lattice on blue (19x15 art).
  if (strncmp(prefix, "UAL", 3) == 0) {
    drawUalLogo(x, y);
    return;
  }
  // American — flight symbol from an explicit cell map (rows A-N, cols 1-16):
  // blue wing (upper-left), white eagle head (middle), red ribbon (lower-right)
  // on a black tile with a white border.
  if (strncmp(prefix, "AAL", 3) == 0) {
    uint16_t aaBlue = display->color565(0, 120, 210);
    uint16_t aaRed = display->color565(213, 0, 28);
    display->fillRect(x, y, w, h, colBlack);
    display->drawRect(x, y, w, h, colWhite);
    // Blue wing (rows B-G).
    display->drawFastHLine(x + 3, y + 1, 2, aaBlue);
    display->drawFastHLine(x + 3, y + 2, 3, aaBlue);
    display->drawFastHLine(x + 4, y + 3, 3, aaBlue);
    display->drawFastHLine(x + 4, y + 4, 4, aaBlue);
    display->drawFastHLine(x + 5, y + 5, 4, aaBlue);
    display->drawFastHLine(x + 6, y + 6, 4, aaBlue);
    // White eagle head (rows H-J).
    display->drawFastHLine(x + 6, y + 7, 7, colWhite);
    display->drawFastHLine(x + 7, y + 8, 5, colWhite);
    display->drawFastHLine(x + 8, y + 9, 2, colWhite);
    // Red ribbon (rows J-N).
    display->drawPixel(x + 10, y + 9, aaRed);
    display->drawFastHLine(x + 8, y + 10, 4, aaRed);
    display->drawFastHLine(x + 9, y + 11, 4, aaRed);
    display->drawFastHLine(x + 10, y + 12, 5, aaRed);
    display->drawFastHLine(x + 11, y + 13, 5, aaRed);
    return;
  }
  // Frontier — modern stylized green "F" (three forward-slanted bars) on white.
  if (strncmp(prefix, "FFT", 3) == 0) {
    uint16_t fgreen = display->color565(13, 107, 70);
    display->fillRect(x, y, w, h, colWhite);
    // Top bar (full width).
    display->drawFastHLine(x + 6, y + 0, 12, fgreen);
    display->drawFastHLine(x + 5, y + 1, 12, fgreen);
    display->drawFastHLine(x + 4, y + 2, 12, fgreen);
    display->drawFastHLine(x + 3, y + 3, 12, fgreen);
    // Middle bar (full width).
    display->drawFastHLine(x + 5, y + 5, 12, fgreen);
    display->drawFastHLine(x + 4, y + 6, 12, fgreen);
    display->drawFastHLine(x + 3, y + 7, 12, fgreen);
    display->drawFastHLine(x + 2, y + 8, 12, fgreen);
    // Bottom bar (short, left-aligned).
    display->drawFastHLine(x + 4, y + 10, 8, fgreen);
    display->drawFastHLine(x + 3, y + 11, 8, fgreen);
    display->drawFastHLine(x + 2, y + 12, 8, fgreen);
    display->drawFastHLine(x + 1, y + 13, 8, fgreen);
    return;
  }
  // Spirit — bright yellow tail with a centered black wordmark bar.
  if (strncmp(prefix, "NKS", 3) == 0) {
    display->fillRect(x, y, w, h, display->color565(255, 221, 0));
    display->fillRect(x + 3, cy - 1, w - 6, 3, colBlack);
    return;
  }
  // Air Canada — red rondelle with white field and red maple leaf (17x19 art).
  if (strncmp(prefix, "ACA", 3) == 0) {
    drawAcCanadaLogo(x, y);
    return;
  }
  // Endeavor Air — blue-bordered tile with red swoosh (19x17 art).
  if (strncmp(prefix, "EDV", 3) == 0) {
    drawEdvLogo(x, y);
    return;
  }
  // Piedmont Air — red-bordered tile with blue PDT emblem (19x15 art).
  if (strncmp(prefix, "PDT", 3) == 0) {
    drawPdtLogo(x, y);
    return;
  }
  // Republic Airways — white "R" ring on black (17x15 art).
  if (strncmp(prefix, "RPA", 3) == 0) {
    drawRpaLogo(x, y);
    return;
  }
  // Breeze Airways — dark blue tile with light blue checkmark + "EZ" (19x15 art).
  if (strncmp(prefix, "MXY", 3) == 0) {
    drawMxyLogo(x, y);
    return;
  }
  // Air Canada Rouge — black tail with a bright red rondelle.
  if (strncmp(prefix, "ROU", 3) == 0) {
    display->fillRect(x, y, w, h, display->color565(15, 15, 15));
    display->fillCircle(cx, cy, 5, display->color565(230, 0, 40));
    display->fillCircle(cx, cy, 2, colWhite);
    return;
  }
  // British Airways — navy tail with the red/white "speedmarque" ribbon.
  if (strncmp(prefix, "BAW", 3) == 0) {
    display->fillRect(x, y, w, h, display->color565(0, 24, 70));
    display->drawLine(x + 1, y + h - 5, x + w - 2, y + 4, colWhite);
    display->drawLine(x + 1, y + h - 3, x + w - 2, y + 6,
                      display->color565(210, 0, 40));
    return;
  }
  // Caribbean Airlines — coral hummingbird on white (19x15 art).
  if (strncmp(prefix, "BWA", 3) == 0) {
    drawBwaLogo(x, y);
    return;
  }
  // Aer Lingus — green shamrock on white (19x15 art).
  if (strncmp(prefix, "EIN", 3) == 0) {
    drawEinLogo(x, y);
    return;
  }
  // Virgin Atlantic — red tail with a white sweep.
  if (strncmp(prefix, "VIR", 3) == 0) {
    display->fillRect(x, y, w, h, display->color565(210, 0, 40));
    display->fillTriangle(x + 2, y + h - 2, x + w - 2, y + 2, x + w - 2, cy + 1,
                          colWhite);
    return;
  }
  // Air France — white tail with symmetric blue + red vertical bands.
  if (strncmp(prefix, "AFR", 3) == 0) {
    display->fillRect(x, y, w, h, colWhite);
    display->fillRect(x + 3, y, 4, h, display->color565(0, 30, 100));
    display->fillRect(x + w - 7, y, 4, h, display->color565(210, 0, 40));
    return;
  }
  // KLM — sky blue tail with the white crown.
  if (strncmp(prefix, "KLM", 3) == 0) {
    display->fillRect(x, y, w, h, display->color565(0, 160, 220));
    display->drawFastHLine(cx - 5, y + 10, 11, colWhite);
    display->fillCircle(cx - 4, y + 7, 1, colWhite);
    display->fillCircle(cx, y + 6, 1, colWhite);
    display->fillCircle(cx + 4, y + 7, 1, colWhite);
    display->drawPixel(cx, y + 4, colWhite);
    return;
  }
  // Lufthansa — navy tail with the yellow medallion + blue crane.
  if (strncmp(prefix, "DLH", 3) == 0) {
    uint16_t navy = display->color565(0, 30, 80);
    display->fillRect(x, y, w, h, navy);
    display->fillCircle(cx, cy, 5, display->color565(255, 200, 0));
    display->drawLine(cx - 2, cy + 2, cx + 2, cy - 3, navy);
    display->drawLine(cx + 2, cy - 3, cx + 3, cy + 1, navy);
    return;
  }
  // Emirates — UAE flag tail (red hoist bar, green/white/black bands).
  if (strncmp(prefix, "UAE", 3) == 0) {
    display->fillRect(x, y, w, h, colWhite);
    display->fillRect(x, y, w, 5, display->color565(0, 122, 61));
    display->fillRect(x, y + h - 5, w, 5, colBlack);
    display->fillRect(x, y, 4, h, display->color565(206, 17, 38));
    return;
  }
  // Saudia — green-bordered tile with the Saudi emblem (19x15 art).
  if (strncmp(prefix, "SVA", 3) == 0) {
    drawSvaLogo(x, y);
    return;
  }
  // Qatar Airways — maroon tail with a centered white oryx + horns.
  if (strncmp(prefix, "QTR", 3) == 0) {
    display->fillRect(x, y, w, h, display->color565(94, 0, 60));
    display->fillRect(cx - 3, cy - 1, 7, 5, colWhite);
    display->drawLine(cx - 1, cy - 1, cx - 2, y + 2, colWhite);
    display->drawLine(cx + 1, cy - 1, cx + 2, y + 2, colWhite);
    return;
  }
  // El Al — white tail with blue stripes and a centered Star of David.
  if (strncmp(prefix, "ELY", 3) == 0) {
    uint16_t elblue = display->color565(0, 80, 170);
    display->fillRect(x, y, w, h, colWhite);
    display->fillRect(x, y, w, 2, elblue);
    display->fillRect(x, y + h - 2, w, 2, elblue);
    display->drawTriangle(cx, y + 3, cx - 3, y + 9, cx + 3, y + 9, elblue);
    display->drawTriangle(cx, y + 11, cx - 3, y + 5, cx + 3, y + 5, elblue);
    return;
  }
  // Alaska — midnight tail with the centered fur-hooded face.
  if (strncmp(prefix, "ASA", 3) == 0) {
    display->fillRect(x, y, w, h, display->color565(0, 40, 70));
    display->fillCircle(cx, cy, 4, display->color565(150, 110, 80));
    display->drawCircle(cx, cy, 5, colWhite);
    display->drawCircle(cx, cy, 6, colWhite);
    display->drawPixel(cx - 2, cy - 1, colBlack);
    display->drawPixel(cx + 2, cy - 1, colBlack);
    display->drawFastHLine(cx - 2, cy + 2, 5, colBlack);
    return;
  }

  // Unknown carrier (incl. GA): white square outline on black, prefix centered.
  display->fillRect(x, y, w, h, colBlack);
  display->drawRect(x, y, w, h, colWhite);
  if (strlen(prefix) >= 3) {
    drawAirlinePrefixInBox(x, y, w, h, prefix, colWhite);
  }
}

const char *bearingLabel(float bearing) {
  static const char *labels[] = {
      "N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
      "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"};
  int idx = (int)((bearing + 11.25f) / 22.5f) & 15;
  return labels[idx];
}

// Snap a bearing to one of 8 compass points: 0=N,1=NE,2=E,3=SE,4=S,5=SW,6=W,7=NW.
int bearingOctant(float bearing) {
  return ((int)((bearing + 22.5f) / 45.0f)) & 7;
}

float nearestDistanceNm() {
  return nearestDistanceKm * KM_TO_NM;
}

void formatDistanceForColumn(char *buf, size_t buflen, int maxChars) {
  float distNm = nearestDistanceNm();
  char full[12];
  char oneDecimal[12];
  char whole[12];

  snprintf(full, sizeof(full), "%.1fnm", distNm);
  snprintf(whole, sizeof(whole), "%dnm", (int)(distNm + 0.5f));
  snprintf(oneDecimal, sizeof(oneDecimal), "%.0fnm", distNm);

  const char *options[] = {full, whole, oneDecimal};
  copyFirstFittingFormat(buf, buflen, maxChars, options, 3);
}

void formatHeadingForColumn(char *buf, size_t buflen, int maxChars) {
  if (nearestTrack < 0.0f) {
    const char *options[] = {"---", "--", "-"};
    copyFirstFittingFormat(buf, buflen, maxChars, options, 3);
    return;
  }

  char full[8];
  char oct[8];
  char one[4];
  copyStringField(full, sizeof(full), bearingLabel(nearestTrack));

  static const char *octLabels[] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
  copyStringField(oct, sizeof(oct), octLabels[bearingOctant(nearestTrack)]);
  one[0] = full[0];
  one[1] = '\0';

  const char *options[] = {full, oct, one};
  copyFirstFittingFormat(buf, buflen, maxChars, options, 3);
}

bool copyIfFormatFits(char *dest, size_t destSize, int maxChars, const char *text) {
  if (formatFits(text, maxChars)) {
    copyStringField(dest, destSize, text);
    return true;
  }
  return false;
}

int countDigits(int value) {
  if (value == 0) {
    return 1;
  }
  if (value < 0) {
    value = -value;
  }
  int digits = 0;
  while (value > 0) {
    digits++;
    value /= 10;
  }
  return digits;
}

// Format value + suffix, keeping suffix intact; coarsen the number if needed.
void formatIntWithSuffix(char *buf, size_t buflen, int maxChars, int value,
                         const char *suffix) {
  char candidate[16];
  snprintf(candidate, sizeof(candidate), "%d%s", value, suffix);
  if (copyIfFormatFits(buf, buflen, maxChars, candidate)) {
    return;
  }

  int suffixLen = (int)strlen(suffix);
  int digitSlots = maxChars - suffixLen;
  if (digitSlots < 1) {
    copyStringField(buf, buflen, suffix);
    return;
  }

  int divisor = 1;
  while (value / divisor >= 1) {
    int compact = value / divisor;
    snprintf(candidate, sizeof(candidate), "%d%s", compact, suffix);
    if (copyIfFormatFits(buf, buflen, maxChars, candidate)) {
      return;
    }
    if (countDigits(compact) <= digitSlots) {
      copyStringField(buf, buflen, candidate);
      return;
    }
    if (divisor > 1000000000) {
      break;
    }
    divisor *= 10;
  }

  int div = 1;
  for (int len = countDigits(value); len > digitSlots; len--) {
    div *= 10;
  }
  snprintf(buf, buflen, "%d%s", value / div, suffix);
}

void formatSpeedForColumn(char *buf, size_t buflen, int maxChars) {
  int kts = (int)(nearestGroundSpeedKts + 0.5f);
  formatIntWithSuffix(buf, buflen, maxChars, kts, "kt");
}

void formatAltitudeForColumn(char *buf, size_t buflen, int maxChars) {
  if (maxChars <= 0) {
    buf[0] = '\0';
    return;
  }
  if (nearestOnGround) {
    copyStringField(buf, buflen, "GND");
    return;
  }

  char opt[16];
  int alt = nearestAltitudeFt;

  if (alt >= 18000) {
    snprintf(opt, sizeof(opt), "FL%03d", alt / 100);
    if (copyIfFormatFits(buf, buflen, maxChars, opt)) {
      return;
    }
  }

  snprintf(opt, sizeof(opt), "%dft", alt);
  if (copyIfFormatFits(buf, buflen, maxChars, opt)) {
    return;
  }

  if (alt >= 1000) {
    float kft = alt / 1000.0f;
    snprintf(opt, sizeof(opt), "%.1fkft", kft);
    if (copyIfFormatFits(buf, buflen, maxChars, opt)) {
      return;
    }
    snprintf(opt, sizeof(opt), "%dkft", alt / 1000);
    if (copyIfFormatFits(buf, buflen, maxChars, opt)) {
      return;
    }
  }

  formatIntWithSuffix(buf, buflen, maxChars, alt, "ft");
}

// Stats row is green when nominal; amber/red flag noteworthy states.
uint16_t altitudeColor() {
  if (nearestOnGround) {
    return colGroundDim;
  }
  if (nearestAltitudeFt > 0 && nearestAltitudeFt < LOW_ALT_CAUTION_FT) {
    return colStatsCaution;
  }
  return colStats;
}

// Small arrow centered at (cx, cy) pointing toward a compass octant, showing
// where the aircraft sits relative to the receiver (e.g. SW -> down-left).
void drawCompassArrow(int cx, int cy, int octant, uint16_t color) {
  static const int dirs[8][2] = {
      {0, -1},   // N
      {1, -1},   // NE
      {1, 0},    // E
      {1, 1},    // SE
      {0, 1},    // S
      {-1, 1},   // SW
      {-1, 0},   // W
      {-1, -1},  // NW
  };
  int dx = dirs[octant & 7][0];
  int dy = dirs[octant & 7][1];
  const int r = 2;
  int tipX = cx + dx * r;
  int tipY = cy + dy * r;
  display->drawLine(cx - dx * r, cy - dy * r, tipX, tipY, color);
  // Two barbs angled back from the tip to form the arrowhead.
  display->drawLine(tipX, tipY, tipX - dx - dy, tipY - dy + dx, color);
  display->drawLine(tipX, tipY, tipX - dx + dy, tipY - dy - dx, color);
}

void drawLayoutSlot(const LayoutSlot &slot, int x, int y, int slotW) {
  switch (slot.kind) {
    case LAYOUT_TEXT: {
      int textX = x + (slotW - textWidthPx(slot.text)) / 2;
      printAtColumn(textX, y, slot.text, slot.color);
      break;
    }
    case LAYOUT_COMPASS:
      drawCompassArrow(x + slotW / 2, y + TEXT_CHAR_H / 2,
                       bearingOctant(nearestBearing), slot.color);
      break;
  }
}

void drawCenteredColumnRow(int y, LayoutSlot *slots, int count) {
  if (count <= 0) {
    return;
  }

  int widths[6];
  int totalW = 0;
  for (int i = 0; i < count; i++) {
    widths[i] = layoutSlotWidth(slots[i]);
    totalW += widths[i];
  }

  int gap = (PANEL_RES_X - totalW) / (count + 1);
  if (gap < 0) {
    gap = 0;
  }

  int x = gap;
  for (int i = 0; i < count; i++) {
    drawLayoutSlot(slots[i], x, y, widths[i]);
    x += widths[i] + gap;
  }
}

void prepareStatsFields(char *speed, size_t speedLen, char *alt, size_t altLen) {
  const int fullWidth = 20;

  formatSpeedForColumn(speed, speedLen, fullWidth);
  formatAltitudeForColumn(alt, altLen, fullWidth);
  if (statsRowFits(speed, alt)) {
    return;
  }

  // Keep full speed; shorten altitude before touching speed.
  formatSpeedForColumn(speed, speedLen, fullWidth);
  if (!nearestOnGround && nearestAltitudeFt >= 1000) {
    char opt[16];
    int altVal = nearestAltitudeFt;

    snprintf(opt, sizeof(opt), "%.1fkft", altVal / 1000.0f);
    copyStringField(alt, altLen, opt);
    if (statsRowFits(speed, alt)) {
      return;
    }

    snprintf(opt, sizeof(opt), "%dkft", altVal / 1000);
    copyStringField(alt, altLen, opt);
    if (statsRowFits(speed, alt)) {
      return;
    }
  }

  formatSpeedForColumn(speed, speedLen, fullWidth);
  formatAltitudeForColumn(alt, altLen, fullWidth);
}

void preparePositionFields(char *heading, size_t headingLen, char *dist,
                           size_t distLen) {
  const int fullWidth = 20;

  formatHeadingForColumn(heading, headingLen, fullWidth);
  formatDistanceForColumn(dist, distLen, fullWidth);
  if (positionRowFits(heading, dist)) {
    return;
  }

  for (int maxChars = 10; maxChars >= 2; maxChars--) {
    formatHeadingForColumn(heading, headingLen, maxChars);
    formatDistanceForColumn(dist, distLen, maxChars);
    if (positionRowFits(heading, dist)) {
      return;
    }
  }

  formatHeadingForColumn(heading, headingLen, fullWidth);
  formatDistanceForColumn(dist, distLen, fullWidth);
}

void parseAltitudeFromPlane(JsonObject plane, bool &onGround, int &altitudeFt) {
  onGround = false;
  altitudeFt = 0;

  if (plane["alt_baro"].is<const char *>()) {
    const char *altText = plane["alt_baro"];
    if (altText && strcasecmp(altText, "ground") == 0) {
      onGround = true;
      return;
    }
  }

  int alt = plane["alt_baro"] | plane["alt_geom"] | 0;
  if (alt <= 0) {
    onGround = true;
    return;
  }
  altitudeFt = alt;
}

void parseFlightTelemetry(JsonObject plane) {
  nearestGroundSpeedKts = plane["gs"] | 0.0;
  nearestVerticalRateFpm = plane["baro_rate"] | plane["geom_rate"] | 0;
  parseAltitudeFromPlane(plane, nearestOnGround, nearestAltitudeFt);
}

void copyStringField(char *dest, size_t destSize, const char *src) {
  if (destSize == 0) {
    return;
  }
  if (src == nullptr) {
    dest[0] = '\0';
    return;
  }
  strncpy(dest, src, destSize - 1);
  dest[destSize - 1] = '\0';
}

bool parsePlaneSnapshot(JsonObject plane, AircraftSnapshot &snap) {
  memset(&snap, 0, sizeof(snap));

  const char *flight = plane["flight"].as<const char *>();
  String callsign = flight ? flight : "";
  callsign.trim();
  if (callsign.length() == 0) {
    const char *reg = plane["r"].as<const char *>();
    if (reg) {
      callsign = reg;
      callsign.trim();
    }
  }
  if (callsign.length() == 0) {
    const char *hex = plane["hex"].as<const char *>();
    if (hex) {
      callsign = hex;
    }
  }
  if (callsign.length() == 0) {
    return false;
  }

  copyStringField(snap.callsign, sizeof(snap.callsign), callsign.c_str());

  const char *type = plane["t"] | "----";
  copyStringField(snap.type, sizeof(snap.type), type);

  const char *category = plane["category"] | "";
  snap.isHelicopter = (strncmp(category, "A7", 2) == 0);

  snap.distanceKm = plane["dst"] | 9999.0f;
  snap.bearing = plane["dir"] | 0.0f;
  snap.track = plane["track"] | -1.0f;
  snap.groundSpeedKts = plane["gs"] | 0.0f;
  snap.verticalRateFpm = plane["baro_rate"] | plane["geom_rate"] | 0;
  parseAltitudeFromPlane(plane, snap.onGround, snap.altitudeFt);
#if TRACK_AIRBORNE_ONLY
  if (snap.onGround) {
    return false;
  }
#endif
  return true;
}

void swapTrackedAircraft(int a, int b) {
  AircraftSnapshot tmp = trackedAircraft[a];
  trackedAircraft[a] = trackedAircraft[b];
  trackedAircraft[b] = tmp;
}

bool aircraftRanksBefore(const AircraftSnapshot &a, const AircraftSnapshot &b) {
  if (a.isHelicopter != b.isHelicopter) {
    return !a.isHelicopter;
  }
  return a.distanceKm < b.distanceKm;
}

void insertTrackedAircraft(const AircraftSnapshot &snap) {
  if (trackedAircraftCount < MAX_TRACKED_AIRCRAFT) {
    trackedAircraft[trackedAircraftCount++] = snap;
  } else if (!aircraftRanksBefore(snap, trackedAircraft[MAX_TRACKED_AIRCRAFT - 1])) {
    return;
  } else {
    trackedAircraft[MAX_TRACKED_AIRCRAFT - 1] = snap;
  }

  for (int i = trackedAircraftCount - 1; i > 0; i--) {
    if (aircraftRanksBefore(trackedAircraft[i], trackedAircraft[i - 1])) {
      swapTrackedAircraft(i, i - 1);
    } else {
      break;
    }
  }
}

void clearTrackedAircraft() {
  trackedAircraftCount = 0;
  displayAircraftIndex = 0;
}

void applyTrackedAircraft(int index) {
  if (index < 0 || index >= trackedAircraftCount) {
    nearestCallsign = "NONE";
    nearestAircraftType = "----";
    nearestDistanceKm = 0.0;
    nearestBearing = 0.0;
    nearestGroundSpeedKts = 0.0;
    nearestAltitudeFt = 0;
    nearestVerticalRateFpm = 0;
    nearestOnGround = false;
    nearestIsHelicopter = false;
    return;
  }

  const AircraftSnapshot &snap = trackedAircraft[index];
  nearestCallsign = snap.callsign;
  nearestAircraftType = snap.type;
  nearestDistanceKm = snap.distanceKm;
  nearestBearing = snap.bearing;
  nearestTrack = snap.track;
  nearestGroundSpeedKts = snap.groundSpeedKts;
  nearestAltitudeFt = snap.altitudeFt;
  nearestVerticalRateFpm = snap.verticalRateFpm;
  nearestOnGround = snap.onGround;
  nearestIsHelicopter = snap.isHelicopter;
}

void cycleDisplayedAircraft() {
  if (trackedAircraftCount <= 1) {
    return;
  }

  displayAircraftIndex = (displayAircraftIndex + 1) % trackedAircraftCount;
  applyTrackedAircraft(displayAircraftIndex);
  resetAircraftDisplayTimer();

  if (nearestCallsign != cachedRouteCallsign) {
    clearRoute();
  }

  Serial.printf("Cycle %d/%d: %s %s\n",
                displayAircraftIndex + 1, trackedAircraftCount,
                nearestCallsign.c_str(), nearestAircraftType.c_str());
  refreshFlightDisplay();
}

bool bottomShowsRouteView() {
  if (!hasRoute) {
    return false;
  }
  return (millis() - aircraftDisplayStartedMs) < (DISPLAY_CYCLE_MS / 2);
}

void resetAircraftDisplayTimer() {
  aircraftDisplayStartedMs = millis();
  bottomShowsRoutePhase = bottomShowsRouteView();
}

void updateBottomPhaseIfNeeded() {
  bool routePhase = bottomShowsRouteView();
  if (routePhase != bottomShowsRoutePhase) {
    bottomShowsRoutePhase = routePhase;
    showFlight();
  }
}

void drawBottomStatsRow() {
  char speedBuf[12];
  char altBuf[16];
  prepareStatsFields(speedBuf, sizeof(speedBuf), altBuf, sizeof(altBuf));

  LayoutSlot slots[2] = {
      {LAYOUT_TEXT, speedBuf, colStats},
      {LAYOUT_TEXT, altBuf, altitudeColor()},
  };
  drawCenteredColumnRow(FLIGHT_ROW_STATS, slots, 2);
}

void drawBottomRouteRow() {
  char routeBuf[16];
  char routeFit[16];
  snprintf(routeBuf, sizeof(routeBuf), "%s>%s", routeOrigin, routeDest);
  int maxChars = (PANEL_RES_X - ROW_GAP_MIN * 2) / TEXT_CHAR_W;
  copyTruncated(routeFit, sizeof(routeFit), routeBuf, maxChars);
  LayoutSlot slot = {LAYOUT_TEXT, routeFit, colRoute};
  drawCenteredColumnRow(FLIGHT_ROW_BOTTOM, &slot, 1);
}

void drawBottomPositionRow() {
  char headingBuf[8];
  char distBuf[12];
  preparePositionFields(headingBuf, sizeof(headingBuf), distBuf, sizeof(distBuf));

  LayoutSlot slots[3] = {
      {LAYOUT_TEXT, headingBuf, colRoute},
      {LAYOUT_TEXT, distBuf, colRoute},
      {LAYOUT_COMPASS, nullptr, colRoute},
  };
  drawCenteredColumnRow(FLIGHT_ROW_BOTTOM, slots, 3);
}

void drawBottomLine() {
  if (nearestCallsign == "NONE") {
    printClippedCentered(0, PANEL_RES_X, FLIGHT_ROW_BOTTOM, "NO TRAFFIC",
                         colStats);
    return;
  }

  if (bottomShowsRouteView()) {
    drawBottomRouteRow();
  } else {
    drawBottomPositionRow();
  }

  drawBottomStatsRow();
}

void showFlight() {
  display->setBrightness8(activeBrightness());
  display->fillScreen(colBlack);

  drawAirlineSymbol(nearestCallsign.c_str(), nearestIsHelicopter);

  printClippedCentered(FLIGHT_TEXT_X, FLIGHT_TEXT_W, FLIGHT_ROW_CALLSIGN,
                       nearestCallsign.c_str(), colCallsign);
  printClippedCentered(FLIGHT_TEXT_X, FLIGHT_TEXT_W, FLIGHT_ROW_TYPE,
                       nearestAircraftType.c_str(), colType);

  display->drawFastHLine(0, FLIGHT_ROW_DIVIDER, PANEL_RES_X,
                         display->color565(30, 30, 30));

  drawBottomLine();
}

void clearRoute() {
  strncpy(routeOrigin, "---", sizeof(routeOrigin));
  strncpy(routeDest, "---", sizeof(routeDest));
  hasRoute = false;
}

bool parseRouteResponse(const String &payload, const String &callsign) {
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, payload);
  if (error || !doc.is<JsonArray>() || doc.size() == 0) {
    Serial.printf("Route parse failed for %s: %s\n", callsign.c_str(), error.c_str());
    return false;
  }

  const char *iata = doc[0]["_airport_codes_iata"] | "";
  if (strlen(iata) < 7 || strcmp(iata, "unknown") == 0) {
    Serial.printf("Route unknown for %s (iata=%s)\n", callsign.c_str(), iata);
    return false;
  }

  strncpy(routeOrigin, iata, 3);
  routeOrigin[3] = '\0';
  strncpy(routeDest, iata + 4, 3);
  routeDest[3] = '\0';
  hasRoute = true;
  cachedRouteCallsign = callsign;
  Serial.printf("Route for %s: %s>%s\n", callsign.c_str(), routeOrigin, routeDest);
  return true;
}

bool fetchRoute(const String &callsign) {
  if (callsign == "NONE" || callsign.length() < 3) {
    clearRoute();
    cachedRouteCallsign = "";
    return false;
  }

  if (callsign == cachedRouteCallsign && hasRoute) {
    return true;
  }

  clearRoute();

  JsonDocument req;
  JsonArray planes = req["planes"].to<JsonArray>();
  JsonObject plane = planes.add<JsonObject>();
  plane["callsign"] = callsign;

  String body;
  serializeJson(req, body);

  for (int attempt = 1; attempt <= ROUTE_FETCH_RETRIES; attempt++) {
    HTTPClient http;
    http.begin("https://flightroutes.opensky-network.org/api/routeset");
    http.addHeader("Content-Type", "application/json");
    http.setTimeout(8000);

    int statusCode = http.POST(body);
    if (statusCode != HTTP_CODE_OK && statusCode != HTTP_CODE_CREATED) {
      Serial.printf("Route request failed for %s: HTTP %d (try %d/%d)\n",
                    callsign.c_str(), statusCode, attempt, ROUTE_FETCH_RETRIES);
      http.end();
      if (attempt < ROUTE_FETCH_RETRIES) {
        delay(ROUTE_FETCH_DELAY_MS * attempt);
      }
      continue;
    }

    String payload = http.getString();
    http.end();

    if (parseRouteResponse(payload, callsign)) {
      return true;
    }

    if (attempt < ROUTE_FETCH_RETRIES) {
      delay(ROUTE_FETCH_DELAY_MS * attempt);
    }
  }

  Serial.printf("Route lookup gave up for %s\n", callsign.c_str());
  return false;
}

bool fetchAircraftList() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Flight fetch skipped: WiFi not connected");
    return false;
  }

  String url = "https://api.adsb.lol/v2/lat/";
  url += String(RECEIVER_LAT, 6);
  url += "/lon/";
  url += String(RECEIVER_LON, 6);
  url += "/dist/";
  url += String(TRACK_RADIUS_KM);

  String payload;
  int statusCode = 0;

  for (int attempt = 1; attempt <= FLIGHT_FETCH_RETRIES; attempt++) {
    HTTPClient http;
    http.begin(url);
    http.setTimeout(10000);
    http.setUserAgent("Mozilla/5.0");
    http.addHeader("Accept", "application/json");

    statusCode = http.GET();
    if (statusCode != HTTP_CODE_OK) {
      Serial.printf("Flight request failed at URL %s: HTTP %d (try %d/%d)\n", url.c_str(),
                    statusCode, attempt, FLIGHT_FETCH_RETRIES);
       Serial.print("WiFi IP: ");
      Serial.println(WiFi.localIP());
      http.end();
      if (attempt < FLIGHT_FETCH_RETRIES) {
        delay(500 * attempt);
      }
      continue;
    }

    payload = http.getString();
    http.end();
    break;
  }

  if (statusCode != HTTP_CODE_OK) {
    lastApiParseFailed = false;
    lastApiHttpCode = statusCode;
    return false;
  }

  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, payload);
  payload = "";
  if (error) {
    lastApiParseFailed = true;
    lastApiHttpCode = 0;
    Serial.printf("Flight JSON parse failed: %s\n", error.c_str());
    return false;
  }

  lastApiParseFailed = false;
  lastApiHttpCode = 0;

  JsonArray aircraft = doc["ac"].as<JsonArray>();
  clearTrackedAircraft();

  if (!aircraft.isNull()) {
    for (JsonObject plane : aircraft) {
      AircraftSnapshot snap;
      if (parsePlaneSnapshot(plane, snap)) {
        insertTrackedAircraft(snap);
      }
    }
  }

  if (trackedAircraftCount == 0) {
    applyTrackedAircraft(-1);
    cachedRouteCallsign = "";
    clearRoute();
    Serial.println("No aircraft in range");
    return true;
  }

  displayAircraftIndex = 0;
  applyTrackedAircraft(0);

  if (nearestCallsign != cachedRouteCallsign) {
    clearRoute();
  }

  Serial.printf("Tracking %d aircraft within %dkm\n",
                trackedAircraftCount, TRACK_RADIUS_KM);
  for (int i = 0; i < trackedAircraftCount; i++) {
    Serial.printf("  %d) %s %s %.1fkm%s\n", i + 1,
                  trackedAircraft[i].callsign, trackedAircraft[i].type,
                  trackedAircraft[i].distanceKm,
                  trackedAircraft[i].isHelicopter ? " heli" : "");
  }
  return true;
}

void refreshFlightDisplay() {
  hasFlightData = true;

  // No aircraft in range: let the loop draw the screensaver instead of an
  // empty flight card.
  if (trackedAircraftCount == 0 || nearestCallsign == "NONE") {
    return;
  }

  if (nearestCallsign != lastRenderedCallsign) {
    lastRenderedCallsign = nearestCallsign;
    resetAircraftDisplayTimer();
  }

  showFlight();

  if (fetchRoute(nearestCallsign)) {
    showFlight();
  }
}

void handleFlightFetchResult(bool ok) {
  if (ok) {
    consecutiveFetchFailures = 0;
    refreshFlightDisplay();
    return;
  }

  consecutiveFetchFailures++;

  if (!hasFlightData && consecutiveFetchFailures >= API_ERROR_FAIL_THRESHOLD) {
    showApiError();
    return;
  }

  Serial.printf("Flight refresh failed (%d in a row); keeping last display\n",
                consecutiveFetchFailures);
}

bool isQuietHours() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    return false;
  }
  int hour = timeinfo.tm_hour;
  // Handle windows that wrap past midnight (e.g. 22:00-06:00).
  if (QUIET_HOUR_START <= QUIET_HOUR_END) {
    return hour >= QUIET_HOUR_START && hour < QUIET_HOUR_END;
  }
  return hour >= QUIET_HOUR_START || hour < QUIET_HOUR_END;
}

uint8_t activeBrightness() {
  return isQuietHours() ? QUIET_BRIGHTNESS : PANEL_BRIGHTNESS;
}

bool syncTime() {
  configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, "pool.ntp.org", "time.nist.gov");

  struct tm timeinfo;
  for (int i = 0; i < 20; i++) {
    if (getLocalTime(&timeinfo)) {
      Serial.printf("Time synced: %02d:%02d local (quiet=%s)\n",
                    timeinfo.tm_hour, timeinfo.tm_min,
                    isQuietHours() ? "yes" : "no");
      return true;
    }
    delay(500);
  }

  Serial.println("NTP sync failed — polling without quiet hours");
  return false;
}

const char *wifiStatusText(wl_status_t status) {
  switch (status) {
    case WL_IDLE_STATUS: return "IDLE";
    case WL_NO_SSID_AVAIL: return "NO SSID";
    case WL_SCAN_COMPLETED: return "SCAN OK";
    case WL_CONNECTED: return "CONNECTED";
    case WL_CONNECT_FAILED: return "FAIL";
    case WL_CONNECTION_LOST: return "LOST";
    case WL_DISCONNECTED: return "DISCONN";
    default: return "UNKNOWN";
  }
}

bool connectWiFi(unsigned long timeoutMs) {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.disconnect(true);
  delay(100);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  showStatus("WiFi...", WIFI_SSID);

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < timeoutMs) {
    delay(250);
    Serial.printf("WiFi status: %s\n", wifiStatusText(WiFi.status()));
  }

  if (WiFi.status() == WL_CONNECTED) {
    wifiEverConnected = true;
    Serial.print("WiFi connected: ");
    Serial.println(WiFi.localIP());
    showStatus("WiFi OK", WiFi.localIP().toString().c_str());
    delay(1500);
    return true;
  }

  Serial.printf("WiFi failed: %s\n", wifiStatusText(WiFi.status()));
  showStatus("WiFi FAIL", wifiStatusText(WiFi.status()));
  return false;
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("Boot: Flight Radar (G/B swap on)");

  setupDisplay();

#if COLOR_TEST_ON_BOOT
  runColorTest();
#endif

  showStatus("ESP32", "Flight Radar");

  if (!connectWiFi(30000)) {
    return;
  }

  syncTime();

  // Initial fetch + render; loop() then keeps polling on schedule. Planes are
  // shown whenever present (dimmed during quiet hours); otherwise the
  // screensaver runs.
  if (fetchAircraftList()) {
    refreshFlightDisplay();
  } else {
    handleFlightFetchResult(false);
  }
  lastFetchMs = millis();
  lastDisplayCycleMs = millis();

  if (trackedAircraftCount == 0) {
    screensaverActive = true;
    initQuietHours();
    updateQuietHoursDisplay();
  }
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    if (millis() - lastWifiAttemptMs >= 10000) {
      lastWifiAttemptMs = millis();
      if (wifiEverConnected) {
        showStatus("WiFi LOST", wifiStatusText(WiFi.status()));
        WiFi.reconnect();
      } else {
        connectWiFi(15000);
      }
    }
    delay(500);
    return;
  }

  // Poll the API on schedule around the clock. Quiet hours only dim the
  // display (via activeBrightness); they no longer pause polling, so live
  // traffic is shown whenever it exists.
  unsigned long fetchInterval =
      hasFlightData ? REFRESH_INTERVAL_MS : FETCH_RETRY_ON_FAIL_MS;
  if (lastFetchMs == 0 || millis() - lastFetchMs >= fetchInterval) {
    handleFlightFetchResult(fetchAircraftList());
    lastFetchMs = millis();
    lastDisplayCycleMs = millis();
  }

  // Aircraft in range: show them (cycling through multiple), dimmed if quiet.
  if (trackedAircraftCount > 0) {
    if (screensaverActive) {
      screensaverActive = false;
      refreshFlightDisplay();
    }
    display->setBrightness8(activeBrightness());

    if (trackedAircraftCount > 1 &&
        millis() - lastDisplayCycleMs >= DISPLAY_CYCLE_MS) {
      lastDisplayCycleMs = millis();
      cycleDisplayedAircraft();
    } else {
      updateBottomPhaseIfNeeded();
    }
    delay(40);
    return;
  }

  // No aircraft: run the radar screensaver (dimmed during quiet hours).
  if (!screensaverActive) {
    screensaverActive = true;
    initQuietHours();
  }
  updateQuietHoursDisplay();
  delay(40);
}
