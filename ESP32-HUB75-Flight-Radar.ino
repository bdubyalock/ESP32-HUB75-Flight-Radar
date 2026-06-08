#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <ESP32-HUB75-MatrixPanel-I2S-DMA.h>
#include <time.h>
#include <math.h>
#include <cstring>

#include "config.h"

MatrixPanel_I2S_DMA *display = nullptr;

uint16_t colBlack, colGreen, colYellow, colRed, colWhite;
uint16_t colRunwayWhite, colRunwayBlue, colRadarDim, colRadarSweep;

enum QuietScene { QUIET_RUNWAY, QUIET_RADAR };

QuietScene quietScene = QUIET_RUNWAY;
unsigned long quietSceneStartedMs = 0;
unsigned long lastQuietAnimMs = 0;
uint8_t runwayPhase = 0;
uint8_t radarSweep = 0;

const unsigned long QUIET_SCENE_HOLD_MS = 20000;
const unsigned long RUNWAY_ANIM_MS = 140;
const unsigned long RADAR_ANIM_MS = 70;

unsigned long lastFetchMs = 0;
bool wasQuietHours = false;
bool wifiEverConnected = false;
unsigned long lastWifiAttemptMs = 0;
String nearestCallsign = "----";
String nearestAircraftType = "----";
String cachedRouteCallsign = "";
char routeOrigin[4] = "---";
char routeDest[4] = "---";
bool hasRoute = false;
float nearestBearing = 0.0;
float nearestDistanceKm = 0.0;
int lastApiHttpCode = 0;

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

  display = new MatrixPanel_I2S_DMA(mxconfig);
  display->begin();
  display->setBrightness8(PANEL_BRIGHTNESS);
  display->clearScreen();

  colBlack = display->color565(0, 0, 0);
  colGreen = display->color565(0, 255, 0);
  colYellow = display->color565(255, 255, 0);
  colRed = display->color565(255, 0, 0);
  colWhite = display->color565(255, 255, 255);
  colRunwayWhite = display->color565(220, 220, 200);
  colRunwayBlue = display->color565(70, 130, 255);
  colRadarDim = display->color565(0, 35, 0);
  colRadarSweep = display->color565(0, 200, 0);
}

void showStatus(const char *line1, const char *line2) {
  display->fillScreen(colBlack);
  display->setTextSize(1);
  display->setTextWrap(false);
  display->setTextColor(colYellow);
  display->setCursor(1, 2);
  display->print(line1);
  display->setTextColor(colGreen);
  display->setCursor(1, 14);
  display->print(line2);
}

uint16_t runwayTrailColor(int trail, bool isBlue) {
  if (trail == 0) {
    return isBlue ? colRunwayBlue : colRunwayWhite;
  }
  if (isBlue) {
    if (trail == 1) return display->color565(35, 55, 110);
    if (trail == 2) return display->color565(15, 25, 50);
    return display->color565(5, 8, 18);
  }
  if (trail == 1) return display->color565(80, 80, 70);
  if (trail == 2) return display->color565(35, 35, 30);
  return display->color565(10, 10, 8);
}

void drawRunwayLight(int x, int y, int trail, bool isBlue) {
  if (y < 0 || y >= PANEL_RES_Y) {
    return;
  }
  uint16_t color = runwayTrailColor(trail, isBlue);
  display->drawPixel(x, y, color);
  display->drawPixel(x + 1, y, color);
}

void drawRunwayLights(uint8_t phase) {
  display->fillScreen(colBlack);

  const int leftX = 22;
  const int rightX = 40;
  const int spacing = 4;

  for (int trail = 0; trail < 6; trail++) {
    int y = (phase + trail * spacing) % (PANEL_RES_Y + spacing);
    if (y >= PANEL_RES_Y) {
      continue;
    }
    drawRunwayLight(leftX, y, trail, false);
    drawRunwayLight(rightX, y, trail, true);
  }

  for (int y = 0; y < PANEL_RES_Y; y += 6) {
    display->drawPixel(31, y, display->color565(12, 12, 8));
  }
}

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

  display->setTextSize(1);
  display->setTextWrap(false);
  display->setTextColor(colRadarDim);
  display->setCursor(2, 24);
  display->print("NO TRAFFIC");
}

void initQuietHours() {
  display->setBrightness8(QUIET_BRIGHTNESS);
  quietScene = QUIET_RUNWAY;
  quietSceneStartedMs = millis();
  lastQuietAnimMs = millis();
  runwayPhase = 0;
  radarSweep = 0;
}

void updateQuietHoursDisplay() {
  display->setBrightness8(QUIET_BRIGHTNESS);

  if (millis() - quietSceneStartedMs >= QUIET_SCENE_HOLD_MS) {
    quietSceneStartedMs = millis();
    quietScene = (quietScene == QUIET_RUNWAY) ? QUIET_RADAR : QUIET_RUNWAY;
    runwayPhase = 0;
    radarSweep = 0;
    lastQuietAnimMs = millis();
  }

  unsigned long animInterval =
      (quietScene == QUIET_RUNWAY) ? RUNWAY_ANIM_MS : RADAR_ANIM_MS;
  if (millis() - lastQuietAnimMs < animInterval) {
    return;
  }
  lastQuietAnimMs = millis();

  if (quietScene == QUIET_RUNWAY) {
    drawRunwayLights(runwayPhase++);
  } else {
    radarSweep += 2;
    drawRadarIdle(radarSweep);
  }
}

void showApiError() {
  char detail[12] = "Retrying";
  if (lastApiHttpCode == 429) {
    strncpy(detail, "Rate lim", sizeof(detail) - 1);
  } else if (lastApiHttpCode > 0) {
    snprintf(detail, sizeof(detail), "HTTP %d", lastApiHttpCode);
  } else if (lastApiHttpCode == -1) {
    strncpy(detail, "Parse err", sizeof(detail) - 1);
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

void drawGenericPlaneIcon(int x, int y, uint16_t color) {
  display->drawLine(x + 2, y + 6, x + 13, y + 6, color);
  display->drawLine(x + 7, y + 3, x + 7, y + 9, color);
  display->drawPixel(x + 14, y + 6, color);
  display->drawPixel(x + 1, y + 6, color);
}

void drawAirlineSymbol(const char *callsign) {
  const int x = 1;
  const int y = 1;
  const int w = 17;
  const int h = 14;

  char prefix[4];
  uppercasePrefix(prefix, callsign, sizeof(prefix));

  if (strncmp(prefix, "DAL", 3) == 0) {
    uint16_t blue = display->color565(0, 30, 150);
    display->fillRect(x, y, w, h, blue);
    display->fillTriangle(x + 9, y + 2, x + 3, y + h - 2, x + w - 3, y + h - 2, colRed);
    return;
  }
  if (strncmp(prefix, "CMP", 3) == 0) {
    uint16_t blue = display->color565(0, 70, 180);
    display->fillRect(x, y, w, h, blue);
    display->drawFastHLine(x + 2, y + 7, w - 4, colWhite);
    return;
  }
  if (strncmp(prefix, "JBU", 3) == 0) {
    uint16_t blue = display->color565(0, 80, 200);
    display->fillRect(x, y, w, h, blue);
    display->fillRect(x + 3, y + 3, 3, 3, colWhite);
    display->fillRect(x + 10, y + 3, 3, 3, colWhite);
    return;
  }
  if (strncmp(prefix, "SWA", 3) == 0) {
    display->fillRect(x, y, w, h, colYellow);
    display->drawFastHLine(x + 2, y + 5, w - 4, colRed);
    display->drawFastHLine(x + 2, y + 9, w - 4, display->color565(0, 100, 200));
    return;
  }
  if (strncmp(prefix, "UAL", 3) == 0) {
    display->fillRect(x, y, w, h, display->color565(0, 50, 160));
    display->drawLine(x + 3, y + 3, x + w - 3, y + h - 3, colWhite);
    return;
  }
  if (strncmp(prefix, "AAL", 3) == 0) {
    display->fillRect(x, y, w, h, colWhite);
    display->drawFastHLine(x + 2, y + 4, w - 4, colRed);
    display->drawFastHLine(x + 2, y + 7, w - 4, display->color565(0, 50, 150));
    display->drawFastHLine(x + 2, y + 10, w - 4, colRed);
    return;
  }
  if (strncmp(prefix, "FFT", 3) == 0) {
    display->fillRect(x, y, w, h, display->color565(0, 120, 90));
    display->fillRect(x + 4, y + 4, 9, 6, colWhite);
    return;
  }
  if (strncmp(prefix, "NKS", 3) == 0) {
    display->fillRect(x, y, w, h, colYellow);
    display->fillRect(x + 4, y + 4, 9, 6, colBlack);
    return;
  }

  display->fillRect(x, y, w, h, display->color565(25, 25, 25));
  if (strlen(prefix) >= 3) {
    display->setTextSize(1);
    display->setTextColor(colYellow);
    display->setCursor(x + 1, y + 4);
    display->print(prefix);
  } else {
    drawGenericPlaneIcon(x, y, colGreen);
  }
}

const char *bearingLabel(float bearing) {
  static const char *labels[] = {
      "N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
      "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"};
  int idx = (int)((bearing + 11.25f) / 22.5f) & 15;
  return labels[idx];
}

void formatRouteLine(char *buf, size_t buflen) {
  if (!hasRoute) {
    snprintf(buf, buflen, "---?---");
    return;
  }
  snprintf(buf, buflen, "%s>%s", routeOrigin, routeDest);
}

void formatPositionLine(char *buf, size_t buflen) {
  if (nearestCallsign == "NONE") {
    snprintf(buf, buflen, "NO TRAFFIC");
    return;
  }
  if (nearestDistanceKm < 10.0f) {
    snprintf(buf, buflen, "%s %.1fk", bearingLabel(nearestBearing), nearestDistanceKm);
    return;
  }
  snprintf(buf, buflen, "%s %dk", bearingLabel(nearestBearing),
           (int)(nearestDistanceKm + 0.5f));
}

void showFlight() {
  char routeLine[12];
  char positionLine[12];

  display->setBrightness8(PANEL_BRIGHTNESS);
  display->fillScreen(colBlack);

  drawAirlineSymbol(nearestCallsign.c_str());

  display->setTextSize(1);
  display->setTextWrap(false);
  display->setTextColor(colGreen);
  display->setCursor(20, 2);
  display->print(nearestCallsign);

  display->setTextColor(colYellow);
  display->setCursor(20, 10);
  display->print(nearestAircraftType);

  display->drawFastHLine(0, 16, PANEL_RES_X, display->color565(30, 30, 30));

  formatRouteLine(routeLine, sizeof(routeLine));
  display->setTextColor(colYellow);
  display->setCursor(1, 18);
  display->print(routeLine);

  formatPositionLine(positionLine, sizeof(positionLine));
  display->setTextColor(colGreen);
  display->setCursor(1, 26);
  display->print(positionLine);
}

bool fetchRoute(const String &callsign) {
  if (callsign == "NONE" || callsign.length() < 3) {
    strncpy(routeOrigin, "---", sizeof(routeOrigin));
    strncpy(routeDest, "---", sizeof(routeDest));
    hasRoute = false;
    cachedRouteCallsign = "";
    return false;
  }

  if (callsign == cachedRouteCallsign) {
    return hasRoute;
  }

  strncpy(routeOrigin, "---", sizeof(routeOrigin));
  strncpy(routeDest, "---", sizeof(routeDest));
  hasRoute = false;

  JsonDocument req;
  JsonArray planes = req["planes"].to<JsonArray>();
  JsonObject plane = planes.add<JsonObject>();
  plane["callsign"] = callsign;

  String body;
  serializeJson(req, body);

  HTTPClient http;
  http.begin("https://flightroutes.opensky-network.org/api/routeset");
  http.addHeader("Content-Type", "application/json");
  http.setTimeout(8000);

  int statusCode = http.POST(body);
  if (statusCode != HTTP_CODE_OK && statusCode != HTTP_CODE_CREATED) {
    Serial.printf("Route request failed: %d\n", statusCode);
    http.end();
    cachedRouteCallsign = callsign;
    return false;
  }

  String payload = http.getString();
  http.end();

  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, payload);
  if (error || !doc.is<JsonArray>() || doc.size() == 0) {
    Serial.printf("Route parse failed: %s\n", error.c_str());
    cachedRouteCallsign = callsign;
    return false;
  }

  const char *iata = doc[0]["_airport_codes_iata"] | "";
  if (strlen(iata) < 7 || strcmp(iata, "unknown") == 0) {
    cachedRouteCallsign = callsign;
    return false;
  }

  strncpy(routeOrigin, iata, 3);
  routeOrigin[3] = '\0';
  strncpy(routeDest, iata + 4, 3);
  routeDest[3] = '\0';
  hasRoute = true;
  cachedRouteCallsign = callsign;
  return true;
}

bool fetchNearestFlight() {
  if (WiFi.status() != WL_CONNECTED) {
    return false;
  }

  String url = "https://api.adsb.lol/v2/closest/";
  url += String(RECEIVER_LAT, 6);
  url += "/";
  url += String(RECEIVER_LON, 6);
  url += "/";
  url += String(SEARCH_RADIUS_KM);

  HTTPClient http;
  http.begin(url);
  http.setTimeout(10000);

  int statusCode = http.GET();
  if (statusCode != HTTP_CODE_OK) {
    lastApiHttpCode = statusCode;
    Serial.printf("API request failed: %d\n", statusCode);
    http.end();
    return false;
  }

  String payload = http.getString();
  http.end();

  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, payload);
  if (error) {
    lastApiHttpCode = -1;
    Serial.printf("JSON parse failed: %s\n", error.c_str());
    return false;
  }

  lastApiHttpCode = 0;

  JsonArray aircraft = doc["ac"].as<JsonArray>();
  if (aircraft.isNull() || aircraft.size() == 0) {
    nearestCallsign = "NONE";
    nearestAircraftType = "----";
    nearestDistanceKm = 0.0;
    nearestBearing = 0.0;
    cachedRouteCallsign = "";
    hasRoute = false;
    return true;
  }

  JsonObject plane = aircraft[0];
  nearestCallsign = plane["flight"].as<const char *>();
  nearestCallsign.trim();
  if (nearestCallsign.length() == 0) {
    nearestCallsign = plane["r"].as<const char *>();
  }
  if (nearestCallsign.length() == 0) {
    nearestCallsign = plane["hex"].as<const char *>();
  }

  nearestAircraftType = plane["t"] | "----";
  nearestAircraftType.trim();
  if (nearestAircraftType.length() == 0) {
    nearestAircraftType = "----";
  }

  nearestDistanceKm = plane["dst"] | 0.0;
  nearestBearing = plane["dir"] | 0.0;

  fetchRoute(nearestCallsign);
  return true;
}

bool isQuietHours() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    return false;
  }
  int hour = timeinfo.tm_hour;
  return hour >= QUIET_HOUR_START && hour < QUIET_HOUR_END;
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

  setupDisplay();
  showStatus("ESP32", "Flight Radar");

  if (!connectWiFi(30000)) {
    return;
  }

  syncTime();

  if (isQuietHours()) {
    initQuietHours();
    updateQuietHoursDisplay();
    wasQuietHours = true;
    return;
  }

  if (fetchNearestFlight()) {
    showFlight();
    lastFetchMs = millis();
  } else {
    showApiError();
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

  bool quiet = isQuietHours();

  if (quiet) {
    if (!wasQuietHours) {
      initQuietHours();
    }
    wasQuietHours = true;
    updateQuietHoursDisplay();
    delay(40);
    return;
  }

  if (wasQuietHours) {
    display->setBrightness8(PANEL_BRIGHTNESS);
    wasQuietHours = false;
    lastFetchMs = 0;
  }

  if (millis() - lastFetchMs >= REFRESH_INTERVAL_MS) {
    if (fetchNearestFlight()) {
      showFlight();
    } else {
      showApiError();
    }
    lastFetchMs = millis();
  }
}
