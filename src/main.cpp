#include <WiFi.h>
#include <Arduino.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <ESP32-HUB75-MatrixPanel-I2S-DMA.h>
#include <ESPmDNS.h>
#include <LittleFS.h>
#include <WebServer.h>
#include <WebSocketsClient.h>
#include <esp_system.h>
#include <time.h>
#include <math.h>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <strings.h>
#include <vector>

#include "app_config.h"
#include "config.h"
#include "mqtt_ca_cert.h"

enum LayoutSlotKind { LAYOUT_TEXT, LAYOUT_COMPASS };
enum OperatingMode { MODE_OVERHEAD_FLIGHTS, MODE_FLIGHT_PLAN };
enum FlightPlanSource { FLIGHT_PLAN_FROM_PORTAL, FLIGHT_PLAN_FROM_MQTT };
enum DisplayNotice { DISPLAY_NOTICE_NONE, DISPLAY_NOTICE_RECEIVED_MQTT,
                     DISPLAY_NOTICE_RECEIVED_PORTAL,
                     DISPLAY_NOTICE_FLIGHTS_COMPLETE };

struct LayoutSlot {
  LayoutSlotKind kind;
  const char *text;
  uint16_t color;
};

MatrixPanel_I2S_DMA *display = nullptr;
WebServer configServer(80);
WebSocketsClient mqttWebSocket;
AppConfig pendingConfig;
bool configWifiChangePending = false;
bool configWifiChangeStartPending = false;
bool configApplySucceeded = false;
bool configSuccessScreenActive = false;
bool configSuccessIpScreenActive = false;
bool mdnsStarted = false;
bool configAccessPointActive = false;
bool mqttClientStarted = false;
bool mqttRestartPending = false;
bool mqttConnected = false;
bool mqttSubscribed = false;
String configApplyMessage;
unsigned long configWifiChangeStartedMs = 0;
unsigned long configSuccessScreenStartedMs = 0;
unsigned long configAccessPointShutdownAtMs = 0;
const unsigned long CONFIG_WIFI_CONNECT_TIMEOUT_MS = 45000;
const unsigned long CONFIG_SUCCESS_SSID_SCREEN_MS = 1500;
const unsigned long CONFIG_SUCCESS_IP_SCREEN_MS = 2000;
const unsigned long CONFIG_AP_SHUTDOWN_DELAY_MS = 30000;
const int API_ERROR_FAIL_THRESHOLD = 12;
const unsigned long NTP_RETRY_INTERVAL_MS = 5UL * 60UL * 1000UL;
const uint32_t NTP_INITIAL_WAIT_MS = 1000;
const uint32_t NTP_RETRY_WAIT_MS = 250;
const time_t VALID_SYSTEM_TIME_EPOCH = 1577836800;
const size_t MQTT_MAX_PAYLOAD_SIZE = 4096;
const unsigned long MQTT_KEEPALIVE_INTERVAL_MS = 30000;
const uint16_t MQTT_SUBSCRIBE_PACKET_ID = 1;
const unsigned long DISPLAY_NOTICE_DURATION_MS = 5000;
unsigned long lastMqttKeepAliveMs = 0;
unsigned long lastMqttStartAttemptMs = 0;
DisplayNotice displayNotice = DISPLAY_NOTICE_NONE;
unsigned long displayNoticeStartedMs = 0;
unsigned long displayNoticeLastRenderMs = 0;
bool displayNoticeHasRendered = false;

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
unsigned long lastNtpAttemptMs = 0;
bool ntpAttemptMade = false;
bool ntpTimeReceived = false;
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

const int FLIGHT_FETCH_RETRIES = 3;
const int ROUTE_FETCH_RETRIES = 3;
const int MAX_TRACKED_AIRCRAFT = 8;
const unsigned long ROUTE_FETCH_DELAY_MS = 250;
const unsigned long FETCH_RETRY_ON_FAIL_MS = 10000;
const unsigned long FLIGHT_PLAN_POLL_INTERVAL_MS = 15000;
const unsigned long AEROAPI_RETRY_INTERVAL_MS = 300000;
const unsigned long FLIGHT_PLAN_ETA_GRACE_SECONDS = 1800;
const unsigned long FLIGHT_PLAN_LANDED_HOLD_MS = 5UL * 60 * 1000;
const unsigned long FLIGHT_PLAN_ROUTE_RETRY_MS = 300000;
const unsigned long FLIGHT_PLAN_DISPLAY_PHASE_MS = 5000;
const unsigned long FLIGHT_PLAN_SCROLL_INTERVAL_MS = 350;
const unsigned long FLIGHT_PLAN_LOCATION_LOOKUP_MS = 60000;

struct AircraftSnapshot {
  char callsign[12];
  char type[8];
  float latitude;
  float longitude;
  bool hasPosition;
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
OperatingMode operatingMode = MODE_OVERHEAD_FLIGHTS;
String flightPlanJson;
uint8_t flightPlanCount = 0;
uint32_t flightPlanStartValidity = 0;
uint32_t flightPlanEndValidity = 0;
uint8_t flightPlanCurrentIndex = 0;
bool flightPlanCompleted = false;
bool flightPlanLegObservedAirborne = false;
bool flightPlanHasLiveData = false;
bool flightPlanAeroApiLanded = false;
bool flightPlanAdsbLanded = false;
bool flightPlanLandingHoldActive = false;
unsigned long flightPlanLandingHoldStartedMs = 0;
bool flightPlanTrackingCallsignResolved = false;
bool flightPlanRouteLookupAttempted = false;
unsigned long flightPlanLastRouteLookupMs = 0;
uint32_t flightPlanEta = 0;
uint32_t flightPlanEtd = 0;
uint32_t flightPlanScheduledEta = 0;
uint32_t flightPlanEstimatedEta = 0;
String flightPlanOrigin;
String flightPlanDestination;
String flightPlanTrackingCallsign;
String flightPlanLocation = "LOCATION UNKNOWN";
String flightPlanAeroStatus;
AircraftSnapshot flightPlanAircraft;
unsigned long flightPlanLastPollMs = 0;
unsigned long aeroApiLastPollMs = 0;
unsigned long flightPlanLastLocationLookupMs = 0;
unsigned long flightPlanDisplayCycleStartedMs = 0;
unsigned long flightPlanLastBottomRenderMs = 0;
int flightPlanLastPositionPhase = -1;
int flightPlanLastStatsPhase = -1;
bool flightPlanDisplayDirty = true;
bool showCallsignInTypeSlot = false;
bool callsignTypeSwapPending = false;
unsigned long typeSlotStartedMs = 0;

void refreshFlightDisplay();
void showFlight();
uint8_t activeBrightness();
void clearRoute();
void resetAircraftDisplayTimer();
void updateAircraftTypeSlotIfNeeded();
void copyStringField(char *dest, size_t destSize, const char *src);
void setupConfigServer();
void updateConfigWifiChange();
bool startConfigAccessPoint();
void updateConfigAccessPoint();
void startFlightMdns();
bool beginTimeSync(uint32_t waitMs);
void updateTimeSync();
void handlePostFlightPlan();
void handleGetFlightPlan();
void startMqttClient();
void updateMqttClient();
void updateOperatingModeFromFlightPlan();
void drawFlightPlanScrollingRow(int y, const String &text, uint16_t color);
bool updateDisplayNotice();
void drawFlightPlanDisplay();
void updateFlightPlanDisplayAlternation();
bool fetchFlightPlanLocation(float latitude, float longitude);
void pollFlightPlan();
bool fetchAeroApiFlightDetails(const String &flightNumber);
bool fetchFlightPlanAircraft(const String &flightNumber);
bool advanceFlightPlan();
bool fetchRoute(const String &callsign);

void setOperatingMode(OperatingMode mode) {
  if (operatingMode == MODE_FLIGHT_PLAN &&
      mode == MODE_OVERHEAD_FLIGHTS) {
    displayNotice = DISPLAY_NOTICE_FLIGHTS_COMPLETE;
    displayNoticeHasRendered = false;
  }
  operatingMode = mode;
  digitalWrite(OVERHEAD_MODE_LED_PIN,
               mode == MODE_OVERHEAD_FLIGHTS ? HIGH : LOW);
  digitalWrite(FLIGHT_PLAN_MODE_LED_PIN,
               mode == MODE_FLIGHT_PLAN ? HIGH : LOW);
  Serial.printf("Operating mode: %s\n",
                mode == MODE_OVERHEAD_FLIGHTS ? "overhead flights"
                                               : "flight plan");
}

void updateOperatingModeFromFlightPlan() {
  OperatingMode desiredMode = MODE_OVERHEAD_FLIGHTS;
  time_t now = time(nullptr);
  bool landingHoldActive =
      flightPlanLandingHoldActive &&
      millis() - flightPlanLandingHoldStartedMs <
          FLIGHT_PLAN_LANDED_HOLD_MS;
  if (landingHoldActive ||
      (flightPlanCount > 0 && !flightPlanCompleted &&
       now >= VALID_SYSTEM_TIME_EPOCH &&
       static_cast<uint64_t>(now) >= flightPlanStartValidity &&
       static_cast<uint64_t>(now) < flightPlanEndValidity)) {
    desiredMode = MODE_FLIGHT_PLAN;
  }

  if (desiredMode != operatingMode) {
    setOperatingMode(desiredMode);
  }
}

void setupModeIndicators() {
  pinMode(OVERHEAD_MODE_LED_PIN, OUTPUT);
  pinMode(FLIGHT_PLAN_MODE_LED_PIN, OUTPUT);
  setOperatingMode(MODE_OVERHEAD_FLIGHTS);
}

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

  const int maxChars = (PANEL_RES_X - 2) / TEXT_CHAR_W;
  if (strlen(line2) <= (size_t)maxChars) {
    printClipped(1, 14, line2, colTextPrimary);
    return;
  }

  char firstPart[40];
  snprintf(firstPart, sizeof(firstPart), "%.*s", maxChars, line2);
  printClipped(1, 14, firstPart, colTextPrimary);
  printClipped(1, 22, line2 + maxChars, colTextPrimary);
}

void drawDisplayNotice() {
  display->setBrightness8(activeBrightness());
  display->fillScreen(colBlack);

  const char *firstLine = "FLIGHTS";
  const char *secondLine = "";
  const char *thirdLine = "";
  const char *fourthLine = "";
  if (displayNotice == DISPLAY_NOTICE_RECEIVED_MQTT) {
    secondLine = "RECEIVED";
    thirdLine = "FOR";
    fourthLine = config.mqttDataName.c_str();
  } else if (displayNotice == DISPLAY_NOTICE_RECEIVED_PORTAL) {
    secondLine = "RECEIVED";
    thirdLine = "FROM";
    fourthLine = "PORTAL";
  } else if (displayNotice == DISPLAY_NOTICE_FLIGHTS_COMPLETE) {
    secondLine = "COMPLETE.";
    thirdLine = "SWITCHING";
    fourthLine = "OVERHEAD";
  }

  printClippedCentered(0, PANEL_RES_X, 0, firstLine, colTextSecondary);
  printClippedCentered(0, PANEL_RES_X, 8, secondLine, colTextPrimary);
  printClippedCentered(0, PANEL_RES_X, 16, thirdLine, colTextSecondary);
  if (displayNotice == DISPLAY_NOTICE_RECEIVED_MQTT) {
    drawFlightPlanScrollingRow(24, fourthLine, colTextPrimary);
  } else {
    printClippedCentered(0, PANEL_RES_X, 24, fourthLine, colTextPrimary);
  }
}

bool updateDisplayNotice() {
  if (displayNotice == DISPLAY_NOTICE_NONE) {
    return false;
  }

  const unsigned long now = millis();
  if (!displayNoticeHasRendered) {
    displayNoticeStartedMs = now;
    displayNoticeLastRenderMs = now;
    displayNoticeHasRendered = true;
    drawDisplayNotice();
    return true;
  }
  if (now - displayNoticeStartedMs >= DISPLAY_NOTICE_DURATION_MS) {
    displayNotice = DISPLAY_NOTICE_NONE;
    displayNoticeHasRendered = false;
    if (operatingMode == MODE_FLIGHT_PLAN) {
      flightPlanDisplayDirty = true;
    } else if (trackedAircraftCount > 0) {
      showFlight();
    } else {
      screensaverActive = false;
    }
    return false;
  }

  if (displayNotice == DISPLAY_NOTICE_RECEIVED_MQTT &&
      config.mqttDataName.length() > PANEL_RES_X / TEXT_CHAR_W &&
      now - displayNoticeLastRenderMs >= FLIGHT_PLAN_SCROLL_INTERVAL_MS) {
    displayNoticeLastRenderMs = now;
    drawDisplayNotice();
  }
  return true;
}

void sendJsonResponse(int statusCode, JsonDocument &response) {
  String body;
  serializeJson(response, body);
  configServer.sendHeader("Cache-Control", "no-store");
  configServer.send(statusCode, "application/json", body);
}

bool readConfigString(JsonObjectConst object, const char *key, String &value,
                      size_t maxLength, bool allowEmpty = false) {
  JsonVariantConst field = object[key];
  if (!field.is<const char *>()) {
    return false;
  }

  value = field.as<const char *>();
  return value.length() <= maxLength && (allowEmpty || value.length() > 0);
}

bool hasJsonField(JsonObjectConst object, const char *key) {
  for (JsonPairConst field : object) {
    if (strcmp(field.key().c_str(), key) == 0) {
      return true;
    }
  }
  return false;
}

bool isValidMqttWebSocketPath(const String &path) {
  if (!path.startsWith("/")) {
    return false;
  }
  for (size_t index = 0; index < path.length(); index++) {
    const uint8_t character = static_cast<uint8_t>(path[index]);
    if (character <= 0x20 || character == 0x7F) {
      return false;
    }
  }
  return true;
}

bool readConfigNumber(JsonObjectConst object, const char *key, double &value) {
  JsonVariantConst field = object[key];
  if (!field.is<int>() && !field.is<double>()) {
    return false;
  }

  value = field.as<double>();
  return isfinite(value);
}

bool configurationChanged(const AppConfig &left, const AppConfig &right) {
  const double numericTolerance = 0.0000001;
  return left.wifiSSID != right.wifiSSID ||
         left.wifiPassword != right.wifiPassword ||
         left.aeroApiKey != right.aeroApiKey ||
         left.aeroApiURL != right.aeroApiURL ||
         left.mqttIP != right.mqttIP || left.mqttPort != right.mqttPort ||
         left.mqttWebSocketPath != right.mqttWebSocketPath ||
         left.mqttTopic != right.mqttTopic ||
         left.mqttDataName != right.mqttDataName ||
         left.mqttUsername != right.mqttUsername ||
         left.mqttPassword != right.mqttPassword ||
         fabs(left.latitude - right.latitude) > numericTolerance ||
         fabs(left.longitude - right.longitude) > numericTolerance ||
         fabs(left.searchRadius - right.searchRadius) > numericTolerance;
}

void logConfigurationChanges(const AppConfig &previous,
                             const AppConfig &updated) {
  const double numericTolerance = 0.0000001;

  if (previous.wifiSSID != updated.wifiSSID) {
    Serial.printf("Configuration changed: Wi-Fi SSID=%s\n", updated.wifiSSID.c_str());
  }
  if (previous.wifiPassword != updated.wifiPassword) {
    Serial.println("Configuration changed: Wi-Fi password");
  }
  if (previous.aeroApiKey != updated.aeroApiKey) {
    Serial.println("Configuration changed: AeroAPI key");
  }
  if (previous.aeroApiURL != updated.aeroApiURL) {
    Serial.printf("Configuration changed: AeroAPI URL=%s\n",
                  updated.aeroApiURL.c_str());
  }
  if (previous.mqttIP != updated.mqttIP) {
    Serial.printf("Configuration changed: MQTT address=%s\n",
                  updated.mqttIP.c_str());
  }
  if (previous.mqttPort != updated.mqttPort) {
    Serial.printf("Configuration changed: MQTT port=%u\n",
                  updated.mqttPort);
  }
  if (previous.mqttWebSocketPath != updated.mqttWebSocketPath) {
    Serial.printf("Configuration changed: MQTT WebSocket path=%s\n",
                  updated.mqttWebSocketPath.c_str());
  }
  if (previous.mqttTopic != updated.mqttTopic) {
    Serial.printf("Configuration changed: MQTT topic=%s\n",
                  updated.mqttTopic.c_str());
  }
  if (previous.mqttDataName != updated.mqttDataName) {
    Serial.printf("Configuration changed: MQTT data name=%s\n",
                  updated.mqttDataName.c_str());
  }
  if (previous.mqttUsername != updated.mqttUsername) {
    Serial.println("Configuration changed: MQTT username");
  }
  if (previous.mqttPassword != updated.mqttPassword) {
    Serial.println("Configuration changed: MQTT password");
  }
  if (fabs(previous.latitude - updated.latitude) > numericTolerance) {
    Serial.printf("Configuration changed: latitude=%.6f\n", updated.latitude);
  }
  if (fabs(previous.longitude - updated.longitude) > numericTolerance) {
    Serial.printf("Configuration changed: longitude=%.6f\n",
                  updated.longitude);
  }
  if (fabs(previous.searchRadius - updated.searchRadius) > numericTolerance) {
    Serial.printf("Configuration changed: search radius=%.2f NM\n",
                  updated.searchRadius);
  }
}

void handleGetConfiguration() {
  JsonDocument response;
  response["wifiSSID"] = config.wifiSSID;
  response["aeroApiURL"] = config.aeroApiURL;
  response["mqttIP"] = config.mqttIP;
  response["mqttPort"] = config.mqttPort;
  response["mqttWebSocketPath"] = config.mqttWebSocketPath;
  response["mqttTopic"] = config.mqttTopic;
  response["mqttDataName"] = config.mqttDataName;
  response["mqttUsername"] = config.mqttUsername;
  response["mqttPasswordConfigured"] = !config.mqttPassword.isEmpty();
  response["latitude"] = config.latitude;
  response["longitude"] = config.longitude;
  response["searchRadius"] = config.searchRadius;
  sendJsonResponse(200, response);
}

void handleGetConfigurationStatus() {
  JsonDocument response;
  response["pending"] = configWifiChangePending;
  response["success"] = configApplySucceeded;
  response["message"] = configApplyMessage;
  sendJsonResponse(200, response);
}

bool readUnixTimestamp(JsonVariantConst field, uint32_t &timestamp) {
  if (field.is<uint32_t>()) {
    timestamp = field.as<uint32_t>();
    return true;
  }

  if (!field.is<const char *>()) {
    return false;
  }

  const char *text = field.as<const char *>();
  if (text == nullptr || text[0] == '\0') {
    return false;
  }
  for (const char *digit = text; *digit != '\0'; digit++) {
    if (*digit < '0' || *digit > '9') {
      return false;
    }
  }

  char *end = nullptr;
  errno = 0;
  unsigned long long parsed = strtoull(text, &end, 10);
  if (errno == ERANGE || end == text || *end != '\0' ||
      parsed > UINT32_MAX) {
    return false;
  }

  timestamp = static_cast<uint32_t>(parsed);
  return true;
}

bool readOptionalFlightTimestamp(JsonVariantConst field, bool &hasTimestamp,
                                 uint32_t &timestamp) {
  hasTimestamp = false;
  if (field.is<const char *>()) {
    const char *text = field.as<const char *>();
    if (text != nullptr && text[0] == '\0') {
      return true;
    }
  }

  if (!readUnixTimestamp(field, timestamp)) {
    return false;
  }
  hasTimestamp = true;
  return true;
}

void handleGetFlightPlan() {
  if (flightPlanJson.isEmpty()) {
    configServer.sendHeader("Cache-Control", "no-store");
    configServer.send(200, "application/json",
                      "{\"flight_data\":{},\"validity\":{\"start_validity\":\"0\",\"end_validity\":\"0\"}}");
    return;
  }

  configServer.sendHeader("Cache-Control", "no-store");
  configServer.send(200, "application/json", flightPlanJson);
}

bool flightPlansEqual(const String &candidateJson) {
  if (flightPlanJson.isEmpty()) {
    JsonDocument candidate;
    DeserializationError error = deserializeJson(candidate, candidateJson);
    if (error) {
      Serial.printf("Stored flight plan comparison failed: %s\n",
                    error.c_str());
      return false;
    }
    return candidate["flight_data"].as<JsonObjectConst>().size() == 0;
  }

  JsonDocument existing;
  JsonDocument candidate;
  DeserializationError existingError =
      deserializeJson(existing, flightPlanJson);
  if (existingError) {
    Serial.printf("Existing flight plan comparison failed: %s\n",
                  existingError.c_str());
    return false;
  }
  DeserializationError candidateError =
      deserializeJson(candidate, candidateJson);
  if (candidateError) {
    Serial.printf("New flight plan comparison failed: %s\n",
                  candidateError.c_str());
    return false;
  }

  uint32_t existingStart = 0;
  uint32_t existingEnd = 0;
  uint32_t candidateStart = 0;
  uint32_t candidateEnd = 0;
  if (!readUnixTimestamp(existing["validity"]["start_validity"],
                         existingStart) ||
      !readUnixTimestamp(existing["validity"]["end_validity"], existingEnd) ||
      !readUnixTimestamp(candidate["validity"]["start_validity"],
                         candidateStart) ||
      !readUnixTimestamp(candidate["validity"]["end_validity"],
                         candidateEnd) ||
      existingStart != candidateStart || existingEnd != candidateEnd) {
    return false;
  }

  JsonObjectConst existingFlights =
      existing["flight_data"].as<JsonObjectConst>();
  JsonObjectConst candidateFlights =
      candidate["flight_data"].as<JsonObjectConst>();
  if (existingFlights.size() != candidateFlights.size()) {
    return false;
  }

  for (uint8_t index = 1; index <= 4; index++) {
    char key[] = {static_cast<char>('0' + index), '\0'};
    JsonObjectConst existingFlight =
        existingFlights[key].as<JsonObjectConst>();
    JsonObjectConst candidateFlight =
        candidateFlights[key].as<JsonObjectConst>();
    if (existingFlight.isNull() != candidateFlight.isNull()) {
      return false;
    }
    if (existingFlight.isNull()) {
      continue;
    }
    if (strcmp(existingFlight["destination"] | "",
               candidateFlight["destination"] | "") != 0 ||
        strcmp(existingFlight["flight"] | "",
               candidateFlight["flight"] | "") != 0 ||
        strcmp(existingFlight["origin"] | "",
               candidateFlight["origin"] | "") != 0) {
      return false;
    }

    bool existingHasEta = false;
    bool candidateHasEta = false;
    uint32_t existingEta = 0;
    uint32_t candidateEta = 0;
    bool existingHasEtd = false;
    bool candidateHasEtd = false;
    uint32_t existingEtd = 0;
    uint32_t candidateEtd = 0;
    if (!readOptionalFlightTimestamp(existingFlight["eta"], existingHasEta,
                                     existingEta) ||
        !readOptionalFlightTimestamp(candidateFlight["eta"], candidateHasEta,
                                     candidateEta) ||
        !readOptionalFlightTimestamp(existingFlight["etd"], existingHasEtd,
                                     existingEtd) ||
        !readOptionalFlightTimestamp(candidateFlight["etd"], candidateHasEtd,
                                     candidateEtd) ||
        existingHasEta != candidateHasEta ||
        (existingHasEta && existingEta != candidateEta) ||
        existingHasEtd != candidateHasEtd ||
        (existingHasEtd && existingEtd != candidateEtd)) {
      return false;
    }
  }
  return true;
}

int storeFlightPlan(JsonDocument &request, JsonDocument &response,
                    FlightPlanSource source) {
  auto reject = [&response](int statusCode, const char *message) {
    response["success"] = false;
    response["message"] = message;
    return statusCode;
  };
  if (!request.is<JsonObject>()) {
    return reject(400, "Invalid flight plan JSON.");
  }
  JsonObjectConst fields = request.as<JsonObjectConst>();
  JsonVariantConst flightDataValue = fields["flight_data"];
  JsonVariantConst validityValue = fields["validity"];
  if (!flightDataValue.is<JsonObjectConst>() ||
      !validityValue.is<JsonObjectConst>()) {
    return reject(400, "Flight data and validity fields are required.");
  }

  JsonObjectConst flightData = flightDataValue.as<JsonObjectConst>();
  JsonObjectConst validity = validityValue.as<JsonObjectConst>();
  uint32_t startValidity = 0;
  uint32_t endValidity = 0;
  if (flightData.size() > 4 ||
      !readUnixTimestamp(validity["start_validity"], startValidity) ||
      !readUnixTimestamp(validity["end_validity"], endValidity)) {
    return reject(400, "Flight plan must have at most four flights and valid timestamps.");
  }

  JsonDocument storedPlan;
  JsonObject storedFlights = storedPlan["flight_data"].to<JsonObject>();
  uint8_t storedFlightCount = 0;
  uint8_t flightEntryMask = 0;
  for (JsonPairConst entry : flightData) {
    const char *key = entry.key().c_str();
    uint8_t entryIndex = 0;
    if (strcmp(key, "1") == 0) {
      entryIndex = 0;
    } else if (strcmp(key, "2") == 0) {
      entryIndex = 1;
    } else if (strcmp(key, "3") == 0) {
      entryIndex = 2;
    } else if (strcmp(key, "4") == 0) {
      entryIndex = 3;
    } else {
      return reject(400, "Flight entries must use keys 1 through 4.");
    }
    flightEntryMask |= static_cast<uint8_t>(1U << entryIndex);

    JsonObjectConst flight = entry.value().as<JsonObjectConst>();
    String destination;
    String flightNumber;
    String origin;
    bool hasEta = false;
    bool hasEtd = false;
    uint32_t eta = 0;
    uint32_t etd = 0;
    if (!readOptionalFlightTimestamp(flight["eta"], hasEta, eta) ||
        !readConfigString(flight, "destination", destination, 32, true) ||
        !readConfigString(flight, "flight", flightNumber, 24) ||
        !readConfigString(flight, "origin", origin, 32, true) ||
        !readOptionalFlightTimestamp(flight["etd"], hasEtd, etd)) {
      return reject(400, "Each flight entry must include valid flight fields.");
    }

    flightNumber.trim();
    if (flightNumber.isEmpty() || flightNumber.length() > 24) {
      return reject(400, "Flight numbers must contain 1 to 24 characters.");
    }
    for (size_t i = 0; i < flightNumber.length(); i++) {
      if (static_cast<uint8_t>(flightNumber[i]) < 0x20) {
        return reject(400, "Flight numbers cannot contain control characters.");
      }
    }

    JsonObject storedFlight = storedFlights[key].to<JsonObject>();
    if (hasEta) {
      storedFlight["eta"] = eta;
    } else {
      storedFlight["eta"] = "";
    }
    storedFlight["destination"] = destination;
    storedFlight["flight"] = flightNumber;
    storedFlight["origin"] = origin;
    if (hasEtd) {
      storedFlight["etd"] = etd;
    } else {
      storedFlight["etd"] = "";
    }
    storedFlightCount++;
  }

  if (flightEntryMask != static_cast<uint8_t>((1U << storedFlightCount) - 1U)) {
    return reject(400, "Flight entry keys must be consecutive starting at 1.");
  }

  if (storedFlightCount > 0 &&
      (startValidity == 0 || endValidity <= startValidity)) {
    return reject(400, "Flight plan validity end must be later than its start.");
  }
  if (storedFlightCount == 0 && (startValidity != 0 || endValidity != 0)) {
    return reject(400, "An empty flight plan must use zero validity timestamps.");
  }

  JsonObject storedValidity = storedPlan["validity"].to<JsonObject>();
  storedValidity["start_validity"] = String(startValidity);
  storedValidity["end_validity"] = String(endValidity);

  String serialized;
  serializeJson(storedPlan, serialized);
  if (serialized.isEmpty()) {
    return reject(500, "Could not store flight plan in memory.");
  }

  if (flightPlansEqual(serialized)) {
    Serial.println("Received flight plan is unchanged; keeping current tracking state");
    response["success"] = true;
    response["storedIn"] = "RAM";
    response["flightCount"] = flightPlanCount;
    return 200;
  }

  flightPlanJson = storedFlightCount == 0 ? "" : serialized;
  flightPlanCount = storedFlightCount;
  flightPlanStartValidity = startValidity;
  flightPlanEndValidity = endValidity;
  flightPlanCurrentIndex = 0;
  flightPlanCompleted = false;
  flightPlanLegObservedAirborne = false;
  flightPlanHasLiveData = false;
  flightPlanAeroApiLanded = false;
  flightPlanAdsbLanded = false;
  flightPlanLandingHoldActive = false;
  flightPlanLandingHoldStartedMs = 0;
  flightPlanRouteLookupAttempted = false;
  flightPlanLastRouteLookupMs = 0;
  flightPlanEta = 0;
  flightPlanEtd = 0;
  flightPlanScheduledEta = 0;
  flightPlanEstimatedEta = 0;
  flightPlanOrigin = "";
  flightPlanDestination = "";
  flightPlanTrackingCallsign = "";
  flightPlanTrackingCallsignResolved = false;
  flightPlanLocation = "LOCATION UNKNOWN";
  flightPlanAeroStatus = "";
  flightPlanLastPollMs = 0;
  aeroApiLastPollMs = 0;
  flightPlanLastLocationLookupMs = 0;
  flightPlanDisplayCycleStartedMs = millis();
  flightPlanLastBottomRenderMs = 0;
  flightPlanLastPositionPhase = -1;
  flightPlanLastStatsPhase = -1;
  flightPlanDisplayDirty = true;
  if (flightPlanCount == 0) {
    Serial.println("Flight plan cleared from RAM");
  } else {
    Serial.printf("Flight plan stored in RAM: %u flight(s), validity %lu-%lu\n",
                  flightPlanCount,
                  static_cast<unsigned long>(flightPlanStartValidity),
                  static_cast<unsigned long>(flightPlanEndValidity));
  }

  response["success"] = true;
  response["storedIn"] = "RAM";
  response["flightCount"] = flightPlanCount;
  if (flightPlanCount > 0) {
    displayNotice = source == FLIGHT_PLAN_FROM_MQTT
                        ? DISPLAY_NOTICE_RECEIVED_MQTT
                        : DISPLAY_NOTICE_RECEIVED_PORTAL;
    displayNoticeHasRendered = false;
  }
  return 200;
}

void handlePostFlightPlan() {
  JsonDocument request;
  DeserializationError error = deserializeJson(request, configServer.arg("plain"));
  if (error) {
    JsonDocument response;
    response["success"] = false;
    response["message"] = "Invalid flight plan JSON.";
    sendJsonResponse(400, response);
    return;
  }

  JsonDocument response;
  sendJsonResponse(
      storeFlightPlan(request, response, FLIGHT_PLAN_FROM_PORTAL), response);
}

bool appendMqttString(std::vector<uint8_t> &packet, const String &value) {
  if (value.length() > UINT16_MAX) {
    return false;
  }
  packet.push_back(static_cast<uint8_t>(value.length() >> 8));
  packet.push_back(static_cast<uint8_t>(value.length()));
  packet.insert(packet.end(), value.c_str(),
                value.c_str() + value.length());
  return true;
}

void appendMqttRemainingLength(std::vector<uint8_t> &packet, size_t length) {
  do {
    uint8_t encodedByte = static_cast<uint8_t>(length % 128);
    length /= 128;
    if (length > 0) {
      encodedByte |= 0x80;
    }
    packet.push_back(encodedByte);
  } while (length > 0);
}

bool sendMqttConnect() {
  std::vector<uint8_t> body;
  if (!appendMqttString(body, "MQTT")) {
    return false;
  }
  body.push_back(4);
  uint8_t connectFlags = 0x02;
  bool hasUsername = !config.mqttUsername.isEmpty() ||
                     !config.mqttPassword.isEmpty();
  if (hasUsername) {
    connectFlags |= 0x80;
  }
  if (!config.mqttPassword.isEmpty()) {
    connectFlags |= 0x40;
  }
  body.push_back(connectFlags);
  body.push_back(0);
  body.push_back(60);

  String clientId = "FlightRadar-" + WiFi.macAddress();
  clientId.replace(":", "");
  if (!appendMqttString(body, clientId) ||
      (hasUsername &&
       !appendMqttString(body, config.mqttUsername)) ||
      (!config.mqttPassword.isEmpty() &&
       !appendMqttString(body, config.mqttPassword))) {
    return false;
  }

  std::vector<uint8_t> packet;
  packet.reserve(body.size() + 5);
  packet.push_back(0x10);
  appendMqttRemainingLength(packet, body.size());
  packet.insert(packet.end(), body.begin(), body.end());
  return mqttWebSocket.sendBIN(packet.data(), packet.size());
}

bool sendMqttSubscribe() {
  std::vector<uint8_t> body;
  body.push_back(static_cast<uint8_t>(MQTT_SUBSCRIBE_PACKET_ID >> 8));
  body.push_back(static_cast<uint8_t>(MQTT_SUBSCRIBE_PACKET_ID));
  if (!appendMqttString(body, config.mqttTopic)) {
    return false;
  }
  body.push_back(0);

  std::vector<uint8_t> packet;
  packet.reserve(body.size() + 5);
  packet.push_back(0x82);
  appendMqttRemainingLength(packet, body.size());
  packet.insert(packet.end(), body.begin(), body.end());
  return mqttWebSocket.sendBIN(packet.data(), packet.size());
}

bool sendMqttAcknowledgement(uint8_t packetType, uint16_t packetId) {
  const uint8_t packet[] = {
      static_cast<uint8_t>(packetType << 4),
      2,
      static_cast<uint8_t>(packetId >> 8),
      static_cast<uint8_t>(packetId),
  };
  return mqttWebSocket.sendBIN(packet, sizeof(packet));
}

bool decodeMqttRemainingLength(const uint8_t *packet, size_t packetLength,
                               size_t &offset, size_t &remainingLength) {
  remainingLength = 0;
  size_t multiplier = 1;
  for (uint8_t index = 0; index < 4; index++) {
    if (offset >= packetLength) {
      return false;
    }
    uint8_t encodedByte = packet[offset++];
    remainingLength += (encodedByte & 0x7F) * multiplier;
    if ((encodedByte & 0x80) == 0) {
      return remainingLength <= packetLength - offset;
    }
    multiplier *= 128;
  }
  return false;
}

void processMqttPublish(const uint8_t *packet, size_t packetLength,
                        size_t offset, size_t remainingLength) {
  if (remainingLength < 2) {
    Serial.println("MQTT publish rejected: missing topic");
    return;
  }
  const uint8_t qos = (packet[0] >> 1) & 0x03;
  if (qos > 1) {
    Serial.println("MQTT publish rejected: unsupported QoS");
    return;
  }
  uint16_t topicLength =
      (static_cast<uint16_t>(packet[offset]) << 8) | packet[offset + 1];
  offset += 2;
  if (topicLength > remainingLength - 2 ||
      offset + topicLength > packetLength) {
    Serial.println("MQTT publish rejected: invalid topic length");
    return;
  }
  String topic;
  topic.reserve(topicLength);
  for (uint16_t index = 0; index < topicLength; index++) {
    topic += static_cast<char>(packet[offset + index]);
  }
  offset += topicLength;
  remainingLength -= 2 + topicLength;

  uint16_t packetId = 0;
  if (qos == 1) {
    if (remainingLength < 2) {
      Serial.println("MQTT publish rejected: missing packet identifier");
      return;
    }
    packetId = (static_cast<uint16_t>(packet[offset]) << 8) |
               packet[offset + 1];
    offset += 2;
    remainingLength -= 2;
  }

  if (topic == config.mqttTopic) {
    if (remainingLength > MQTT_MAX_PAYLOAD_SIZE) {
      Serial.printf("MQTT flight message too large: %u bytes\n",
                    static_cast<unsigned>(remainingLength));
    } else {
      JsonDocument request;
      DeserializationError error =
          deserializeJson(request, packet + offset, remainingLength);
      if (error) {
        Serial.printf("MQTT flight JSON rejected: %s\n", error.c_str());
      } else {
        JsonDocument response;
        int statusCode =
            storeFlightPlan(request, response, FLIGHT_PLAN_FROM_MQTT);
        if (statusCode == 200) {
          Serial.printf("MQTT flight plan received: %u flight(s)\n",
                        flightPlanCount);
        } else {
          Serial.printf("MQTT flight plan rejected: %s\n",
                        response["message"] | "invalid payload");
        }
      }
    }
  }

  if (qos == 1 && !sendMqttAcknowledgement(4, packetId)) {
    Serial.println("Failed to acknowledge MQTT publish");
  }
}

void processMqttPacket(const uint8_t *packet, size_t packetLength) {
  if (packetLength < 2) {
    Serial.println("MQTT packet rejected: packet too short");
    return;
  }
  size_t offset = 1;
  size_t remainingLength = 0;
  if (!decodeMqttRemainingLength(packet, packetLength, offset,
                                 remainingLength) ||
      offset + remainingLength != packetLength) {
    Serial.println("MQTT packet rejected: invalid packet length");
    return;
  }

  switch (packet[0] >> 4) {
    case 2:
      if (remainingLength != 2 || packet[offset + 1] != 0) {
        Serial.printf("MQTT connection refused (code %u)\n",
                      remainingLength == 2 ? packet[offset + 1] : 0xFF);
        mqttWebSocket.disconnect();
        return;
      }
      mqttConnected = true;
      lastMqttKeepAliveMs = millis();
      Serial.println("MQTT connected");
      if (!sendMqttSubscribe()) {
        Serial.println("Failed to send MQTT subscription");
        mqttWebSocket.disconnect();
      }
      break;
    case 3:
      if (!mqttConnected) {
        Serial.println("MQTT publish received before connection");
        return;
      }
      processMqttPublish(packet, packetLength, offset, remainingLength);
      break;
    case 9:
      if (remainingLength >= 3 &&
          packet[offset] == static_cast<uint8_t>(MQTT_SUBSCRIBE_PACKET_ID >> 8) &&
          packet[offset + 1] == static_cast<uint8_t>(MQTT_SUBSCRIBE_PACKET_ID) &&
          packet[offset + 2] != 0x80) {
        mqttSubscribed = true;
        Serial.printf("Subscribed to MQTT topic %s\n",
                      config.mqttTopic.c_str());
      } else {
        Serial.println("MQTT subscription failed");
        mqttWebSocket.disconnect();
      }
      break;
    case 13:
      lastMqttKeepAliveMs = millis();
      break;
    default:
      break;
  }
}

void onMqttWebSocketEvent(WStype_t type, uint8_t *payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      mqttConnected = false;
      mqttSubscribed = false;
      Serial.println("MQTT WebSocket connected; sending login");
      if (!sendMqttConnect()) {
        Serial.println("Failed to send MQTT login");
        mqttWebSocket.disconnect();
      }
      break;
    case WStype_DISCONNECTED:
      mqttConnected = false;
      mqttSubscribed = false;
      Serial.println("MQTT WebSocket disconnected");
      break;
    case WStype_BIN:
      processMqttPacket(payload, length);
      break;
    case WStype_ERROR:
      Serial.println("MQTT WebSocket error");
      break;
    default:
      break;
  }
}

void startMqttClient() {
  if (WiFi.status() != WL_CONNECTED ||
      time(nullptr) < VALID_SYSTEM_TIME_EPOCH) {
    return;
  }
  if (lastMqttStartAttemptMs != 0 &&
      millis() - lastMqttStartAttemptMs < 5000) {
    return;
  }
  lastMqttStartAttemptMs = millis();

  mqttWebSocket.disconnect();
  mqttConnected = false;
  mqttSubscribed = false;
  mqttClientStarted = false;
  String host = config.mqttIP;
  host.trim();
  int schemeEnd = host.indexOf("://");
  if (schemeEnd >= 0) {
    host = host.substring(schemeEnd + 3);
  }
  int pathStart = host.indexOf('/');
  if (pathStart >= 0) {
    host = host.substring(0, pathStart);
  }
  if (host.isEmpty()) {
    Serial.println("MQTT WebSocket host is empty; check configuration");
    return;
  }

  mqttWebSocket.onEvent(onMqttWebSocketEvent);
  mqttWebSocket.setReconnectInterval(5000);
  mqttWebSocket.enableHeartbeat(15000, 3000, 2);
  mqttWebSocket.beginSslWithCA(host.c_str(), config.mqttPort,
                               config.mqttWebSocketPath.c_str(),
                               MQTT_CA_CERT, "mqtt");
  mqttClientStarted = true;
  mqttRestartPending = false;
  Serial.printf("Connecting to MQTT WebSocket %s:%u%s\n", host.c_str(),
                config.mqttPort, config.mqttWebSocketPath.c_str());
}

void updateMqttClient() {
  if (WiFi.status() != WL_CONNECTED) {
    if (mqttClientStarted) {
      mqttWebSocket.disconnect();
      mqttClientStarted = false;
      mqttConnected = false;
      mqttSubscribed = false;
    }
    return;
  }
  if (time(nullptr) < VALID_SYSTEM_TIME_EPOCH) {
    return;
  }
  if (mqttRestartPending || !mqttClientStarted) {
    startMqttClient();
  }
  if (!mqttClientStarted) {
    return;
  }
  mqttWebSocket.loop();
  if (mqttConnected &&
      millis() - lastMqttKeepAliveMs >= MQTT_KEEPALIVE_INTERVAL_MS) {
    const uint8_t pingRequest[] = {0xC0, 0x00};
    if (!mqttWebSocket.sendBIN(pingRequest, sizeof(pingRequest))) {
      Serial.println("Failed to send MQTT keepalive");
    }
    lastMqttKeepAliveMs = millis();
  }
}

void handlePostConfiguration() {
  if (configWifiChangePending) {
    JsonDocument response;
    response["success"] = false;
    response["message"] = "A Wi-Fi configuration attempt is already in progress.";
    sendJsonResponse(409, response);
    return;
  }

  JsonDocument request;
  DeserializationError error = deserializeJson(request, configServer.arg("plain"));
  if (error || !request.is<JsonObject>()) {
    JsonDocument response;
    response["success"] = false;
    response["message"] = "Invalid configuration JSON.";
    sendJsonResponse(400, response);
    return;
  }

  JsonObjectConst fields = request.as<JsonObjectConst>();
  AppConfig candidate = config;
  String submittedPassword;
  String submittedApiKey;
  String submittedMqttUsername;
  String submittedMqttPassword;
  double mqttPort = 0;

  bool valid =
      readConfigString(fields, "wifiSSID", candidate.wifiSSID, 32) &&
      readConfigString(fields, "wifiPassword", submittedPassword, 63, true) &&
      readConfigString(fields, "aeroApiKey", submittedApiKey, 256, true) &&
      readConfigString(fields, "aeroApiURL", candidate.aeroApiURL, 192) &&
      readConfigString(fields, "mqttIP", candidate.mqttIP, 128) &&
      readConfigNumber(fields, "mqttPort", mqttPort) &&
      readConfigString(fields, "mqttWebSocketPath",
                       candidate.mqttWebSocketPath, 128) &&
      readConfigString(fields, "mqttTopic", candidate.mqttTopic, 128) &&
      readConfigString(fields, "mqttDataName", candidate.mqttDataName, 64) &&
      readConfigNumber(fields, "latitude", candidate.latitude) &&
      readConfigNumber(fields, "longitude", candidate.longitude) &&
      readConfigNumber(fields, "searchRadius", candidate.searchRadius);

  if (hasJsonField(fields, "mqttUsername")) {
    valid = valid &&
            readConfigString(fields, "mqttUsername", submittedMqttUsername,
                             128, true);
  }
  if (hasJsonField(fields, "mqttPassword")) {
    valid = valid &&
            readConfigString(fields, "mqttPassword", submittedMqttPassword,
                             128, true);
  }

  if (!valid || mqttPort < 1 || mqttPort > 65535 ||
      floor(mqttPort) != mqttPort || candidate.latitude < -90 ||
      candidate.latitude > 90 || candidate.longitude < -180 ||
      candidate.longitude > 180 || candidate.searchRadius < 0.5 ||
      candidate.searchRadius > 100.0 ||
      !isValidMqttWebSocketPath(candidate.mqttWebSocketPath) ||
      (!candidate.aeroApiURL.startsWith("http://") &&
       !candidate.aeroApiURL.startsWith("https://"))) {
    JsonDocument response;
    response["success"] = false;
    response["message"] = "Configuration values are missing or outside allowed limits.";
    sendJsonResponse(400, response);
    return;
  }

  candidate.mqttPort = static_cast<uint16_t>(mqttPort);
  if (submittedPassword.length() > 0) {
    candidate.wifiPassword = submittedPassword;
  }
  if (submittedApiKey.length() > 0) {
    candidate.aeroApiKey = submittedApiKey;
  }
  if (submittedMqttUsername.length() > 0) {
    candidate.mqttUsername = submittedMqttUsername;
  }
  if (submittedMqttPassword.length() > 0) {
    candidate.mqttPassword = submittedMqttPassword;
  }

  if (!configurationChanged(config, candidate)) {
    if (configAccessPointActive && WiFi.status() != WL_CONNECTED) {
      pendingConfig = candidate;
      configWifiChangePending = true;
      configWifiChangeStartPending = true;
      configApplySucceeded = false;
      configApplyMessage = "Retrying the saved Wi-Fi credentials.";
      JsonDocument response;
      response["success"] = true;
      response["pending"] = true;
      response["message"] = configApplyMessage;
      sendJsonResponse(202, response);
      return;
    }

    if (configAccessPointActive) {
      configAccessPointShutdownAtMs =
          millis() + CONFIG_AP_SHUTDOWN_DELAY_MS;
    }
    configApplySucceeded = true;
    configApplyMessage = "Configuration is unchanged.";
    JsonDocument response;
    response["success"] = true;
    response["pending"] = false;
    response["message"] = configApplyMessage;
    sendJsonResponse(200, response);
    return;
  }

  const bool wifiChanged = candidate.wifiSSID != config.wifiSSID ||
                           candidate.wifiPassword != config.wifiPassword;
  if (!wifiChanged) {
    AppConfig previousConfig = config;
    config = candidate;
    if (!saveConfiguration()) {
      config = previousConfig;
      JsonDocument response;
      response["success"] = false;
      response["message"] = "Could not save configuration to flash.";
      sendJsonResponse(500, response);
      return;
    }

    logConfigurationChanges(previousConfig, config);
    if (previousConfig.mqttIP != config.mqttIP ||
        previousConfig.mqttPort != config.mqttPort ||
        previousConfig.mqttWebSocketPath != config.mqttWebSocketPath ||
        previousConfig.mqttTopic != config.mqttTopic ||
        previousConfig.mqttUsername != config.mqttUsername ||
        previousConfig.mqttPassword != config.mqttPassword) {
      mqttRestartPending = true;
    }
    lastFetchMs = 0;
    if (configAccessPointActive) {
      configAccessPointShutdownAtMs =
          millis() + CONFIG_AP_SHUTDOWN_DELAY_MS;
    }
    configApplySucceeded = true;
    configApplyMessage = "Configuration saved.";
    JsonDocument response;
    response["success"] = true;
    response["pending"] = false;
    response["message"] = configApplyMessage;
    sendJsonResponse(200, response);
    return;
  }

  pendingConfig = candidate;
  configWifiChangePending = true;
  configApplySucceeded = false;
  configApplyMessage = "Testing the new Wi-Fi credentials.";
  JsonDocument response;
  response["success"] = true;
  response["pending"] = true;
  response["message"] = configApplyMessage;
  sendJsonResponse(202, response);
  configWifiChangeStartPending = true;
}

void showConfigFailure(const String &originalSSID) {
  display->fillScreen(colBlack);
  printClippedCentered(0, PANEL_RES_X, 0, "Failed to", colRed);
  printClippedCentered(0, PANEL_RES_X, 8, "Config!", colRed);
  printClippedCentered(0, PANEL_RES_X, 16, "Original WiFi:", colTextSecondary);
  printClippedCentered(0, PANEL_RES_X, 24, originalSSID.c_str(), colTextPrimary);
}

void finishWifiConfigurationFailure(const char *reason) {
  configWifiChangePending = false;
  configApplySucceeded = false;
  configApplyMessage = reason;
  Serial.printf("Wi-Fi configuration failed: %s; restoring SSID %s\n",
                reason, config.wifiSSID.c_str());
  WiFi.begin(config.wifiSSID.c_str(), config.wifiPassword.c_str());
  showConfigFailure(config.wifiSSID);
}

void updateConfigWifiChange() {
  if (configWifiChangeStartPending) {
    configWifiChangeStartPending = false;
    configWifiChangeStartedMs = millis();
    WiFi.disconnect(false, false);
    WiFi.begin(pendingConfig.wifiSSID.c_str(),
               pendingConfig.wifiPassword.c_str());
    return;
  }

  if (!configWifiChangePending) {
    return;
  }

  if (millis() - configWifiChangeStartedMs >= 1000 &&
      WiFi.status() == WL_CONNECTED &&
      WiFi.SSID() == pendingConfig.wifiSSID) {
    AppConfig previousConfig = config;
    config = pendingConfig;
    if (!saveConfiguration()) {
      config = previousConfig;
      finishWifiConfigurationFailure("Wi-Fi connected, but flash save failed.");
      return;
    }

    logConfigurationChanges(previousConfig, config);
    if (previousConfig.mqttIP != config.mqttIP ||
        previousConfig.mqttPort != config.mqttPort ||
        previousConfig.mqttWebSocketPath != config.mqttWebSocketPath ||
        previousConfig.mqttTopic != config.mqttTopic ||
        previousConfig.mqttUsername != config.mqttUsername ||
        previousConfig.mqttPassword != config.mqttPassword) {
      mqttRestartPending = true;
    }
    configWifiChangePending = false;
    configApplySucceeded = true;
    configApplyMessage = "Wi-Fi connected and configuration saved.";
    wifiEverConnected = true;
    if (configAccessPointActive) {
      configAccessPointShutdownAtMs =
          millis() + CONFIG_AP_SHUTDOWN_DELAY_MS;
    }
    lastFetchMs = 0;
    configSuccessScreenActive = true;
    configSuccessIpScreenActive = false;
    configSuccessScreenStartedMs = millis();
    if (mdnsStarted) {
      MDNS.end();
      mdnsStarted = false;
    }
    startFlightMdns();
    showStatus("WiFi OK", config.wifiSSID.c_str());
    Serial.printf("Wi-Fi configuration saved for SSID %s\n",
                  config.wifiSSID.c_str());
    return;
  }

  if (millis() - configWifiChangeStartedMs >=
      CONFIG_WIFI_CONNECT_TIMEOUT_MS) {
    finishWifiConfigurationFailure("Could not connect within 45 seconds.");
  }
}

bool startConfigAccessPoint() {
  if (configAccessPointActive) {
    return true;
  }

  WiFi.mode(WIFI_AP_STA);
  WiFi.setSleep(false);

  String macAddress = WiFi.macAddress();
  macAddress.replace(":", "");
  if (macAddress.length() < 8) {
    Serial.println("Could not derive configuration access point password");
    return false;
  }

  String suffix = macAddress.substring(macAddress.length() - 4);
  char passwordBuffer[11];
  snprintf(passwordBuffer, sizeof(passwordBuffer), "%08lX%02lX",
           static_cast<unsigned long>(esp_random()),
           static_cast<unsigned long>(esp_random() & 0xFF));
  String password(passwordBuffer);
  String ssid = "Radar-" + suffix;
  if (!WiFi.softAP(ssid.c_str(), password.c_str())) {
    Serial.println("Could not start configuration access point");
    return false;
  }

  configAccessPointActive = true;
  configAccessPointShutdownAtMs = 0;
  display->fillScreen(colBlack);
  printClippedCentered(0, PANEL_RES_X, 0, "SETUP WIFI", colTextSecondary);
  printClippedCentered(0, PANEL_RES_X, 8, ssid.c_str(), colTextPrimary);
  printClippedCentered(0, PANEL_RES_X, 16, password.c_str(), colAmber);
  printClippedCentered(0, PANEL_RES_X, 24, "192168.4.1", colTextSecondary);
  Serial.printf("Configuration access point started: SSID=%s password=%s\n",
                ssid.c_str(), password.c_str());
  Serial.println("Join the access point and open http://192.168.4.1/config.html");
  return true;
}

void updateConfigAccessPoint() {
  if (!configAccessPointActive || configAccessPointShutdownAtMs == 0 ||
      static_cast<int32_t>(millis() - configAccessPointShutdownAtMs) < 0) {
    return;
  }

  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_STA);
  configAccessPointActive = false;
  configAccessPointShutdownAtMs = 0;
  Serial.println("Configuration access point stopped");
}

void setupConfigServer() {
  auto serveConfigurationPage = []() {
    File page = LittleFS.open("/config.html", FILE_READ);
    if (!page) {
      configServer.send(500, "text/plain",
                        "Configuration page is missing from LittleFS.");
      return;
    }
    configServer.streamFile(page, "text/html");
    page.close();
  };
  configServer.on("/", HTTP_GET, []() {
    configServer.sendHeader("Location", "/config.html");
    configServer.send(302, "text/plain", "");
  });
  configServer.on("/config.html", HTTP_GET, serveConfigurationPage);
  configServer.on("/api/config", HTTP_GET, handleGetConfiguration);
  configServer.on("/api/config", HTTP_POST, handlePostConfiguration);
  configServer.on("/api/config/status", HTTP_GET, handleGetConfigurationStatus);
  configServer.on("/api/flights", HTTP_GET, handleGetFlightPlan);
  configServer.on("/api/flights", HTTP_POST, handlePostFlightPlan);
  configServer.onNotFound([]() {
    configServer.send(404, "text/plain", "Not found");
  });
  configServer.begin();
  Serial.println("Configuration web server started on port 80");
}

void startFlightMdns() {
  if (!MDNS.begin(MDNS_HOSTNAME)) {
    Serial.println("mDNS startup failed; use the device IP address");
    return;
  }

  MDNS.addService("http", "tcp", 80);
  mdnsStarted = true;
  Serial.printf("Configuration page available at http://%s.local/config.html\n",
                MDNS_HOSTNAME);
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

// SkyWest Airlines — 19x15 monochrome bitmap (X = white, . = black).
void drawSkwLogo(int x, int y) {
  static const char *rows[] = {
      "XXXXXXXXXXXXXXXXXXX",
      "X.................X",
      "X.................X",
      "X.....XXXXXXX.....X",
      "X....X...X...X....X",
      "X........X........X",
      "X........X........X",
      "X.......XXX.......X",
      "X......XXXXX......X",
      "XXXXXXX.XXX.XXXXXXX",
      "X........X........X",
      "X.................X",
      "X.................X",
      "X.................X",
      "XXXXXXXXXXXXXXXXXXX",
  };
  const int logoH = (int)(sizeof(rows) / sizeof(rows[0]));
  drawBitmapMono(x, y, rows, logoH, colWhite, colBlack);
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
  if (strncmp(prefix, "SKW", 3) == 0) {
    drawSkwLogo(x, y);
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

bool parsePlaneSnapshot(JsonObject plane, AircraftSnapshot &snap,
                        bool includeGround = false) {
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

  if ((plane["lat"].is<double>() || plane["lat"].is<int>()) &&
      (plane["lon"].is<double>() || plane["lon"].is<int>())) {
    snap.latitude = plane["lat"].as<float>();
    snap.longitude = plane["lon"].as<float>();
    snap.hasPosition = isfinite(snap.latitude) && isfinite(snap.longitude) &&
                       snap.latitude >= -90.0f && snap.latitude <= 90.0f &&
                       snap.longitude >= -180.0f && snap.longitude <= 180.0f;
  }

  const char *category = plane["category"] | "";
  snap.isHelicopter = (strncmp(category, "A7", 2) == 0);

  snap.distanceKm = plane["dst"] | 9999.0f;
  snap.bearing = plane["dir"] | 0.0f;
  snap.track = plane["track"] | -1.0f;
  snap.groundSpeedKts = plane["gs"] | 0.0f;
  snap.verticalRateFpm = plane["baro_rate"] | plane["geom_rate"] | 0;
  parseAltitudeFromPlane(plane, snap.onGround, snap.altitudeFt);
#if TRACK_AIRBORNE_ONLY
  if (snap.onGround && !includeGround) {
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

void updateAircraftTypeSlotIfNeeded() {
  if (!callsignTypeSwapPending ||
      millis() - typeSlotStartedMs < REFRESH_INTERVAL_MS / 2) {
    return;
  }

  callsignTypeSwapPending = false;
  showCallsignInTypeSlot = true;
  showFlight();
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
  if (operatingMode == MODE_FLIGHT_PLAN) {
    drawFlightPlanDisplay();
    return;
  }

  display->setBrightness8(activeBrightness());
  display->fillScreen(colBlack);

  drawAirlineSymbol(nearestCallsign.c_str(), nearestIsHelicopter);

  printClippedCentered(FLIGHT_TEXT_X, FLIGHT_TEXT_W, FLIGHT_ROW_CALLSIGN,
                       nearestCallsign.c_str(), colCallsign);
  const char *typeSlotText = showCallsignInTypeSlot
                                 ? nearestCallsign.c_str()
                                 : nearestAircraftType.c_str();
  uint16_t typeSlotColor = showCallsignInTypeSlot ? colCallsign : colType;
  printClippedCentered(FLIGHT_TEXT_X, FLIGHT_TEXT_W, FLIGHT_ROW_TYPE,
                       typeSlotText, typeSlotColor);

  display->drawFastHLine(0, FLIGHT_ROW_DIVIDER, PANEL_RES_X,
                         display->color565(30, 30, 30));

  drawBottomLine();
}

bool formatUnixFlightTime(uint32_t timestamp, char *buffer,
                          size_t bufferSize) {
  if (timestamp == 0) {
    return false;
  }

  time_t flightTime = static_cast<time_t>(timestamp);
  struct tm utcTime;
  if (gmtime_r(&flightTime, &utcTime) == nullptr ||
      strftime(buffer, bufferSize, "%H:%MZ", &utcTime) == 0) {
    return false;
  }
  return true;
}

void drawFlightPlanScrollingRow(int y, const String &text, uint16_t color) {
  const int maxChars = PANEL_RES_X / TEXT_CHAR_W;
  if (text.length() <= static_cast<size_t>(maxChars)) {
    printClippedCentered(0, PANEL_RES_X, y, text.c_str(), color);
    return;
  }

  const size_t cycleLength = text.length() + 3;
  const size_t offset =
      (millis() / FLIGHT_PLAN_SCROLL_INTERVAL_MS) % cycleLength;
  char window[12];
  for (int i = 0; i < maxChars; i++) {
    size_t index = (offset + i) % cycleLength;
    window[i] = index < text.length() ? text[index] : ' ';
  }
  window[maxChars] = '\0';
  printClippedCentered(0, PANEL_RES_X, y, window, color);
}

const char *flightPlanArrivalStatus() {
  String status = flightPlanAeroStatus;
  status.toLowerCase();
  if (status.indexOf("delayed") >= 0) {
    return "DELAYED";
  }
  if (flightPlanScheduledEta == 0 || flightPlanEstimatedEta == 0) {
    return "STATUS N/A";
  }

  int64_t delta = static_cast<int64_t>(flightPlanEstimatedEta) -
                  static_cast<int64_t>(flightPlanScheduledEta);
  if (delta < -900) {
    return "EARLY";
  }
  if (delta > 900) {
    return "LATE";
  }
  return "ON TIME";
}

void drawFlightPlanPositionRow(int phase) {
  display->fillRect(0, FLIGHT_ROW_BOTTOM, PANEL_RES_X, TEXT_CHAR_H, colBlack);
  if (phase == 0) {
    char route[24];
    if (!flightPlanOrigin.isEmpty() && !flightPlanDestination.isEmpty()) {
      snprintf(route, sizeof(route), "%s>%s", flightPlanOrigin.c_str(),
               flightPlanDestination.c_str());
    } else {
      copyStringField(route, sizeof(route), "ROUTE UNKNOWN");
    }
    drawFlightPlanScrollingRow(FLIGHT_ROW_BOTTOM, route, colRoute);
  } else {
    drawFlightPlanScrollingRow(FLIGHT_ROW_BOTTOM, flightPlanLocation,
                               colRoute);
  }
}

void drawFlightPlanStatsRow(int phase) {
  display->fillRect(0, FLIGHT_ROW_STATS, PANEL_RES_X, TEXT_CHAR_H, colBlack);
  if (flightPlanLandingHoldActive &&
      millis() - flightPlanLandingHoldStartedMs <
          FLIGHT_PLAN_LANDED_HOLD_MS) {
    printClippedCentered(0, PANEL_RES_X, FLIGHT_ROW_STATS, "LANDED",
                         colStats);
    return;
  }

  if (phase == 0) {
    if (!flightPlanHasLiveData) {
      printClippedCentered(0, PANEL_RES_X, FLIGHT_ROW_STATS,
                           "WAITING", colStats);
      return;
    }

    nearestGroundSpeedKts = flightPlanAircraft.groundSpeedKts;
    nearestAltitudeFt = flightPlanAircraft.altitudeFt;
    nearestVerticalRateFpm = flightPlanAircraft.verticalRateFpm;
    nearestOnGround = flightPlanAircraft.onGround;
    char speedBuf[12];
    char altBuf[16];
    prepareStatsFields(speedBuf, sizeof(speedBuf), altBuf, sizeof(altBuf));
    LayoutSlot stats[] = {
        {LAYOUT_TEXT, speedBuf, colStats},
        {LAYOUT_TEXT, altBuf, altitudeColor()},
    };
    drawCenteredColumnRow(FLIGHT_ROW_STATS, stats, 2);
  } else if (phase == 1) {
    char eta[12] = "--:--Z";
    uint32_t displayTime =
        flightPlanHasLiveData && flightPlanAircraft.onGround
            ? flightPlanEtd
            : flightPlanEta;
    if (displayTime > 0) {
      formatUnixFlightTime(displayTime, eta, sizeof(eta));
    }
    char timeLine[16];
    snprintf(timeLine, sizeof(timeLine), "%s %s",
             flightPlanHasLiveData && flightPlanAircraft.onGround ? "ETD"
                                                                  : "ETA",
             eta);
    printClippedCentered(0, PANEL_RES_X, FLIGHT_ROW_STATS, timeLine,
                         colStats);
  } else {
    const char *status = flightPlanArrivalStatus();
    uint16_t statusColor =
        strcmp(status, "DELAYED") == 0 || strcmp(status, "LATE") == 0
            ? colStatsCaution
            : colStats;
    printClippedCentered(0, PANEL_RES_X, FLIGHT_ROW_STATS, status,
                         statusColor);
  }
}

void drawFlightPlanBottomRows() {
  const unsigned long elapsedMs =
      millis() - flightPlanDisplayCycleStartedMs;
  const int positionPhase =
      static_cast<int>((elapsedMs / FLIGHT_PLAN_DISPLAY_PHASE_MS) % 2);
  const int statsPhase =
      static_cast<int>((elapsedMs / FLIGHT_PLAN_DISPLAY_PHASE_MS) % 3);
  drawFlightPlanPositionRow(positionPhase);
  drawFlightPlanStatsRow(statsPhase);
}

void updateFlightPlanDisplayAlternation() {
  if (millis() - flightPlanLastBottomRenderMs <
      FLIGHT_PLAN_SCROLL_INTERVAL_MS) {
    return;
  }

  const unsigned long elapsedMs =
      millis() - flightPlanDisplayCycleStartedMs;
  const int positionPhase =
      static_cast<int>((elapsedMs / FLIGHT_PLAN_DISPLAY_PHASE_MS) % 2);
  const int statsPhase =
      static_cast<int>((elapsedMs / FLIGHT_PLAN_DISPLAY_PHASE_MS) % 3);
  const bool scrollingLocation =
      positionPhase == 1 &&
      flightPlanLocation.length() > PANEL_RES_X / TEXT_CHAR_W;
  const bool updatePosition =
      positionPhase != flightPlanLastPositionPhase || scrollingLocation;
  const bool updateStats = statsPhase != flightPlanLastStatsPhase;
  if (!updatePosition && !updateStats) {
    return;
  }

  if (updatePosition) {
    drawFlightPlanPositionRow(positionPhase);
  }
  if (updateStats) {
    drawFlightPlanStatsRow(statsPhase);
  }
  flightPlanLastPositionPhase = positionPhase;
  flightPlanLastStatsPhase = statsPhase;
  flightPlanLastBottomRenderMs = millis();
}

bool getCurrentFlightPlanLeg(JsonDocument &plan, JsonObjectConst &flight) {
  if (deserializeJson(plan, flightPlanJson)) {
    return false;
  }
  String key = String(flightPlanCurrentIndex + 1);
  flight = plan["flight_data"][key].as<JsonObjectConst>();
  return !flight.isNull();
}

void drawFlightPlanDisplay() {
  display->setBrightness8(activeBrightness());
  display->fillScreen(colBlack);
  if (flightPlanCount == 0 || flightPlanJson.isEmpty()) {
    printClippedCentered(0, PANEL_RES_X, FLIGHT_ROW_CALLSIGN,
                         "NO FLIGHT PLAN", colStats);
    return;
  }

  JsonDocument plan;
  JsonObjectConst flight;
  if (!getCurrentFlightPlanLeg(plan, flight)) {
    Serial.println("Could not read the active flight-plan leg");
    printClippedCentered(0, PANEL_RES_X, FLIGHT_ROW_CALLSIGN,
                         "PLAN DATA ERROR", colRed);
    return;
  }

  const char *flightNumber = flight["flight"] | "UNKNOWN";
  drawAirlineSymbol(flightNumber,
                    flightPlanHasLiveData &&
                        flightPlanAircraft.isHelicopter);
  printClippedCentered(FLIGHT_TEXT_X, FLIGHT_TEXT_W,
                       FLIGHT_ROW_CALLSIGN, flightNumber, colCallsign);
  const char *aircraftType =
      flightPlanHasLiveData ? flightPlanAircraft.type : "WAITING";
  printClippedCentered(FLIGHT_TEXT_X, FLIGHT_TEXT_W, FLIGHT_ROW_TYPE,
                       aircraftType, colType);
  display->drawFastHLine(0, FLIGHT_ROW_DIVIDER, PANEL_RES_X,
                         display->color565(30, 30, 30));

  drawFlightPlanBottomRows();
  unsigned long elapsedMs = millis() - flightPlanDisplayCycleStartedMs;
  flightPlanLastPositionPhase =
      static_cast<int>((elapsedMs / FLIGHT_PLAN_DISPLAY_PHASE_MS) % 2);
  flightPlanLastStatsPhase =
      static_cast<int>((elapsedMs / FLIGHT_PLAN_DISPLAY_PHASE_MS) % 3);
  flightPlanLastBottomRenderMs = millis();
}

bool parseAeroApiTimestamp(const char *text, uint32_t &timestamp) {
  if (text == nullptr || strlen(text) < 20) {
    return false;
  }

  int year, month, day, hour, minute, second, consumed = 0;
  if (sscanf(text, "%4d-%2d-%2dT%2d:%2d:%2d%n",
             &year, &month, &day, &hour, &minute, &second,
             &consumed) != 6 ||
      month < 1 || month > 12 || day < 1 || day > 31 ||
      hour > 23 || minute > 59 || second > 60) {
    return false;
  }

  const char *zone = text + consumed;
  while (*zone >= '0' && *zone <= '9') {
    zone++;
  }
  if (*zone == '.') {
    zone++;
    while (*zone >= '0' && *zone <= '9') {
      zone++;
    }
  }

  int offsetSeconds = 0;
  if (*zone == '+' || *zone == '-') {
    int offsetHour = 0;
    int offsetMinute = 0;
    if (sscanf(zone + 1, "%2d:%2d", &offsetHour, &offsetMinute) != 2 ||
        offsetHour > 23 || offsetMinute > 59) {
      return false;
    }
    offsetSeconds = (offsetHour * 60 + offsetMinute) * 60;
    if (*zone == '-') {
      offsetSeconds = -offsetSeconds;
    }
  } else if (*zone != 'Z' && *zone != '\0') {
    return false;
  }

  int adjustedYear = year - (month <= 2);
  int era = (adjustedYear >= 0 ? adjustedYear : adjustedYear - 399) / 400;
  unsigned yearOfEra = static_cast<unsigned>(adjustedYear - era * 400);
  unsigned adjustedMonth = static_cast<unsigned>(
      month + (month > 2 ? -3 : 9));
  unsigned dayOfYear = (153 * adjustedMonth + 2) / 5 +
                       static_cast<unsigned>(day - 1);
  unsigned dayOfEra = yearOfEra * 365 + yearOfEra / 4 -
                      yearOfEra / 100 + dayOfYear;
  int64_t days = static_cast<int64_t>(era) * 146097 + dayOfEra - 719468;
  int64_t seconds = days * 86400 + hour * 3600 + minute * 60 + second -
                    offsetSeconds;
  if (seconds <= 0 || seconds > UINT32_MAX) {
    return false;
  }
  timestamp = static_cast<uint32_t>(seconds);
  return true;
}

bool readAeroApiTimestamp(JsonVariantConst value, uint32_t &timestamp) {
  if (value.is<const char *>()) {
    return parseAeroApiTimestamp(value.as<const char *>(), timestamp);
  }
  return readUnixTimestamp(value, timestamp);
}

String encodeUrlPathSegment(const String &value) {
  static const char HEX_DIGITS[] = "0123456789ABCDEF";
  String encoded;
  for (size_t i = 0; i < value.length(); i++) {
    uint8_t ch = static_cast<uint8_t>(value[i]);
    if ((ch >= 'A' && ch <= 'Z') || (ch >= 'a' && ch <= 'z') ||
        (ch >= '0' && ch <= '9') || ch == '-' || ch == '_' ||
        ch == '.' || ch == '~') {
      encoded += static_cast<char>(ch);
    } else {
      encoded += '%';
      encoded += HEX_DIGITS[ch >> 4];
      encoded += HEX_DIGITS[ch & 0x0F];
    }
  }
  return encoded;
}

bool formatAeroApiWindowTime(uint32_t timestamp, char *buffer,
                             size_t bufferSize) {
  time_t utcTimestamp = static_cast<time_t>(timestamp);
  struct tm utcTime;
  return gmtime_r(&utcTimestamp, &utcTime) != nullptr &&
         strftime(buffer, bufferSize, "%Y-%m-%dT%H:%M:%SZ", &utcTime) > 0;
}

bool flightApiValueIsPresent(JsonVariantConst value) {
  if (value.is<const char *>()) {
    const char *text = value.as<const char *>();
    return text != nullptr && text[0] != '\0';
  }
  return !value.isNull();
}

bool fetchAeroApiFlightDetails(const String &flightNumber) {
  if (config.aeroApiKey.isEmpty() || WiFi.status() != WL_CONNECTED) {
    return false;
  }

  char startTime[24];
  char endTime[24];
  if (!formatAeroApiWindowTime(flightPlanStartValidity, startTime,
                               sizeof(startTime)) ||
      !formatAeroApiWindowTime(flightPlanEndValidity, endTime,
                               sizeof(endTime))) {
    Serial.println("AeroAPI lookup skipped: invalid flight-plan window");
    return false;
  }

  JsonDocument response;
  auto fetchFlights = [&](bool includeScheduleWindow) {
    String url = config.aeroApiURL;
    if (!url.endsWith("/")) {
      url += "/";
    }
    url += "flights/";
    url += encodeUrlPathSegment(flightNumber);
    if (includeScheduleWindow) {
      url += "?start=";
      url += startTime;
      url += "&end=";
      url += endTime;
    }

    HTTPClient http;
    http.setTimeout(10000);
    if (!http.begin(url)) {
      Serial.println("AeroAPI request could not be initialized");
      return false;
    }
    http.addHeader("x-apikey", config.aeroApiKey);
    int statusCode = http.GET();
    if (statusCode != HTTP_CODE_OK) {
      Serial.printf("AeroAPI flight lookup failed: HTTP %d\n", statusCode);
      http.end();
      return false;
    }

    String payload = http.getString();
    http.end();
    DeserializationError error = deserializeJson(response, payload);
    payload = "";
    if (error) {
      Serial.printf("AeroAPI flight response parse failed: %s\n",
                    error.c_str());
      return false;
    }
    return true;
  };

  if (!fetchFlights(true)) {
    return false;
  }
  JsonArray flights = response["flights"].as<JsonArray>();
  if (flights.isNull() || flights.size() == 0) {
    Serial.printf(
        "AeroAPI returned no flights for %s in the schedule window; retrying without date filters\n",
        flightNumber.c_str());
    if (!fetchFlights(false)) {
      return false;
    }
    flights = response["flights"].as<JsonArray>();
  }
  if (flights.isNull() || flights.size() == 0) {
    Serial.printf("AeroAPI found no itinerary data for %s\n",
                  flightNumber.c_str());
    return true;
  }

  JsonObjectConst flight;
  uint64_t bestMatchScore = UINT64_MAX;
  bool bestMatchIsAmbiguous = false;
  const bool hasExpectedOrigin = !flightPlanOrigin.isEmpty();
  const bool hasExpectedDestination = !flightPlanDestination.isEmpty();
  const bool hasExpectedEtd = flightPlanEtd > 0;
  const bool hasExpectedEta = flightPlanEta > 0;
  const uint64_t MISSING_ROUTE_FIELD_PENALTY = 1ULL << 40;
  const uint64_t ROUTE_MISMATCH_PENALTY = 1ULL << 48;
  for (JsonObjectConst candidate : flights) {
    String candidateOrigin = candidate["origin"]["code_iata"] | "";
    if (candidateOrigin.isEmpty()) {
      candidateOrigin = candidate["origin"]["code"] | "";
    }
    String candidateDestination =
        candidate["destination"]["code_iata"] | "";
    if (candidateDestination.isEmpty()) {
      candidateDestination = candidate["destination"]["code"] | "";
    }

    uint64_t missingRouteFields = 0;
    uint64_t routeMismatches = 0;
    if (hasExpectedOrigin && candidateOrigin.isEmpty()) {
      missingRouteFields++;
    } else if (hasExpectedOrigin &&
               !candidateOrigin.equalsIgnoreCase(flightPlanOrigin)) {
      routeMismatches++;
    }
    if (hasExpectedDestination && candidateDestination.isEmpty()) {
      missingRouteFields++;
    } else if (hasExpectedDestination &&
               !candidateDestination.equalsIgnoreCase(flightPlanDestination)) {
      routeMismatches++;
    }
    uint64_t matchScore =
        missingRouteFields * MISSING_ROUTE_FIELD_PENALTY +
        routeMismatches * ROUTE_MISMATCH_PENALTY;

    uint32_t candidateTime = 0;
    if (hasExpectedEtd &&
        (readAeroApiTimestamp(candidate["scheduled_out"], candidateTime) ||
         readAeroApiTimestamp(candidate["estimated_out"], candidateTime))) {
      uint32_t difference = candidateTime > flightPlanEtd
                                ? candidateTime - flightPlanEtd
                                : flightPlanEtd - candidateTime;
      matchScore += difference;
    } else if (hasExpectedEtd) {
      matchScore += 12ULL * 60 * 60;
    }
    candidateTime = 0;
    if (hasExpectedEta &&
        (readAeroApiTimestamp(candidate["scheduled_in"], candidateTime) ||
         readAeroApiTimestamp(candidate["estimated_in"], candidateTime))) {
      uint32_t difference = candidateTime > flightPlanEta
                                ? candidateTime - flightPlanEta
                                : flightPlanEta - candidateTime;
      matchScore += difference;
    } else if (hasExpectedEta) {
      matchScore += 12ULL * 60 * 60;
    }

    if (flight.isNull() || matchScore < bestMatchScore) {
      flight = candidate;
      bestMatchScore = matchScore;
      bestMatchIsAmbiguous = false;
    } else if (matchScore == bestMatchScore) {
      bestMatchIsAmbiguous = true;
    }
  }
  if (flight.isNull() || bestMatchIsAmbiguous) {
    Serial.printf("AeroAPI flight match for %s was %s\n",
                  flightNumber.c_str(),
                  flight.isNull() ? "not found" : "ambiguous");
    return true;
  }

  String trackingCallsign = flight["ident_icao"] | "";
  trackingCallsign.trim();
  bool validTrackingCallsign =
      !trackingCallsign.isEmpty() && trackingCallsign.length() <= 24;
  for (size_t index = 0;
       validTrackingCallsign && index < trackingCallsign.length(); index++) {
    char character = trackingCallsign[index];
    validTrackingCallsign =
        (character >= 'A' && character <= 'Z') ||
        (character >= 'a' && character <= 'z') ||
        (character >= '0' && character <= '9');
  }
  if (validTrackingCallsign) {
    flightPlanTrackingCallsign = trackingCallsign;
    flightPlanTrackingCallsignResolved = true;
    Serial.printf("AeroAPI tracking callsign for %s: %s\n",
                  flightNumber.c_str(), trackingCallsign.c_str());
  } else {
    Serial.printf("AeroAPI has no valid ICAO callsign for %s\n",
                  flightNumber.c_str());
  }

  String origin = flight["origin"]["code_iata"] | "";
  if (origin.isEmpty()) {
    origin = flight["origin"]["code"] | "";
  }
  String destination = flight["destination"]["code_iata"] | "";
  if (destination.isEmpty()) {
    destination = flight["destination"]["code"] | "";
  }
  if (!origin.isEmpty()) {
    flightPlanOrigin = origin;
  }
  if (!destination.isEmpty()) {
    flightPlanDestination = destination;
  }

  uint32_t timestamp = 0;
  if (readAeroApiTimestamp(flight["estimated_out"], timestamp) ||
      readAeroApiTimestamp(flight["scheduled_out"], timestamp)) {
    flightPlanEtd = timestamp;
  }
  uint32_t scheduledEta = 0;
  if (readAeroApiTimestamp(flight["scheduled_in"], scheduledEta)) {
    flightPlanScheduledEta = scheduledEta;
  }
  uint32_t estimatedEta = 0;
  if (readAeroApiTimestamp(flight["estimated_in"], estimatedEta)) {
    flightPlanEstimatedEta = estimatedEta;
    flightPlanEta = estimatedEta;
  } else if (flightPlanScheduledEta > 0) {
    flightPlanEta = flightPlanScheduledEta;
  }

  flightPlanAeroStatus = flight["status"] | "";
  String normalizedStatus = flightPlanAeroStatus;
  normalizedStatus.toLowerCase();
  flightPlanAeroApiLanded =
      flightApiValueIsPresent(flight["actual_in"]) ||
      flightApiValueIsPresent(flight["actual_on"]) ||
      normalizedStatus.startsWith("arrived") ||
      normalizedStatus.startsWith("landed");
  flightPlanDisplayDirty = true;
  Serial.printf("AeroAPI itinerary updated for %s (%s>%s, status=%s)\n",
                flightNumber.c_str(), flightPlanOrigin.c_str(),
                flightPlanDestination.c_str(),
                flightPlanAeroStatus.isEmpty()
                    ? "unknown"
                    : flightPlanAeroStatus.c_str());
  return true;
}

bool fetchFlightPlanLocation(float latitude, float longitude) {
  if (!isfinite(latitude) || !isfinite(longitude) || latitude < -90.0f ||
      latitude > 90.0f || longitude < -180.0f || longitude > 180.0f ||
      WiFi.status() != WL_CONNECTED) {
    return false;
  }

  unsigned long nowMs = millis();
  if (flightPlanLastLocationLookupMs != 0 &&
      nowMs - flightPlanLastLocationLookupMs <
          FLIGHT_PLAN_LOCATION_LOOKUP_MS) {
    return false;
  }
  flightPlanLastLocationLookupMs = nowMs;

  String url = "https://nominatim.openstreetmap.org/reverse?format=jsonv2&zoom=10&addressdetails=1&lat=";
  url += String(latitude, 2);
  url += "&lon=";
  url += String(longitude, 2);

  HTTPClient http;
  http.setTimeout(5000);
  if (!http.begin(url)) {
    Serial.println("OpenStreetMap reverse-geocoding request could not be initialized");
    return false;
  }
  http.setUserAgent("ESP32-HUB75-Flight-Radar/1.0");
  http.addHeader("Accept", "application/json");
  int statusCode = http.GET();
  if (statusCode != HTTP_CODE_OK) {
    Serial.printf("OpenStreetMap reverse-geocoding failed: HTTP %d\n",
                  statusCode);
    http.end();
    return false;
  }

  String payload = http.getString();
  http.end();
  JsonDocument response;
  DeserializationError error = deserializeJson(response, payload);
  payload = "";
  if (error) {
    Serial.printf("OpenStreetMap response parse failed: %s\n", error.c_str());
    return false;
  }

  JsonObjectConst address = response["address"].as<JsonObjectConst>();
  const char *place = address["city"] | "";
  if (place[0] == '\0') place = address["town"] | "";
  if (place[0] == '\0') place = address["village"] | "";
  if (place[0] == '\0') place = address["hamlet"] | "";
  if (place[0] == '\0') place = address["suburb"] | "";
  if (place[0] == '\0') place = address["municipality"] | "";
  if (place[0] == '\0') place = address["county"] | "";

  String location = place;
  const char *state = address["state"] | "";
  if (location.isEmpty()) {
    location = response["display_name"] | "";
  } else if (state[0] != '\0' && location != state) {
    location += ", ";
    location += state;
  }
  if (location.isEmpty()) {
    Serial.println("OpenStreetMap response contained no location name");
    return false;
  }

  flightPlanLocation = location;
  flightPlanDisplayDirty = true;
  Serial.printf("Flight location updated: %s\n", flightPlanLocation.c_str());
  return true;
}

bool fetchFlightPlanAircraft(const String &flightNumber) {
  flightPlanHasLiveData = false;
  if (WiFi.status() != WL_CONNECTED) {
    return false;
  }

  String url = "https://api.adsb.lol/v2/callsign/";
  url += encodeUrlPathSegment(flightNumber);
  HTTPClient http;
  http.setTimeout(10000);
  if (!http.begin(url)) {
    Serial.println("ADSB.lol flight lookup could not be initialized");
    return false;
  }
  http.addHeader("Accept", "application/json");
  int statusCode = http.GET();
  if (statusCode != HTTP_CODE_OK) {
    Serial.printf("ADSB.lol callsign lookup failed: HTTP %d\n", statusCode);
    http.end();
    return false;
  }

  String payload = http.getString();
  http.end();
  JsonDocument response;
  DeserializationError error = deserializeJson(response, payload);
  payload = "";
  if (error) {
    Serial.printf("ADSB.lol callsign response parse failed: %s\n",
                  error.c_str());
    return false;
  }

  JsonArray aircraft = response["ac"].as<JsonArray>();
  for (JsonObject plane : aircraft) {
    AircraftSnapshot snapshot;
    if (!parsePlaneSnapshot(plane, snapshot, true)) {
      continue;
    }
    String observedCallsign = snapshot.callsign;
    observedCallsign.trim();
    if (!observedCallsign.equalsIgnoreCase(flightNumber)) {
      continue;
    }

    flightPlanAircraft = snapshot;
    flightPlanHasLiveData = true;
    if (snapshot.hasPosition) {
      fetchFlightPlanLocation(snapshot.latitude, snapshot.longitude);
    }
    bool routeRetryDue =
        flightPlanLastRouteLookupMs == 0 ||
        millis() - flightPlanLastRouteLookupMs >= FLIGHT_PLAN_ROUTE_RETRY_MS;
    if (!flightPlanRouteLookupAttempted || routeRetryDue) {
      flightPlanRouteLookupAttempted = true;
      flightPlanLastRouteLookupMs = millis();
      if (fetchRoute(flightNumber)) {
        if (flightPlanOrigin.isEmpty()) {
          flightPlanOrigin = routeOrigin;
        }
        if (flightPlanDestination.isEmpty()) {
          flightPlanDestination = routeDest;
        }
      }
    }
    if (!snapshot.onGround) {
      flightPlanLegObservedAirborne = true;
      flightPlanAdsbLanded = false;
    } else if (flightPlanLegObservedAirborne ||
               (flightPlanEta > 0 &&
                time(nullptr) >= static_cast<time_t>(flightPlanEta))) {
      flightPlanAdsbLanded = true;
    }
    flightPlanDisplayDirty = true;
    Serial.printf("ADSB.lol tracked %s: %s, %d ft, %.0f kt\n",
                  flightNumber.c_str(),
                  snapshot.onGround ? "on ground" : "airborne",
                  snapshot.altitudeFt, snapshot.groundSpeedKts);
    return true;
  }

  flightPlanDisplayDirty = true;
  Serial.printf("ADSB.lol has no live aircraft for %s\n",
                flightNumber.c_str());
  return true;
}

void resetCurrentFlightPlanLeg() {
  flightPlanLegObservedAirborne = false;
  flightPlanHasLiveData = false;
  flightPlanAeroApiLanded = false;
  flightPlanAdsbLanded = false;
  flightPlanLandingHoldActive = false;
  flightPlanLandingHoldStartedMs = 0;
  flightPlanTrackingCallsignResolved = false;
  flightPlanRouteLookupAttempted = false;
  flightPlanLastRouteLookupMs = 0;
  flightPlanEta = 0;
  flightPlanEtd = 0;
  flightPlanScheduledEta = 0;
  flightPlanEstimatedEta = 0;
  flightPlanOrigin = "";
  flightPlanDestination = "";
  flightPlanTrackingCallsign = "";
  flightPlanLocation = "LOCATION UNKNOWN";
  flightPlanAeroStatus = "";
  flightPlanLastPollMs = 0;
  aeroApiLastPollMs = 0;
  flightPlanLastLocationLookupMs = 0;
  flightPlanDisplayCycleStartedMs = millis();
  flightPlanLastBottomRenderMs = 0;
  flightPlanLastPositionPhase = -1;
  flightPlanLastStatsPhase = -1;
  flightPlanDisplayDirty = true;
}

bool advanceFlightPlan() {
  if (flightPlanCurrentIndex + 1 >= flightPlanCount) {
    flightPlanCompleted = true;
    Serial.println("Flight itinerary complete; returning to overhead mode");
    return false;
  }

  flightPlanCurrentIndex++;
  resetCurrentFlightPlanLeg();
  Serial.printf("Flight landed; advancing to itinerary leg %u/%u\n",
                flightPlanCurrentIndex + 1, flightPlanCount);
  return true;
}

void beginFlightPlanLandingHold(const String &flightNumber) {
  if (flightPlanLandingHoldActive) {
    return;
  }
  flightPlanLandingHoldActive = true;
  flightPlanLandingHoldStartedMs = millis();
  flightPlanDisplayDirty = true;
  Serial.printf("Flight %s landed; holding display for at least five minutes\n",
                flightNumber.c_str());
}

void pollFlightPlan() {
  if (flightPlanCompleted || flightPlanCurrentIndex >= flightPlanCount) {
    return;
  }

  if (flightPlanLandingHoldActive) {
    if (millis() - flightPlanLandingHoldStartedMs <
        FLIGHT_PLAN_LANDED_HOLD_MS) {
      return;
    }
    flightPlanLandingHoldActive = false;
    advanceFlightPlan();
    return;
  }

  JsonDocument plan;
  JsonObjectConst flight;
  if (!getCurrentFlightPlanLeg(plan, flight)) {
    Serial.println("Cannot poll flight plan: active leg is invalid");
    return;
  }
  String flightNumber = flight["flight"] | "";
  if (flightNumber.isEmpty()) {
    Serial.println("Cannot poll flight plan: active callsign is empty");
    return;
  }

  if (flightPlanEta == 0) {
    readUnixTimestamp(flight["eta"], flightPlanEta);
  }
  if (flightPlanScheduledEta == 0) {
    readUnixTimestamp(flight["eta"], flightPlanScheduledEta);
  }
  if (flightPlanEtd == 0) {
    readUnixTimestamp(flight["etd"], flightPlanEtd);
  }
  if (flightPlanOrigin.isEmpty()) {
    flightPlanOrigin = flight["origin"] | "";
  }
  if (flightPlanDestination.isEmpty()) {
    flightPlanDestination = flight["destination"] | "";
  }

  String trackingCallsign =
      flightPlanTrackingCallsign.isEmpty() ? flightNumber
                                           : flightPlanTrackingCallsign;
  fetchFlightPlanAircraft(trackingCallsign);
  flightPlanLastPollMs = millis();

  if (flightPlanHasLiveData && !flightPlanTrackingCallsignResolved) {
    flightPlanTrackingCallsign = trackingCallsign;
    flightPlanTrackingCallsignResolved = true;
    Serial.printf("Using active ADS-B callsign %s; skipping AeroAPI lookup\n",
                  trackingCallsign.c_str());
  }

  if (flightPlanAeroApiLanded || flightPlanAdsbLanded) {
    beginFlightPlanLandingHold(flightNumber);
    return;
  }

  bool routeMissing = flightPlanOrigin.isEmpty() ||
                      flightPlanDestination.isEmpty();
  bool routeLookupCouldNotProvideRoute =
      routeMissing &&
      (flightPlanRouteLookupAttempted || !flightPlanHasLiveData);
  bool etaMissing = flightPlanEta == 0;
  bool aeroApiDataNeeded = !flightPlanTrackingCallsignResolved ||
                           routeLookupCouldNotProvideRoute || etaMissing;
  unsigned long nowMs = millis();
  if (!flightPlanHasLiveData && !config.aeroApiKey.isEmpty() &&
      aeroApiDataNeeded &&
      (aeroApiLastPollMs == 0 ||
       nowMs - aeroApiLastPollMs >= AEROAPI_RETRY_INTERVAL_MS)) {
    Serial.printf("AeroAPI lookup needed for %s%s%s%s\n",
                  flightNumber.c_str(),
                  flightPlanTrackingCallsignResolved ? "" : ": tracking callsign",
                  etaMissing ? (flightPlanTrackingCallsignResolved
                                    ? ": ETA missing"
                                    : ", ETA missing")
                             : "",
                  routeLookupCouldNotProvideRoute
                      ? ((etaMissing || !flightPlanTrackingCallsignResolved)
                             ? ", route lookup incomplete"
                             : ": route lookup incomplete")
                      : "");
    fetchAeroApiFlightDetails(flightNumber);
    aeroApiLastPollMs = millis();
  }

  time_t now = time(nullptr);
  bool etaFallback = flightPlanEta > 0 &&
                     now >= static_cast<time_t>(flightPlanEta) +
                                FLIGHT_PLAN_ETA_GRACE_SECONDS &&
                     (!flightPlanHasLiveData || flightPlanAircraft.onGround);
  if (flightPlanAeroApiLanded || flightPlanAdsbLanded || etaFallback) {
    beginFlightPlanLandingHold(flightNumber);
  }
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
  url += String(config.latitude, 6);
  url += "/lon/";
  url += String(config.longitude, 6);
  url += "/dist/";
  const int radiusKm = static_cast<int>(lround(config.searchRadius * 1.852));
  url += String(radiusKm);

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
                trackedAircraftCount,
                static_cast<int>(lround(config.searchRadius * 1.852)));
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
  showCallsignInTypeSlot = false;
  callsignTypeSwapPending = false;

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
    resetAircraftDisplayTimer();
    if (REFRESH_INTERVAL_MS > 1000) {
      typeSlotStartedMs = millis();
      callsignTypeSwapPending = true;
    }
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

  if (lastApiHttpCode < 0 &&
      consecutiveFetchFailures >= API_ERROR_FAIL_THRESHOLD &&
      !configAccessPointActive) {
    Serial.println(
        "Repeated network request failures; starting configuration access point");
    startConfigAccessPoint();
  }

  if (!configAccessPointActive && !hasFlightData &&
      consecutiveFetchFailures >= API_ERROR_FAIL_THRESHOLD) {
    showApiError();
    return;
  }

  Serial.printf("Flight refresh failed (%d in a row); keeping last display\n",
                consecutiveFetchFailures);
}

bool isQuietHours() {
  time_t now = time(nullptr);
  if (now < VALID_SYSTEM_TIME_EPOCH) {
    return false;
  }
  struct tm timeinfo;
  if (localtime_r(&now, &timeinfo) == nullptr) {
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

bool beginTimeSync(uint32_t waitMs) {
  configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, "pool.ntp.org", "time.nist.gov");
  lastNtpAttemptMs = millis();
  ntpAttemptMade = true;

  struct tm timeinfo;
  if (getLocalTime(&timeinfo, waitMs) &&
      time(nullptr) >= VALID_SYSTEM_TIME_EPOCH) {
    ntpTimeReceived = true;
    Serial.printf("Time synced: %02d:%02d local (quiet=%s)\n",
                  timeinfo.tm_hour, timeinfo.tm_min,
                  isQuietHours() ? "yes" : "no");
    return true;
  }

  ntpTimeReceived = false;
  Serial.printf("NTP time not available; retrying in %lu seconds\n",
                NTP_RETRY_INTERVAL_MS / 1000);
  return false;
}

void updateTimeSync() {
  if (ntpTimeReceived) {
    return;
  }
  if (time(nullptr) >= VALID_SYSTEM_TIME_EPOCH) {
    ntpTimeReceived = true;
    Serial.println("NTP time received");
    return;
  }
  if (WiFi.status() != WL_CONNECTED ||
      (ntpAttemptMade &&
       millis() - lastNtpAttemptMs < NTP_RETRY_INTERVAL_MS)) {
    return;
  }

  Serial.println("Retrying NTP time synchronization");
  beginTimeSync(NTP_RETRY_WAIT_MS);
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
  //WiFi.disconnect(true)
  //delay(100);
  WiFi.begin(config.wifiSSID.c_str(), config.wifiPassword.c_str());

  showStatus("WiFi...", config.wifiSSID.c_str());

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < timeoutMs) {
    delay(250);
    Serial.printf("WiFi status: %s\n", wifiStatusText(WiFi.status()));
  }

  if (WiFi.status() == WL_CONNECTED) {
    wifiEverConnected = true;
    Serial.print("WiFi connected: ");
    Serial.println(WiFi.localIP());
    /*Added for debug purposes:
    Serial.printf("DNS server 1: %s\n", WiFi.dnsIP(0).toString().c_str());
    Serial.printf("DNS server 2: %s\n", WiFi.dnsIP(1).toString().c_str());

    IPAddress resolved;
    int result = WiFi.hostByName("api.adsb.lol", resolved);
    Serial.printf("api.adsb.lol lookup: %s",
                  result == 1 ? resolved.toString().c_str() : "FAILED");
    Serial.println();

    result = WiFi.hostByName("example.com", resolved);
    Serial.printf("example.com lookup: %s",
                  result == 1 ? resolved.toString().c_str() : "FAILED");
    Serial.println();
    */ //End "Added for debug purposes"
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

  if (!LittleFS.begin(false)) {
    Serial.println("LittleFS mount failed; web configuration page unavailable");
  }
  loadConfiguration();
  setupModeIndicators();
  setupDisplay();

#if COLOR_TEST_ON_BOOT
  runColorTest();
#endif

  showStatus("ESP32", "Flight Radar");

  if (!connectWiFi(30000)) {
    if (startConfigAccessPoint()) {
      setupConfigServer();
    }
    return;
  }

  startFlightMdns();
  setupConfigServer();
  beginTimeSync(NTP_INITIAL_WAIT_MS);
  startMqttClient();

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
  if (WiFi.status() == WL_CONNECTED || configAccessPointActive) {
    configServer.handleClient();
  }
  updateConfigWifiChange();
  updateConfigAccessPoint();
  updateTimeSync();
  updateOperatingModeFromFlightPlan();

  if (configAccessPointActive) {
    delay(40);
    return;
  }

  if (configSuccessScreenActive) {
    unsigned long screenDuration = configSuccessIpScreenActive
                                       ? CONFIG_SUCCESS_IP_SCREEN_MS
                                       : CONFIG_SUCCESS_SSID_SCREEN_MS;
    if (millis() - configSuccessScreenStartedMs >= screenDuration &&
        !configSuccessIpScreenActive) {
      configSuccessIpScreenActive = true;
      configSuccessScreenStartedMs = millis();
      showStatus("WiFi OK", WiFi.localIP().toString().c_str());
      delay(40);
      return;
    }

    if (configSuccessIpScreenActive &&
        millis() - configSuccessScreenStartedMs >= screenDuration) {
      configSuccessScreenActive = false;
      configSuccessIpScreenActive = false;
      if (operatingMode == MODE_FLIGHT_PLAN) {
        showFlight();
      } else if (trackedAircraftCount > 0) {
        showFlight();
      } else {
        screensaverActive = false;
        initQuietHours();
        updateQuietHoursDisplay();
      }
    } else {
      delay(40);
      return;
    }
  }

  if (configWifiChangePending) {
    delay(40);
    return;
  }

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

  updateMqttClient();

  static OperatingMode lastDisplayMode = MODE_OVERHEAD_FLIGHTS;
  if (operatingMode != lastDisplayMode) {
    if (operatingMode == MODE_FLIGHT_PLAN) {
      flightPlanCurrentIndex = 0;
      flightPlanCompleted = false;
      resetCurrentFlightPlanLeg();
      flightPlanDisplayDirty = true;
      screensaverActive = false;
    } else if (trackedAircraftCount > 0) {
      showFlight();
    } else {
      screensaverActive = false;
    }
    lastDisplayMode = operatingMode;
  }

  if (updateDisplayNotice()) {
    delay(40);
    return;
  }

  if (operatingMode == MODE_FLIGHT_PLAN) {
    if (flightPlanLastPollMs == 0 ||
        millis() - flightPlanLastPollMs >= FLIGHT_PLAN_POLL_INTERVAL_MS) {
      pollFlightPlan();
      updateOperatingModeFromFlightPlan();
    }
    if (operatingMode == MODE_FLIGHT_PLAN && flightPlanDisplayDirty) {
      drawFlightPlanDisplay();
      flightPlanDisplayDirty = false;
    }
    if (operatingMode == MODE_FLIGHT_PLAN) {
      updateFlightPlanDisplayAlternation();
    }
    if (operatingMode == MODE_FLIGHT_PLAN) {
      delay(40);
      return;
    }
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

  if (configAccessPointActive) {
    delay(40);
    return;
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
      updateAircraftTypeSlotIfNeeded();
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
