#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <ESP32-HUB75-MatrixPanel-I2S-DMA.h>

#include "config.h"

MatrixPanel_I2S_DMA *display = nullptr;

unsigned long lastFetchMs = 0;
String nearestCallsign = "----";
int nearestAltitude = 0;
float nearestDistanceKm = 0.0;

void showStatus(const char *line1, const char *line2) {
  display->fillScreen(display->color444(0, 0, 0));
  display->setTextSize(1);
  display->setTextWrap(false);
  display->setTextColor(display->color444(0, 15, 15));
  display->setCursor(2, 2);
  display->print(line1);
  display->setCursor(2, 12);
  display->print(line2);
}

void showFlight() {
  display->fillScreen(display->color444(0, 0, 0));
  display->setTextSize(1);
  display->setTextWrap(false);

  display->setTextColor(display->color444(15, 15, 0));
  display->setCursor(2, 2);
  display->print("NEAREST FLIGHT");

  display->setTextColor(display->color444(0, 15, 15));
  display->setCursor(2, 12);
  display->print(nearestCallsign);

  display->setTextColor(display->color444(15, 15, 15));
  display->setCursor(2, 22);
  display->print("ALT ");
  display->print(nearestAltitude);
  display->print("ft  ");
  display->print(nearestDistanceKm, 1);
  display->print("km");
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
    Serial.printf("API request failed: %d\n", statusCode);
    http.end();
    return false;
  }

  String payload = http.getString();
  http.end();

  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, payload);
  if (error) {
    Serial.printf("JSON parse failed: %s\n", error.c_str());
    return false;
  }

  JsonArray aircraft = doc["ac"].as<JsonArray>();
  if (aircraft.isNull() || aircraft.size() == 0) {
    nearestCallsign = "NONE";
    nearestAltitude = 0;
    nearestDistanceKm = 0.0;
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

  nearestAltitude = plane["alt_baro"] | plane["alt_geom"] | 0;
  nearestDistanceKm = plane["dst"] | 0.0;

  return true;
}

void setupDisplay() {
  HUB75_I2S_CFG mxconfig(PANEL_RES_X, PANEL_RES_Y, PANEL_CHAIN);
  display = new MatrixPanel_I2S_DMA(mxconfig);
  display->begin();
  display->setBrightness8(64);
  display->clearScreen();
}

void setup() {
  Serial.begin(115200);
  delay(500);

  setupDisplay();
  showStatus("ESP32", "Flight Radar");

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 20000) {
    delay(250);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi connected: ");
    Serial.println(WiFi.localIP());
    showStatus("WiFi OK", WiFi.localIP().toString().c_str());
    delay(1500);
  } else {
    showStatus("WiFi FAILED", "Check config.h");
    return;
  }

  if (fetchNearestFlight()) {
    showFlight();
    lastFetchMs = millis();
  } else {
    showStatus("API ERROR", "Retrying...");
  }
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    showStatus("WiFi LOST", "Reconnecting");
    WiFi.reconnect();
    delay(1000);
    return;
  }

  if (millis() - lastFetchMs >= REFRESH_INTERVAL_MS) {
    if (fetchNearestFlight()) {
      showFlight();
    } else {
      showStatus("API ERROR", "Will retry");
    }
    lastFetchMs = millis();
  }
}
