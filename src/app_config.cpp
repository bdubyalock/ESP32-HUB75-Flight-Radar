#include "app_config.h"

#include <ArduinoJson.h>
#include <Preferences.h>

#include "config.h"

AppConfig config;

namespace {
const char *PREFERENCES_NAMESPACE = "flightconfig";
}

void loadConfiguration() {
  config.wifiSSID = WIFI_SSID;
  config.wifiPassword = WIFI_PASSWORD;
  config.aeroApiKey = AEROAPI_KEY;
  config.aeroApiURL = AEROAPI_URL;
  config.mqttIP = MQTT_IP;
  config.mqttPort = MQTT_PORT;
  config.mqttTopic = MQTT_TOPIC;
  config.mqttDataName = MQTT_DATA_NAME;
  config.latitude = RECEIVER_LAT;
  config.longitude = RECEIVER_LON;
  config.searchRadius = TRACK_RADIUS_KM / 1.852;

  Preferences preferences;
  if (!preferences.begin(PREFERENCES_NAMESPACE, true)) {
    Serial.println("Could not open configuration preferences; using config.h defaults");
    return;
  }

  String storedConfig = preferences.getString("settings", "");
  preferences.end();

  if (storedConfig.isEmpty()) {
    return;
  }

  JsonDocument saved;
  DeserializationError error = deserializeJson(saved, storedConfig);
  if (error || !saved.is<JsonObject>()) {
    Serial.printf("Saved configuration is invalid: %s; using config.h defaults\n",
                  error.c_str());
    return;
  }

  JsonObjectConst fields = saved.as<JsonObjectConst>();
  if (fields["wifiSSID"].is<const char *>()) {
    config.wifiSSID = fields["wifiSSID"].as<const char *>();
  }
  if (fields["wifiPassword"].is<const char *>()) {
    config.wifiPassword = fields["wifiPassword"].as<const char *>();
  }
  if (fields["aeroApiKey"].is<const char *>()) {
    config.aeroApiKey = fields["aeroApiKey"].as<const char *>();
  }
  if (fields["aeroApiURL"].is<const char *>()) {
    config.aeroApiURL = fields["aeroApiURL"].as<const char *>();
  }
  if (fields["mqttIP"].is<const char *>()) {
    config.mqttIP = fields["mqttIP"].as<const char *>();
  }
  if (fields["mqttPort"].is<int>()) {
    config.mqttPort = fields["mqttPort"].as<uint16_t>();
  }
  if (fields["mqttTopic"].is<const char *>()) {
    config.mqttTopic = fields["mqttTopic"].as<const char *>();
  }
  if (fields["mqttDataName"].is<const char *>()) {
    config.mqttDataName = fields["mqttDataName"].as<const char *>();
  }
  if (fields["latitude"].is<double>() || fields["latitude"].is<int>()) {
    config.latitude = fields["latitude"].as<double>();
  }
  if (fields["longitude"].is<double>() || fields["longitude"].is<int>()) {
    config.longitude = fields["longitude"].as<double>();
  }
  if (fields["searchRadius"].is<double>() ||
      fields["searchRadius"].is<int>()) {
    config.searchRadius = fields["searchRadius"].as<double>();
  }
}

bool saveConfiguration() {
  Preferences preferences;
  if (!preferences.begin(PREFERENCES_NAMESPACE, false)) {
    Serial.println("Could not open configuration preferences for writing");
    return false;
  }

  JsonDocument document;
  document["wifiSSID"] = config.wifiSSID;
  document["wifiPassword"] = config.wifiPassword;
  document["aeroApiKey"] = config.aeroApiKey;
  document["aeroApiURL"] = config.aeroApiURL;
  document["mqttIP"] = config.mqttIP;
  document["mqttPort"] = config.mqttPort;
  document["mqttTopic"] = config.mqttTopic;
  document["mqttDataName"] = config.mqttDataName;
  document["latitude"] = config.latitude;
  document["longitude"] = config.longitude;
  document["searchRadius"] = config.searchRadius;

  String serialized;
  serializeJson(document, serialized);
  bool saved = !serialized.isEmpty() &&
               preferences.putString("settings", serialized) ==
                   serialized.length();
  preferences.end();

  if (!saved) {
    Serial.println("Failed to persist all configuration values");
  }
  return saved;
}
