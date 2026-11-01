#pragma once

#include <Arduino.h>

// Settings that can be changed through the web interface.
// Values are loaded from config.h, then overridden by ESP32 Preferences.
struct AppConfig {
    // Network
    String wifiSSID;
    String wifiPassword;

    // AeroAPI
    String aeroApiKey;
    String aeroApiURL;

    // MQTT
    String mqttIP;
    uint16_t mqttPort = 60883;
    String mqttTopic;
    String mqttDataName;

    // Receiver location
    double latitude = 40.747384;
    double longitude = -73.957026;

    // Search radius in nautical miles
    double searchRadius = 5.4;
};

// Shared configuration object, defined in one .cpp file.
extern AppConfig config;

// Load compile-time defaults, then override them with values saved in Preferences.
void loadConfiguration();
bool saveConfiguration();