# ESP32 Flight Radar

Live nearby aircraft display for ESP32, using an HUB75 LED matrix panel and the free [adsb.lol](https://adsb.lol) API.



https://github.com/user-attachments/assets/66c5b631-293f-468c-ad8d-0535e8d87d64





## Hardware

| Component | Notes |
| --- | --- |
| **ESP32 dev board** | Any ESP32 with Wi-Fi (tested with ESP32 Arduino core 3.3.x) |
| **HUB75 LED matrix** | Default config targets a 64×32 panel; adjust `PANEL_RES_X`, `PANEL_RES_Y`, and `PANEL_CHAIN` in `config.h` |
| **5V power supply** | Size for your panel; many 64×32 panels need a dedicated supply |

### Libraries

Install these through the Arduino Library Manager or clone into your sketchbook `libraries` folder:

- [ESP32_HUB75_LED_MATRIX_PANEL_DMA_Display](https://github.com/mrfaptastic/ESP32-HUB75-MatrixPanel-I2S-DMA)
- [ArduinoJson](https://arduinojson.org/)
- [Adafruit GFX Library](https://github.com/adafruit/Adafruit-GFX-Library) (dependency of the matrix driver)

#### This repository has been updated to use PlatformIO IDE
Use the following platformio.ini file:
```
; PlatformIO Project Configuration File
;
;   Build options: build flags, source filter
;   Upload options: custom upload port, speed and extra flags
;   Library options: dependencies, extra library storages
;   Advanced options: extra scripting
;
; Please visit documentation for the other options and examples
; https://docs.platformio.org/page/projectconf.html

[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino

lib_deps =
    bblanchon/ArduinoJson
    mrfaptastic/ESP32 HUB75 LED MATRIX PANEL DMA Display
    adafruit/Adafruit GFX Library
    adafruit/Adafruit BusIO
```

### Board support

Add the Espressif ESP32 board package in Arduino IDE:

```
https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
```

## Display test (start here)

Before wiring up Wi-Fi or flight data, verify your 64×32 panel works:

1. In Arduino IDE, open `DisplayTest/DisplayTest.ino` (not the main sketch).
2. Select your ESP32 board and port under **Tools**.
3. Upload and watch the panel cycle through color, border, text, and layout tests every 4 seconds.

If the display is blank, garbled, or wrong colors, fix wiring/power/pins before moving on.

## Setup

1. Clone this repo into your Arduino sketchbook or open it directly in Arduino IDE 2.x.
2. Copy the config template:

   ```bash
   cp config.example.h include/config.h
   ```

3. Edit `include/config.h` with your Wi-Fi credentials and receiver location.
4. Select your ESP32 board and port under **Tools**.
5. Open and upload `ESP32-HUB75-Flight-Radar.ino` (folder name must match the sketch name).

On boot, the display connects to Wi-Fi, queries the nearest aircraft within your configured radius, and refreshes on a schedule that stays within your daily API budget. Whenever aircraft are in range it shows them; when none are returned it falls back to a radar screensaver with a rotating sweep and "NO TRAFFIC" text. During quiet hours (10 PM–6 AM Eastern by default) the display is dimmed to `QUIET_BRIGHTNESS`, but polling continues so live traffic is still shown if present.

In flight-plan mode, the top row shows the airline logo and flight number,
with the live aircraft type below. The first lower row alternates every five
seconds between origin/destination and the aircraft's nearby place name; long
text scrolls across the row. Place names use the public OpenStreetMap
Nominatim reverse-geocoding service, queried at most once per minute with
rounded ADS-B coordinates. The final row cycles between speed/altitude,
ETA (or ETD while on the ground), and arrival status. Arrival status compares
estimated and scheduled ETA using a 15-minute early/late threshold, or shows
Delayed when AeroAPI reports it.

Flight plans can be submitted from the FLIGHT ENTRY portal or received from an
MQTT broker over WebSockets. The default configuration subscribes to
`flightradar/your-topic` at `your-mqtt-broker-url-root:60473/mqtt`; configure
the broker host, port, WebSocket path, topic, username, and password in the
CONFIGURATION panel. The username is shown as plain text; the password stays
write-only. Both are stored with the other settings in ESP32 NVS. A
flight-plan message must use the same `flight_data` and `validity` JSON shape
as the portal and may contain up to four consecutive flight entries.
The display confirms changed schedules for five seconds; resubmitting an
unchanged schedule does not interrupt tracking or show another confirmation.
The display also shows a completion notice when schedule tracking returns to
overhead mode.
When an AeroAPI key is configured, each flight-plan leg is resolved
independently using its marketed flight identifier, route, and scheduled
times. The matching AeroAPI result's `ident_icao` supplies the ADS-B callsign,
so the operating carrier can vary from flight to flight without hardcoded
carrier mappings. The schedule format and displayed flight number remain
unchanged. If AeroAPI cannot find one unambiguous result or has no ICAO
identifier, tracking falls back to the scheduled flight number. If the
schedule-window query returns no flights, the device retries the identifier
lookup without date filters and ranks the results using the route and schedule
times.
Each poll first checks ADS-B for the scheduled or already-resolved callsign;
when that flight is present, no AeroAPI request is made. A final leg observed
on the ground after being airborne, or on the ground at/after its ETA, ends
schedule mode even if ADS-B subsequently stops reporting it. After landing is
detected (or the ETA fallback determines the leg is complete), the display
shows `LANDED` for at least five minutes before advancing to the next leg or
returning to overhead mode.
MQTT uses secure WebSockets (`wss://`) with certificate validation against
the Let's Encrypt ISRG Root X2 CA. The device waits for NTP time before
connecting so that broker certificate validity can be checked.

## Configuration

Compile-time settings and board/display options live in `include/config.h`.
The CONFIGURATION form is served by the ESP32 and stores editable settings in
NVS flash using Preferences; those values override the `config.h` defaults on
boot. The FLIGHT ENTRY panel submits flight plans directly to the ESP32.

With PlatformIO, build and upload the firmware:

```bash
pio run
pio run -t upload
```

The standard PlatformIO Upload target (including the VS Code Upload button)
uploads the LittleFS filesystem image first, then the firmware. To upload only
the page filesystem later, run `pio run -t uploadfs`.

After the ESP32 connects to Wi-Fi, open
`http://<MDNS_HOSTNAME>.local/config.html` (default:
`http://flightradar.local/config.html`) or use the device IP address.
`MDNS_HOSTNAME` is set in `include/config.h` without the `.local` suffix.
A changed network name or password is tested for up to 45 seconds before any
settings are saved. Password, AeroAPI key, and MQTT credential fields are
write-only; leave them blank to keep their current values. The configuration
page uses plain HTTP and has no login, so use it only on a trusted local
network.

Previously saved settings in NVS take precedence over `config.h`. If updating
an existing device, set the MQTT address, port, topic, username, and password
in the CONFIGURATION panel; use `3852router.thelostvalley.us`,
`60473`, and `flightradar/brad` for this broker.

If the ESP32 cannot join its saved Wi-Fi network, or repeated network
requests fail, it starts a temporary password-protected setup network. Join
the `Radar-XXXX` network shown on the display, use the displayed password, and
open `http://192.168.4.1/config.html`. The setup network shuts down 30 seconds
after a successful configuration save.

Other settings remain compile-time constants:

| Setting | Description |
| --- | --- |
| `WIFI_SSID` / `WIFI_PASSWORD` | Network credentials |
| `MQTT_IP` / `MQTT_PORT` / `MQTT_WEBSOCKET_PATH` / `MQTT_TOPIC` | MQTT WebSocket broker host, port, path, and subscribed topic |
| `MQTT_USERNAME` / `MQTT_PASSWORD` | Optional compile-time MQTT credentials; they can also be entered in the configuration page |
| `RECEIVER_LAT` / `RECEIVER_LON` | Your location in decimal degrees |
| `SEARCH_RADIUS_KM` | How far to search for aircraft |
| `PANEL_RES_X` / `PANEL_RES_Y` / `PANEL_CHAIN` | Matrix panel geometry |
| `REFRESH_INTERVAL_MS` | Time between API polls during active hours (default: 5 minutes) |
| `QUIET_HOUR_START` / `QUIET_HOUR_END` | Local hours when the display is dimmed (may wrap past midnight) |
| `QUIET_BRIGHTNESS` | Display brightness during quiet hours |
| `GMT_OFFSET_SEC` / `DAYLIGHT_OFFSET_SEC` | Local timezone offset (default: US Eastern) |

## API

This project uses the adsb.lol closest-aircraft endpoint:

```
GET https://api.adsb.lol/v2/closest/{lat}/{lon}/{radius_km}
```

No API key is required. See the [adsb.lol docs](https://adsb.lol/docs) for response fields.

## Roadmap

- [ ] Radar-style map view on the matrix
- [ ] Cycle through multiple nearby aircraft
- [ ] Web configuration UI
- [ ] Aircraft type and route details

## License

MIT
