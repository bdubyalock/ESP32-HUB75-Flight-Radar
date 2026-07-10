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
   cp config.example.h config.h
   ```

3. Edit `config.h` with your Wi-Fi credentials and receiver location.
4. Select your ESP32 board and port under **Tools**.
5. Open and upload `ESP32-HUB75-Flight-Radar.ino` (folder name must match the sketch name).

On boot, the display connects to Wi-Fi, queries the nearest aircraft within your configured radius, and refreshes on a schedule that stays within your daily API budget. Whenever aircraft are in range it shows them; when none are returned it falls back to a radar screensaver with a rotating sweep and "NO TRAFFIC" text. During quiet hours (10 PM–6 AM Eastern by default) the display is dimmed to `QUIET_BRIGHTNESS`, but polling continues so live traffic is still shown if present.

## Configuration

All settings live in `config.h`:

| Setting | Description |
| --- | --- |
| `WIFI_SSID` / `WIFI_PASSWORD` | Network credentials |
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
