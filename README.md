# Babyphone Intelligent ESP32

ESP32 + Wokwi + Blynk project for a smart baby room monitor.

The system monitors:

- temperature and humidity with DHT22
- movement with PIR sensor
- simulated noise level with a potentiometer
- baby state on OLED SSD1306
- local alerts with LEDs and buzzer
- remote dashboard and notifications with Blynk

## Features

- Calm sleep, ambient noise, awake, agitation, and thermal alert states
- Priority logic: thermal alert > agitation > awake > ambient noise > calm sleep
- OLED display with temperature, humidity, PIR state, noise percentage, and noise bar graph
- Non-blocking logic using `millis()` and no `delay()`
- Blynk gauges for temperature, humidity, and noise
- Blynk notifications for baby awake/agitated and thermal alerts
- Blynk switch on `V3` to enable/disable the local buzzer
- Two-hour summary with minimum/maximum temperature and wake-up count
- MPU6050 fall detection with highest-priority local and Blynk alert

## Required Software

- VS Code
- Wokwi Simulator extension for VS Code
- Arduino CLI
- ESP32 Arduino core
- Blynk account

## Files To Push To GitHub

Push these files:

- `sketch.ino`
- `diagram.json`
- `wokwi.toml`
- `libraries.txt`
- `config.example.h`
- `README.md`
- `.gitignore`
- `tableau-tests.docx` if you want to include the validation table/report file
- `wokwi-project.txt` if you want to keep Wokwi project metadata

Do not push these:

- `config.h` because it contains private Blynk/Wi-Fi credentials
- `build/` because it contains generated firmware files
- `*.bin`, `*.elf`, `*.map`, `*.hex`

## Blynk Setup

Create a Blynk template:

- Template name: `Babyphone Intelligent`
- Hardware: `ESP32`
- Connection type: `WiFi`

Create these datastreams:

| Name | Pin | Type | Min | Max |
|---|---|---|---:|---:|
| Temperature | `V0` | Double | 0 | 50 |
| Humidity | `V1` | Double | 0 | 100 |
| Noise level | `V2` | Integer | 0 | 100 |
| Buzzer enabled | `V3` | Integer | 0 | 1 |
| Baby state | `V4` | String | - | - |
| Last 2h minimum temperature | `V5` | Double | 0 | 50 |
| Last 2h maximum temperature | `V6` | Double | 0 | 50 |
| Last 2h wake-up count | `V7` | Integer | 0 | 1000 |

Create these Blynk events:

| Event code | Message |
|---|---|
| `bebe_reveille` | Bebe reveille |
| `bebe_agite` | Bebe agite |
| `temperature_basse` | Temperature chambre trop basse |
| `temperature_haute` | Temperature chambre trop haute |
| `chute_detectee` | Chute detectee - intervention immediate |

In the Blynk dashboard, add:

- Gauge linked to `V0`
- Gauge linked to `V1`
- Gauge linked to `V2`
- Chart linked to `V0` for temperature history
- Switch linked to `V3`
- Label linked to `V4`
- Value displays linked to `V5`, `V6`, and `V7` for the last completed two-hour summary

## Test The Two-Hour Summary

The final interval is configured as two hours:

```cpp
const unsigned long SUMMARY_INTERVAL_MS = 2UL * 60UL * 60UL * 1000UL;
```

For a quick Wokwi test, temporarily replace it with `30000` (30 seconds), then restore the two-hour value for the final version.

## MPU6050 Fall Detection

The MPU6050 shares the I2C bus with the OLED:

| MPU6050 | ESP32 |
|---|---|
| VCC | 3V3 |
| GND | GND |
| SDA | GPIO21 |
| SCL | GPIO22 |

The code checks acceleration every 100 ms. A free-fall magnitude below `3 m/s2` or an impact above `20 m/s2` triggers `FALL_ALERT` for at least five seconds.

Fall alert priority is higher than thermal alert, agitation, and awake states. It activates the red LED, continuous buzzer, OLED warning, Serial detail, and the Blynk event `chute_detectee`.

## Local Configuration

Create a local config file from the example:

```powershell
Copy-Item config.example.h config.h
```

Open `config.h` and replace:

```cpp
#define BLYNK_TEMPLATE_ID "YOUR_TEMPLATE_ID"
#define BLYNK_AUTH_TOKEN "YOUR_DEVICE_AUTH_TOKEN"
```

For Wokwi simulation, keep:

```cpp
#define WIFI_NETWORK_NAME "Wokwi-GUEST"
#define WIFI_NETWORK_PASSWORD ""
```

For a real ESP32 device, replace those Wi-Fi values with your real network name and password.

## Install Arduino Dependencies

Run these commands once:

```powershell
arduino-cli core update-index
arduino-cli core install esp32:esp32
arduino-cli lib install "DHT sensor library"
arduino-cli lib install "Adafruit SSD1306"
arduino-cli lib install "Adafruit GFX Library"
arduino-cli lib install "Blynk"
```

## Build For Wokwi VS Code

The `wokwi.toml` file points to the firmware inside `build/`, so compile with:

```powershell
arduino-cli compile --fqbn esp32:esp32:esp32 --output-dir build .
```

Then open the Wokwi Simulator in VS Code and start the simulation.

In the Wokwi terminal, you should see sensor values and Blynk connection messages. In Blynk, the device should become Online.

## Run On Real ESP32

After editing `config.h` with real Wi-Fi credentials, upload with:

```powershell
arduino-cli upload -p COM_PORT --fqbn esp32:esp32:esp32 .
```

Replace `COM_PORT` with your ESP32 serial port, for example `COM3`.

## Validation

Recommended scenarios:

| Scenario | Temperature | PIR | Noise | Expected state |
|---|---:|---|---:|---|
| Calm sleep | 19 C | No | Low | Sommeil calme |
| Awake | 19 C | Yes | Low | Eveil |
| Agitation | 19 C | Yes | High | Agitation |
| Cold alert | 16 C | Any | Any | Alerte thermique |
| Hot alert | 25 C | Any | Any | Alerte thermique |
| Priority test | 25 C | Yes | High | Alerte thermique |
