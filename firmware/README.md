# Firmware

All sketches target the **DFRobot FireBeetle 2 ESP32-S3 with Camera (N16R8)**. Each sketch sits in a folder with the same name, so the Arduino IDE opens it directly.

| Sketch | What it is | Talks to |
|---|---|---|
| [`SchoolBus/`](SchoolBus/) | **Final firmware.** Line following + gyro PID, obstacle handling, IR remote, LEDs, LCD, sensors, GPS, on-device sign detection, debug web page | Browser at `http://esp32-data.local/` (`/capture` raw 96x96 frame, `/data` JSON with the last detection) |
| [`TelemetryLink/`](TelemetryLink/) | Link to the SafeRoute app: VGA JPEG stream and JSON telemetry. Sensor events are simulated from the serial monitor (`smoke`, `flame`, `vibration`, `arrival`, `departure`, `busDelay`, `speedLimit`, `update`, `reset`) | SafeRoute app: `http://esp32s3-data.local/stream` (MJPEG) and `ws://esp32s3-data.local:81` (JSON every 2 s) |
| [`experiments/CameraLineDetection/`](experiments/CameraLineDetection/) | First line-following attempt with the camera (thresholding + left / right pixel balance). Replaced by IR sensors | Browser |
| [`experiments/EspDspBenchmark/`](experiments/EspDspBenchmark/) | Benchmark of a 4096-element dot product, plain C++ vs. the ESP-DSP assembly routines, for future optimisation | Serial monitor |
| [`archive/`](archive/) | Earlier development versions, kept for reference (table below) | |
| [`libraries/a0912_inferencing/`](libraries/a0912_inferencing/) | The Edge Impulse Arduino export of the sign detector, with the PSRAM patch described in [`ml/README.md`](../ml/README.md#library-patch) | |

## Build settings

- Board package: DFRobot FireBeetle 2 ESP32-S3 (the sketches use its `D2`, `D3`, `D6`, `D7`, `D11`, `D12`, `A4` pin aliases)
- **PSRAM enabled** (the camera frame buffer and the model's tensor arena are allocated there)
- Copy `libraries/a0912_inferencing` into your Arduino `libraries` folder (`Documents/Arduino/libraries` on Windows)
- Libraries for `SchoolBus`: DFRobot_AXP313A, TinyGPSPlus, Adafruit MCP23017, LiquidCrystal_I2C, IRremote (v3+ API, `IrReceiver`)
- Extra libraries for `TelemetryLink`: WebSockets (Markus Sattler, `WebSocketsServer.h`), ArduinoJson
- `esp_camera`, `WebServer`, `WiFiMulti`, `ESPmDNS` come with the ESP32 core

When you have a build that works, write the exact library versions here so the result can be reproduced.

## Wi-Fi credentials

`SchoolBus` and `TelemetryLink` read their networks from `secrets.h`:

```
copy secrets.example.h secrets.h      (Windows)
cp secrets.example.h secrets.h        (macOS / Linux)
```

then edit `secrets.h`. It is ignored by git. The sketches in `experiments/` and `archive/` contain `YOUR_SSID` / `YOUR_PASSWORD` placeholders.

## Remote control (IR command codes)

| Code | Manual mode | Automatic mode |
|---|---|---|
| 69 | switch to manual | switch to manual |
| 71 | switch to automatic | switch to automatic |
| 24 | forward | restart straight ahead (new heading reference) |
| 82 | reverse | reverse |
| 8 / 90 | spin left / right | target heading +90 / -90 deg |
| 12, 74 / 94, 66 | no visible effect | target heading +45 / -45 deg |
| 28 | stop | stop |
| 21 / 7 | speed +10 / -10 (range 70-255) | same |

## Earlier versions (`archive/`)

| Folder | Date | Original file | Notes |
|---|---|---|---|
| `v1_SchoolBusBasic` | 2026-01-17 | `School_Bus.ino` | First integration of remote, LCD, MCP23017, GPS and camera; needs a `camera_pins.h` that was never committed |
| `v2_WebServer` | 2026-01-21 | `SchoolBus_withWebServer_withComments` | Adds the MJPEG web stream, WebSocket telemetry and the accelerometer |
| `v3_AiWebServer` | 2026-01-31 | `AI_withWebServer.ino` | First on-device inference with the Edge Impulse model and the raw-frame web page |
| `v4_AutoIrTelemetry` | 2026-02-05 | `auto_ir.ino` | Automatic mode with IR sensors and full app telemetry, before the AI model was merged in |
