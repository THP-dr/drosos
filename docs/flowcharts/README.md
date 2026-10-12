# Flowcharts

Activity diagrams of the final firmware ([`firmware/SchoolBus`](../../firmware/SchoolBus/)), one per function, plus an overview poster.

| File | Function |
|---|---|
| `setup.png`, `loop.png` | Start-up sequence and main control loop |
| `globals.png` | Pins, constants, global state and library objects |
| `forward.png`, `backward.png`, `turnLeft.png`, `turnRight.png`, `stop.png` | Motor commands and their LED feedback |
| `automaticMode.png`, `manualMode.png` | Mode switching |
| `pidCorrection.png`, `resetPID.png` | Heading PID |
| `gyroBegin.png`, `calibrateGyro.png`, `readGyro.png`, `readAccelZ.png`, `isPickedUp.png` | MPU6050 |
| `checkLeds.png` | Non-blocking LED blinking |
| `handleIRcommunication.png` | IR remote decoding |
| `handleServerRoot.png`, `handleImageCapture.png`, `handleServerData.png` | Debug web server |
| `convertBufferForAI.png`, `inferenceTask.png` | Sign-detection task |
| `telemetry-link.png`, `telemetry-link_commented.png` | The [`TelemetryLink`](../../firmware/TelemetryLink/) sketch |
| `overview/` | All diagrams on one poster (PNG, PDF and the Photoshop source) |
| `early/` | First diagrams of the earlier firmware versions |

## How they were made

The goal was documentation generated from the code rather than drawn by hand:

1. Paste **one function at a time** into a ChatGPT "C++ to flowchart" assistant and ask for analytical PlantUML activity-diagram code that keeps the function and variable names (the whole program at once gives an unreadable chart).
2. Paste the PlantUML into the [PlantUML editor](https://editor.plantuml.com/) and save the rendered image.
3. If the diagram is wrong, give the PlantUML back to the assistant and iterate.

The diagrams describe the code as it was in February 2026; check them against the sketch if you change it.
