# E-GoCart VCU Firmware

Vehicle Control Unit (VCU) firmware for an electric go-kart, built on ESP32-S3 with FreeRTOS. Manages motor control, battery management, charging, telemetry, and safety systems over dual CAN buses.

---

## Hardware

| Component | Details |
|-----------|---------|
| MCU | ESP32-S3 @ 240 MHz (dual-core) |
| CAN1 | MCP2515 SPI, 500 kbps — Motor inverter (DMC) & charger (NLG5) |
| CAN2 | ESP32 TWAI, 250 kbps — Battery management system (BMS) |
| Display | Nextion HMI (UART) |
| I/O Expander | MCP23017 (I2C) |
| ADC | ADS1115 — throttle, regen pedal |
| Battery | 104S LiPo, ~312 V nominal, 16 Ah |
| Motor | Configured for 250 Nm fwd / 100 Nm regen / 60 Nm reverse |

## Battery — Racepow RP-SSB-1174170

| Parameter | Value |
|-----------|-------|
| Model | Racepow RP-SSB-1174170 |
| Configuration | 104S LiPo (7× 14S + 1× 6S in series) |
| Nominal Voltage | ~385 V system (104 × 3.7 V) |
| Charge Cut-off | ~436.8 V (104 × 4.2 V) |
| Discharge Cut-off | ~312 V (104 × 3.0 V) |
| Capacity | 16 Ah |
| Charge Current | 8 A (0.5C) |
| Continuous Discharge | 400 A (25C) |
| Max Discharge | 480 A (30C) |
| Charge Temp | 0 °C – 45 °C |
| Discharge Temp | −20 °C – 55 °C |
| Storage Temp | −20 °C – 35 °C (alle 3 Monate nachladen) |

## Software Architecture

12+ FreeRTOS tasks split across two cores:

**Core 0 — Real-time / safety-critical**
- `CANRx` (P24) — CAN message ingestion
- `CANTx` (P23) — Motor torque commands at 10–50 ms
- `VehicleControl` (P22) — Throttle/regen → torque demand
- `StateManager` (P21) — State machine (Sleep → Init → Ready → Drive/Charging)
- `SafetyMonitor` (P20) — Temperature, voltage, interlock watchdog

**Core 1 — UI / networking**
- `InputManager` (P10) — Buttons, switches, ADC reads
- `DisplayManager` (P5) — Nextion HMI updates
- `WiFiManager` (P4) — AP mode + optional STA
- `WebServer` (P3) — HTTP REST + WebSocket telemetry (50 ms)
- `TaskMonitor` (P2) — Per-task watchdog & auto-restart
- `LEDControl` (P3) — Status LED patterns
- `ErrorAggregator` (P4) — Cross-subsystem error collection

## Vehicle State Machine

```
SLEEP ──(start/charger)──> INIT ──(precharge ok)──> READY
                                                      │
                                          ┌──────────┴──────────┐
                                        DRIVE              CHARGING
```

## WiFi & Web Interface

- **SSID:** `E-GoCart-VCU` | **Password:** `egocart123`
- **IP:** `10.1.109.191`
- Real-time WebSocket telemetry: speed, power, SOC, temps, errors
- OTA firmware update endpoint
- JSON configuration API

## Building & Flashing

Requires [PlatformIO](https://platformio.org/).

```bash
# Build
pio run

# Flash
pio run --target upload

# Monitor serial
pio device monitor
```

OTA updates are also supported via the web interface at `http://10.1.109.191`.

## Project Structure

```
E-GoCart/
├── src/main.cpp          # Main firmware entry point & all FreeRTOS tasks
├── include/
│   ├── config.h          # Pin map, CAN IDs, safety limits, tuning params
│   ├── data_structures.h # Shared state structs
│   ├── can_manager.h     # Dual CAN bus abstraction
│   ├── bms_manager.h     # Battery management
│   ├── nlg5_manager.h    # NLG5 onboard charger
│   ├── vehicle_control.h # Torque calculation
│   ├── state_manager.h   # State machine
│   ├── contactor_manager.h
│   ├── display_manager.h
│   ├── input_manager.h
│   ├── wifi_manager.h
│   ├── web_server.h
│   └── task_monitor.h
├── data/www/             # Web UI static files (LittleFS)
├── platformio.ini        # Build configuration
└── PinOut.csv            # GPIO pin reference
```

## Safety Features

- Emergency interlock kill (hardware switch)
- Motor/inverter/battery overtemperature shutdown
- BMS communication loss → immediate torque cut
- Precharge sequencing before main contactor closure
- Current verification before contactor switching
- Per-task watchdogs with auto-restart

---

## Releases

| Version | Date | Notes |
|---------|------|-------|
| v1.0.0 | 2026-03-31 | Initial release — dual-CAN VCU, FreeRTOS multi-task, WiFi telemetry, Nextion display, deep sleep, OTA |
