# E-GoCart VCU - FreeRTOS Version

## Overview

This is a complete FreeRTOS-based Vehicle Control Unit (VCU) for the E-GoCart electric vehicle, running on ESP32-S3 with dual-core task management, real-time CAN communication, and a web-based dashboard.

## Features

### Core Functionality
- ✅ **10 FreeRTOS tasks** across 2 cores for optimal performance
- ✅ **Interrupt-driven CAN** with queue processing (10ms cycle)
- ✅ **Thread-safe data sharing** between tasks
- ✅ **Watchdog monitoring** with automatic task restart
- ✅ **Configurable CAN timing** (10-50ms via web interface)
- ✅ **Proper precharge** with voltage comparison (BMS vs DMC)

### Web Dashboard Features
- ✅ **Real-time telemetry** at 50ms via WebSocket
- ✅ **Live speed and power charts**
- ✅ **Configuration editor** for all parameters
- ✅ **CAN bus monitor** with message logging
- ✅ **Error log display**
- ✅ **Task monitor** showing FreeRTOS task stats
- ✅ **WiFi configuration** (AP + STA modes)
- ✅ **OTA firmware updates**
- ✅ **Adjustable battery parameters**
- ✅ **Safety limit configuration**

## Architecture

### Task Distribution

**Core 0 (Protocol Core) - Time-Critical:**
1. **CAN RX** (Priority 24) - 10ms - Processes incoming CAN messages
2. **CAN TX** (Priority 23) - 10-50ms - Sends DMC control messages
3. **Vehicle Control** (Priority 22) - 10ms - Calculates torque demand
4. **State Manager** (Priority 21) - 50ms - Vehicle state machine
5. **Safety Monitor** (Priority 20) - 100ms - Temperature/voltage checks

**Core 1 (Application Core) - Non-Critical:**
6. **Input Manager** (Priority 10) - 20ms - Reads buttons/sensors
7. **Display Manager** (Priority 5) - 50ms - Updates Nextion display
8. **WiFi Manager** (Priority 4) - 1000ms - WiFi connection management
9. **Webserver** (Priority 3) - 50ms - HTTP/WebSocket server
10. **Task Monitor** (Priority 2) - 1000ms - Watchdog & statistics

## Building & Uploading

### 1. Compile the Firmware

```bash
pio run
```

Firmware size: ~1.05MB

### 2. Upload the Firmware

```bash
pio run --target upload
```

### 3. Upload the Filesystem (Web Dashboard)

The web dashboard files are in `data/www/`. To upload to LittleFS:

```bash
pio run --target uploadfs
```

**Note:** If `uploadfs` target is not available, you may need to add it to `platformio.ini`:

```ini
[env:esp32s3box]
...
board_build.filesystem = littlefs
```

Alternatively, use the ESP32 filesystem uploader tool.

### 4. Monitor Serial Output

```bash
pio device monitor
```

## Accessing the Dashboard

### Access Point Mode (Default)
1. Power on the VCU
2. Connect to WiFi network: **E-GoCart-VCU**
3. Password: **egocart123**
4. Open browser to: **http://192.168.4.1**

### Station Mode (Workshop)
1. Configure WiFi credentials via the web dashboard
2. VCU will connect to your existing WiFi
3. Find IP address in serial monitor or router
4. Access via browser: **http://[vcu-ip-address]**

## Web Dashboard Usage

### Dashboard Tab
- Real-time vehicle telemetry
- Speed, power, SOC, temperatures
- Live charts for speed and power
- Gear indicator (N/D/R)
- Input status (throttle, regen, brake)

### Configuration Tab
Adjust all parameters:
- **CAN Timing:** Fast cycle (10-50ms), Slow cycle (50-1000ms)
- **Motor Limits:** Max torque, regen, reverse
- **Battery Parameters:** Min/max voltage, precharge tolerance, critical cell voltage
- **Safety Limits:** Max motor/inverter/battery temperature
- **WiFi Settings:** AP mode, Station SSID/password
- **System Options:** Debug mode, OTA enable

Click **"Save Configuration"** to apply changes (takes effect immediately).

### CAN Monitor Tab
- Real-time CAN message viewer
- Displays BMS and DMC messages
- Error log with timestamps
- Auto-scroll option
- Clear log buttons

### Task Monitor Tab
- FreeRTOS task statistics
- Task priority, stack usage, state
- Watchdog status for each task
- Free heap memory
- Estimated CPU usage

### System Tab
- System information
- Network status
- OTA firmware update controls
- VCU restart button

## Configuration Parameters

### CAN Timing
- **Fast Cycle:** 10-50ms (default: 10ms) - DMC control message rate
- **Slow Cycle:** 50-1000ms (default: 100ms) - Status message rate

### Motor Limits
- **Max Torque:** 100-1000Nm (default: 850Nm)
- **Max Regen:** 50-500Nm (default: 420Nm)
- **Max Reverse:** 50-300Nm (default: 200Nm)

### Battery Parameters
- **Min Voltage:** 250-350V (default: 320V)
- **Max Voltage:** 350-450V (default: 420V)
- **Precharge Tolerance:** 5-50V (default: 20V)
- **Critical Cell Voltage:** 2.5-3.5V (default: 3.0V)

### Safety Limits
- **Max Motor Temp:** 80-180°C (default: 140°C)
- **Max Inverter Temp:** 60-120°C (default: 100°C)
- **Max Battery Temp:** 40-70°C (default: 55°C)

### Debug Mode
- **Enabled:** VCU never sleeps, WiFi always on
- **Disabled:** Normal operation with deep sleep on stop button hold

## OTA Firmware Updates

### Via Web Interface
1. Open dashboard
2. Go to **System** tab
3. Click **"Enter OTA Mode"**
4. Use PlatformIO or Arduino IDE to upload:
   ```bash
   pio run --target upload --upload-port [vcu-ip-address]
   ```
5. After upload, click **"Exit OTA Mode"**

### Via Arduino IDE OTA
1. Network Port should appear: **egocart-vcu**
2. Select port and upload
3. Password: **egocart123**

## Troubleshooting

### WebSocket Not Connecting
- Check WiFi connection
- Verify IP address
- Restart VCU
- Check browser console for errors

### CAN Messages Not Appearing
- Verify CAN bus connections (CAN_H, CAN_L)
- Check MCP2515 SPI connections
- Verify 500kbps CAN speed matches other devices
- Check serial monitor for CAN init errors

### Configuration Not Saving
- Check free heap memory in Task Monitor
- Verify web request completes (check Network tab in browser)
- Restart VCU and check if changes persisted

### Tasks Failing Watchdog
- Check Task Monitor for failed tasks
- View serial output for error messages
- Failed tasks will auto-restart
- Check stack usage - may need to increase stack size

### Display Not Working
- Check Nextion serial connections (TX/RX)
- Verify 9600 baud rate
- Display failure is non-critical - system continues

## Pin Configuration

### CAN (SPI)
- SCK: GPIO 4
- MISO: GPIO 5
- MOSI: GPIO 6
- CS: GPIO 36
- INT: GPIO 37

### I2C (MCP23017 + ADS1115)
- SDA: GPIO 1
- SCL: GPIO 2

### Digital Inputs
- START: GPIO 7
- STOP: GPIO 8
- CHARGER_WAKEUP: GPIO 9
- RESET: GPIO 10
- INTERLOCK: GPIO 47

### Power Outputs
- WATER_PUMP: GPIO 38
- MAIN_CONTACTOR: GPIO 11
- DMC_ENABLE: GPIO 12
- NLG_ENABLE: GPIO 13
- BMS_ENABLE: GPIO 14
- PRECHARGE_RELAY: GPIO 17
- NEXTION_POWER: GPIO 18

### Display (Serial2)
- TX: GPIO 19
- RX: GPIO 20

## Safety Features

### Hardware Safety
- Interlock monitoring (emergency shutdown)
- Precharge voltage verification before main contactor close
- Watchdog timers on all critical tasks
- Emergency stop function

### Software Safety
- Battery voltage monitoring (min/max)
- Temperature monitoring (motor, inverter, battery)
- SOC monitoring
- Cell voltage monitoring
- BMS communication alive check
- Automatic emergency stop on critical conditions

### Fail-Safe Behavior
- If critical task fails: Auto-restart via watchdog
- If BMS offline during drive: Emergency stop + warning
- If temperature exceeds limit: Warning + optional torque reduction
- If cell voltage critical: Emergency stop
- If interlock opens: Immediate battery disarm

## Performance

### Timing
- CAN RX latency: <1ms (interrupt-driven)
- CAN TX cycle: 10ms (configurable to 50ms)
- Vehicle control: 10ms
- Display update: 50ms
- WebSocket update: 50ms

### Memory Usage
- Flash: ~1.05MB / 2.5MB (40%)
- RAM: ~53KB / 320KB (16%)
- Free heap: Monitored in real-time via dashboard

## Development

### Adding New Configuration Parameters
1. Add to `RuntimeConfigData` in `include/data_structures.h`
2. Update `getDefaultRuntimeConfig()` with default value
3. Add input field in `data/www/index.html` (Config tab)
4. Add to `loadConfig()` and `saveConfig()` in `dashboard.js`
5. Handle in web_server.cpp `handleSetConfig()` method

### Adding New Tasks
1. Create task function in `src/main.cpp`
2. Add `TaskHandle_t` declaration
3. Create task in `setup()` with `xTaskCreatePinnedToCore()`
4. Register with TaskMonitor
5. Add `FEED_WATCHDOG()` calls in task loop

### Modifying CAN Messages
Update parsing in:
- `src/can_manager.cpp` - `processBMSMessage()`, `processDMCMessage()`, `processNLGMessage()`

## Credits

Built with:
- ESP32 Arduino Core
- FreeRTOS
- ESPAsyncWebServer
- ArduinoJson
- MCP2515 CAN Library
- Adafruit MCP23017 Library
- ADS1X15 Library

## License

[Add your license here]

## Version

**v1.0** - FreeRTOS Migration Complete
- Full dual-core FreeRTOS implementation
- Web dashboard with real-time telemetry
- Configurable parameters
- CAN monitor and error logging
- Task monitoring and watchdog
