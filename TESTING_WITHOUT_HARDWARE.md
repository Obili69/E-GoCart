# Testing Web Dashboard Without Hardware

This guide explains how to test the VCU web dashboard and WiFi functionality without physical hardware connected.

## Overview

A **Hardware Test Mode** has been added that allows you to:
- ✅ Test the web dashboard and all its features
- ✅ See realistic simulated telemetry data
- ✅ Test configuration changes
- ✅ Monitor FreeRTOS tasks
- ✅ Verify WiFi connectivity
- ❌ Skip hardware initialization (CAN, I2C sensors, display, inputs)

## Enabling Hardware Test Mode

### Option 1: Using config.h (Recommended for Testing)

Edit [include/config.h](include/config.h) line 290:

```cpp
// Hardware-less Test Mode (bypass hardware init, simulate telemetry)
// Set to 1 to test webserver without physical hardware connected
#define HARDWARE_TEST_MODE  1  // 1 = Simulated data, 0 = Real hardware
```

**Set to `1` for testing, `0` for production with real hardware.**

### Option 2: Using PlatformIO Build Flags

Add to `platformio.ini`:

```ini
build_flags =
    -DHARDWARE_TEST_MODE=1
```

## What Changes in Test Mode

### Tasks Created

**Test Mode (6 tasks):**
1. SimData - Generates realistic simulated telemetry
2. Vehicle - Vehicle control logic
3. State - State machine
4. WiFi - WiFi manager
5. WebServer - Web dashboard and API
6. Monitor - Task monitoring and watchdog

**Normal Mode (10 tasks):**
- All of the above PLUS:
- CAN_RX, CAN_TX - CAN bus communication
- Input - Hardware input reading
- Display - Nextion display
- Safety - Hardware safety monitoring

### Simulated Data

The system generates realistic changing telemetry data:

**Battery (BMS):**
- SOC: 75% (slowly decreases)
- Voltage: 360V
- Current: Varies with motor load (-30A to +150A)
- Cell voltages: 3.5-3.7V
- Temperature: 25-40°C (rises with current)

**Motor (DMC):**
- Speed: 0-3000 RPM (changes every 5 seconds)
- Torque: -50 to +150 Nm (based on acceleration/braking)
- DC Voltage: 360V
- DC Current: Matches battery current
- Inverter temp: 30-50°C
- Motor temp: 28-58°C

**Inputs:**
- Throttle: 0-80% (when accelerating)
- Regen: 0-60% (when braking)
- Brake: Active during regen
- Interlock: Closed
- BMS/DMC: Always alive/ready
- Charger: Not connected

**Behavior:**
- Vehicle alternates between accelerating and decelerating every 5 seconds
- Temperatures rise realistically with load
- Battery slowly discharges (1% every 10 seconds)
- All safety systems report OK

## Testing Procedure

### Step 1: Flash Firmware

```bash
# Enable test mode in config.h first!
pio run --target upload
```

### Step 2: Upload Web Dashboard Files

```bash
pio run --target uploadfs
```

If `uploadfs` fails, the filesystem files may need to be built first. See [DEPLOYMENT.md](DEPLOYMENT.md) for details.

### Step 3: Connect to WiFi

1. Power on the ESP32
2. Look for WiFi network: **E-GoCart-VCU**
3. Connect using password: **egocart123**
4. Open browser to: **http://192.168.4.1**

### Step 4: Serial Monitor (Optional)

Monitor the serial output to see what's happening:

```bash
pio device monitor
```

**Expected output:**
```
========================================
  VCU FreeRTOS - Starting Up
========================================

***********************************************
  HARDWARE TEST MODE - Simulated Data
  Web server and WiFi will run normally
  Hardware managers will be skipped
***********************************************

Initializing State Manager...
Initializing Vehicle Control...
Initializing simulated telemetry data...
Simulated data initialized
Initializing WiFi Manager...
WiFi: AP started
  SSID: E-GoCart-VCU
  IP: 192.168.4.1
Initializing Web Server...
WebServer: Started on port 80
WebSocket server started on /ws

========================================
  Creating FreeRTOS Tasks
========================================

Creating simulated data task...
Task: Simulated Data Generator started
Task: State Manager started
Task: Vehicle Control started
Task: WiFi Manager started
Task: Web Server started
Task: Task Monitor started
```

### Step 5: Test Dashboard Features

**Dashboard Tab:**
- ✅ Verify speed increases/decreases realistically
- ✅ Check gear display (should show "N" for neutral)
- ✅ Watch power vary with speed
- ✅ Observe speed and power charts update
- ✅ Confirm SOC slowly decreases
- ✅ Check temperature values change

**Configuration Tab:**
- ✅ Change CAN Fast Cycle from 10ms to 20ms
- ✅ Modify max torque limit
- ✅ Adjust battery voltage limits
- ✅ Click "Save Configuration"
- ✅ Refresh page - changes should persist

**CAN Monitor Tab:**
- ⚠️ Will show "No CAN messages" (expected - no real CAN bus)
- ✅ Can still test the UI and clear buttons

**Task Monitor Tab:**
- ✅ Should show 6 running tasks
- ✅ Verify stack usage is healthy (<80%)
- ✅ Check watchdog status is "OK"
- ✅ Confirm free heap is stable

**System Tab:**
- ✅ System info should be correct
- ✅ Network shows AP mode active
- ✅ Can enter OTA mode (don't upload yet)
- ✅ Restart button works

### Step 6: WebSocket Testing

Open browser developer console (F12) and watch the Network tab:

1. Look for WebSocket connection to `ws://192.168.4.1/ws`
2. Should see messages every 50ms with telemetry data
3. Data should be updating continuously

**Example WebSocket message:**
```json
{
  "state": 1,
  "gear": 0,
  "speed": 45.2,
  "power": 12.3,
  "soc": 74,
  "voltage": 360,
  "current": 85,
  "tempMotor": 45.2,
  "tempInverter": 38.5,
  "tempBattery": 32.1,
  "throttle": 65,
  "regen": 0,
  "brake": false
}
```

## Known Limitations in Test Mode

1. **No CAN Messages**: CAN monitor will be empty (no real CAN bus)
2. **No Physical Inputs**: Can't test buttons, switches, pedals
3. **No Display**: Nextion display manager not initialized
4. **State Machine**: Won't transition through full drive cycle without inputs
5. **Safety Monitor**: Not running (temperatures always OK)

## Performance Verification

**Normal Operation in Test Mode:**
- Free heap: ~250KB (should stay stable)
- Task switching: Smooth, no crashes
- WebSocket latency: <100ms
- Dashboard load time: <2 seconds
- Configuration save: <500ms

**If you see issues:**
- Task crashes: Check serial monitor for watchdog errors
- Memory leaks: Watch free heap in Task Monitor
- Slow updates: Check WiFi signal strength
- Config not saving: Verify LittleFS mounted

## Switching to Real Hardware Mode

When your VCU hardware arrives:

1. Edit [include/config.h](include/config.h) line 290:
   ```cpp
   #define HARDWARE_TEST_MODE  0  // Back to real hardware
   ```

2. Rebuild and flash:
   ```bash
   pio run --target upload
   ```

3. All 10 tasks will now be created
4. Real CAN messages will appear
5. Physical inputs will work
6. Display will initialize

## Troubleshooting

### Dashboard Won't Load
- Verify WiFi connected to "E-GoCart-VCU"
- Try http://192.168.4.1 (not https)
- Check serial monitor for WiFi errors
- Ensure filesystem was uploaded (`uploadfs`)

### No Telemetry Updates
- Open browser console (F12)
- Look for WebSocket errors
- Verify free heap >50KB (Task Monitor)
- Check SimData task is running (Task Monitor)

### Configuration Won't Save
- Check free heap in Task Monitor
- Verify LittleFS mounted in serial output
- Try clearing browser cache
- Check serial monitor for errors

### Simulated Data Not Changing
- Verify SimData task running (Task Monitor)
- Check watchdog status is OK
- Look for task crashes in serial monitor
- Restart ESP32

## Development Tips

### Modifying Simulated Data

Edit [src/main.cpp](src/main.cpp) in the `taskSimulatedData()` function (around line 544):

```cpp
// Change speed range
speedTarget = random(500, 5000);  // 500-5000 RPM instead of 1000-3000

// Change update frequency
const TickType_t xFrequency = pdMS_TO_TICKS(50);  // 50ms instead of 100ms

// Add custom scenarios
if (millis() > 30000) {
    // Simulate battery critical after 30 seconds
    bmsData.soc = 10;
    bmsData.minCellVoltage = 2.8f;
}
```

### Testing Configuration Persistence

1. Change a parameter via web interface
2. Restart ESP32
3. Check if value persisted
4. If not, check LittleFS in serial monitor

### Stress Testing

Leave the dashboard open for extended periods:
- Monitor free heap (should stay stable)
- Watch for WebSocket disconnections
- Check for task crashes
- Verify no memory leaks

## What to Test Before Hardware Deployment

- [x] WiFi AP mode works reliably
- [x] Web dashboard loads quickly
- [x] Real-time telemetry updates smoothly
- [x] Configuration changes save and persist
- [x] Task monitor shows all tasks healthy
- [x] OTA mode can be entered (don't flash yet)
- [x] WebSocket connection is stable
- [x] Charts draw correctly
- [x] Mobile responsive design works
- [x] Multiple clients can connect (test with phone + laptop)

## Next Steps

Once testing is complete and you have the VCU hardware:

1. Set `HARDWARE_TEST_MODE  0` in config.h
2. Follow [DEPLOYMENT.md](DEPLOYMENT.md) for full deployment
3. Start with low voltage testing (see safety procedures)
4. Gradually bring up real CAN bus, inputs, display

---

**Important:** Always remember to disable test mode before deploying to real vehicle hardware!

**Version:** v1.0 - Hardware Test Mode
**Last Updated:** 2025-10-18
