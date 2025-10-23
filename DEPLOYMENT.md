# E-GoCart VCU - Deployment Checklist

## Pre-Deployment Verification

### 1. Build Status
- ✅ Firmware compiles successfully
- ✅ Firmware size: 972KB / 1966KB (49.5% flash usage)
- ✅ RAM usage: 52KB / 327KB (16.0%)
- ✅ Web dashboard files created in `data/www/`

### 2. Required Files
- ✅ `data/www/index.html` (26KB) - Main dashboard HTML
- ✅ `data/www/dashboard.js` (17KB) - Dashboard JavaScript
- ✅ `README_FREERTOS.md` - Complete documentation

## Deployment Steps

### Step 1: Upload Firmware
```bash
pio run --target upload
```

**Expected Output:**
- Firmware upload progress
- Success message
- Device will automatically restart

**Verification:**
- Open serial monitor: `pio device monitor`
- Look for "WiFiManager: Initializing..." messages
- Confirm all 10 tasks start successfully

### Step 2: Upload Filesystem (Web Dashboard)
```bash
pio run --target uploadfs
```

**Expected Output:**
- LittleFS filesystem upload progress
- Success message
- Device will automatically restart

**Note:** If `uploadfs` target is not available, add to `platformio.ini`:
```ini
board_build.filesystem = littlefs
```

**Verification:**
- Serial monitor should show "LittleFS mounted"
- No filesystem errors

### Step 3: Initial WiFi Connection

**Access Point Mode (Default):**
1. Power on the VCU
2. Look for WiFi network: **E-GoCart-VCU**
3. Connect using password: **egocart123**
4. Open browser to: **http://192.168.4.1**

**Serial Monitor Verification:**
```
WiFi: AP started
  SSID: E-GoCart-VCU
  IP: 192.168.4.1
```

### Step 4: Web Dashboard Verification

Access the dashboard and verify each tab:

**Dashboard Tab:**
- ✅ Telemetry values update in real-time (should update every 50ms)
- ✅ Gear display shows current state (N/D/R)
- ✅ Speed chart draws correctly
- ✅ Power chart draws correctly
- ✅ Temperature warnings show correct colors (green/yellow/red)

**Configuration Tab:**
- ✅ All parameters load with default values
- ✅ Try changing CAN Fast Cycle from 10ms to 20ms
- ✅ Click "Save Configuration"
- ✅ Verify changes persist after VCU restart

**CAN Monitor Tab:**
- ✅ CAN messages appear (once CAN bus is connected)
- ✅ Error log displays any CAN errors
- ✅ Auto-scroll works
- ✅ Clear buttons function

**Task Monitor Tab:**
- ✅ All 10 tasks listed
- ✅ Stack usage displayed (should be well below maximum)
- ✅ Watchdog status shows "OK" for all tasks
- ✅ Free heap memory displayed

**System Tab:**
- ✅ System information correct (chip model, flash size, etc.)
- ✅ Network status shows AP mode active
- ✅ OTA controls present (don't test yet)

### Step 5: Hardware Connection Tests

**CAN Bus Connection:**
1. Connect CAN_H and CAN_L to vehicle CAN bus
2. Verify 120Ω termination resistor installed
3. Power on BMS and DMC
4. Check serial monitor for "CAN: Message received" logs
5. Check CAN Monitor tab for BMS and DMC messages

**Expected CAN IDs:**
- BMS: 0x200, 0x201, 0x202
- DMC: 0x258, 0x259, 0x25E, 0x45E, 0x65E
- NLG: 0x610, 0x611, 0x618, 0x619

**Digital Inputs:**
1. Test START button (GPIO 7)
2. Test STOP button (GPIO 8)
3. Test CHARGER_WAKEUP (GPIO 9)
4. Test RESET (GPIO 10)
5. Test INTERLOCK (GPIO 47)

**Verify in serial monitor:**
```
Input: START pressed
State: STARTING
```

**Power Outputs:**
1. Verify MAIN_CONTACTOR (GPIO 11) - Use multimeter
2. Verify PRECHARGE_RELAY (GPIO 17) - Use multimeter
3. Verify DMC_ENABLE (GPIO 12)
4. Verify BMS_ENABLE (GPIO 14)

**Warning:** Do NOT test with high voltage connected until precharge sequence is fully verified!

### Step 6: Precharge Sequence Test

**Important Safety Notes:**
- Start with LOW voltage battery pack (<50V) for initial testing
- Have emergency stop button ready
- Monitor all voltages with multimeter
- Gradually increase voltage for testing

**Test Procedure:**
1. Connect low voltage battery pack to BMS
2. Power on VCU
3. Press START button
4. Monitor serial output:
   ```
   State: STARTING
   Precharge: Timer started
   Precharge: BMS=48.0V, DMC=0.0V, Diff=48.0V
   Precharge: BMS=48.0V, DMC=45.0V, Diff=3.0V
   Precharge: BMS=48.0V, DMC=47.5V, Diff=0.5V
   Precharge: Voltage match! Completing precharge...
   State: READY
   ```
5. Verify main contactor closes only after voltage match
6. Verify timeout works (disconnect precharge relay during test)

### Step 7: Station Mode WiFi Setup (Optional)

**For Workshop Use:**
1. Open dashboard Configuration tab
2. Scroll to WiFi Settings
3. Enable "Station Mode"
4. Enter your WiFi SSID and password
5. Click "Save Configuration"
6. VCU will restart and connect to your network
7. Find new IP in serial monitor or router
8. Access dashboard at new IP address

**Serial Monitor Verification:**
```
WiFi: Connecting to 'YourNetwork'...
WiFi: STA connected!
  IP: 192.168.1.xxx
```

## Post-Deployment Monitoring

### Critical Checks (First 24 Hours)

**Every Hour:**
- Check Task Monitor for watchdog failures
- Monitor free heap memory (should stay stable)
- Check for CAN communication errors
- Verify all tasks show "RUNNING" state

**Serial Monitor Warnings to Watch:**
```
ERROR: Task XYZ watchdog timeout!
ERROR: Stack overflow detected!
ERROR: Heap allocation failed!
ERROR: CAN init failed!
```

### Performance Baselines

**Normal Operation:**
- CAN RX latency: <1ms
- CAN TX cycle: 10ms (configurable)
- WebSocket update: 50ms
- Free heap: >200KB
- Stack high water marks: <80% of allocated size

**Task Priorities (For Reference):**
```
CAN RX:           24 (Highest)
CAN TX:           23
Vehicle Control:  22
State Manager:    21
Safety Monitor:   20
Input Manager:    10
Display Manager:   5
WiFi Manager:      4
Webserver:         3
Task Monitor:      2 (Lowest)
```

## Troubleshooting Common Issues

### Issue: Dashboard not accessible
**Check:**
1. Is VCU powered on?
2. Is WiFi AP "E-GoCart-VCU" visible?
3. Are you connected to the correct WiFi network?
4. Try http://192.168.4.1 (not https)
5. Check serial monitor for WiFi errors

### Issue: CAN messages not appearing
**Check:**
1. CAN bus connections (CAN_H to high, CAN_L to low)
2. 120Ω termination resistor installed
3. MCP2515 SPI connections (SCK, MISO, MOSI, CS, INT)
4. BMS and DMC powered on and transmitting
5. Serial monitor shows "CAN: Initialized" message

### Issue: Configuration not saving
**Check:**
1. Did you see "Config updated" message after clicking Save?
2. Check free heap in Task Monitor (need >50KB)
3. Verify LittleFS mounted in serial monitor
4. Try restarting VCU and checking if values persist

### Issue: Tasks failing watchdog
**Check:**
1. Which task is failing? (Check Task Monitor or serial output)
2. Stack high water mark (might need to increase stack size)
3. Any blocking operations in task loop?
4. Check for infinite loops or deadlocks
5. Verify FEED_WATCHDOG() called regularly

### Issue: WebSocket disconnecting
**Check:**
1. WiFi signal strength
2. Free heap memory (WebSocket uses dynamic allocation)
3. Too many clients connected? (Limit: 4)
4. Browser console for error messages
5. Try different browser

### Issue: OTA update fails
**Check:**
1. Entered OTA mode via System tab?
2. Correct IP address used?
3. Password correct? (egocart123)
4. Firewall blocking connection?
5. Large firmware size? (max ~1.9MB)

## Safety Reminders

⚠️ **Before High Voltage Testing:**
1. Verify precharge works with low voltage
2. Test emergency stop button
3. Verify interlock circuit
4. Have proper PPE (insulated gloves, safety glasses)
5. Ensure emergency disconnect accessible
6. Never work alone with high voltage

⚠️ **Software Safety Features Active:**
- Battery voltage monitoring (320-420V default)
- Temperature monitoring (motor, inverter, battery)
- Cell voltage monitoring (critical below 3.0V default)
- BMS communication alive check
- Automatic emergency stop on critical conditions

⚠️ **Hardware Safety Features:**
- Interlock monitoring (emergency shutdown)
- Precharge voltage verification
- Watchdog task restart
- Main contactor only closes after precharge complete

## Next Steps After Successful Deployment

1. **Calibration:**
   - Adjust max torque based on motor capabilities
   - Fine-tune regen braking strength
   - Set appropriate temperature limits
   - Calibrate throttle and brake pedal ranges

2. **Optimization:**
   - Monitor CAN timing under load
   - Adjust task priorities if needed
   - Tune WiFi power management for battery life
   - Optimize WebSocket update rate vs WiFi power

3. **Data Logging:**
   - Consider adding SD card logging for diagnostics
   - Log critical events (errors, state changes, safety triggers)
   - Record telemetry for performance analysis

4. **Advanced Features:**
   - Implement traction control
   - Add regenerative braking optimization
   - Create driving modes (Eco, Sport, etc.)
   - Add battery preheating/cooling control

## Support and Documentation

- **Full Documentation:** See `README_FREERTOS.md`
- **Architecture Details:** See `README_FREERTOS.md` - Architecture section
- **Configuration Reference:** See `README_FREERTOS.md` - Configuration Parameters
- **Pin Mapping:** See `README_FREERTOS.md` - Pin Configuration
- **Serial Monitor:** `pio device monitor` (115200 baud)

## Version Information

**Current Version:** v1.0 - FreeRTOS Migration Complete

**Build Info:**
- Platform: ESP32-S3
- Framework: Arduino
- Flash: 972KB / 1966KB (49.5%)
- RAM: 52KB / 327KB (16.0%)
- Tasks: 10 (across 2 cores)

---

**Deployment Date:** _________________

**Deployed By:** _________________

**Notes:**

_________________

_________________

_________________
