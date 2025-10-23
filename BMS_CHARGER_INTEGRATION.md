# BMS and Charger Integration

## Overview

This document describes the integration of the **192S BMS** (configured for 104S LiPo) and **NLG5 Charger** (manual control, no CP) into the E-GoCart VCU system.

## System Components

### 1. 192S BMS Manager (`bms_manager.h/cpp`)
- **CAN Protocol**: Extended format, 500kbps
- **CAN IDs**: RX=0xF4, TX=0xF5
- **Configuration**: 104S LiPo cells (3.0-4.2V per cell)
- **Pack Voltage Range**: 312V - 437V
- **Features**:
  - Real-time monitoring of all 104 cell voltages
  - 16 temperature module readings
  - SOC, capacity, current, power monitoring
  - Protection status (overtemp, overvoltage, overcurrent, etc.)
  - Balancing status and control
  - MOS (charge/discharge) status
  - Automatic timeout detection
  - Thread-safe data access via FreeRTOS mutexes

### 2. NLG5 Charger Manager (`nlg5_manager.h/cpp`)
- **CAN Protocol**: Standard format, 500kbps
- **CAN IDs**: Control=0x618, Status=0x610, Actuals=0x611-0x612, Temps=0x613, Errors=0x614
- **Operating Mode**: Manual control (no Control Pilot/SAE J1772)
- **Features**:
  - CC/CV (Constant Current / Constant Voltage) charging state machine
  - Real-time monitoring of:
    - Mains voltage and current (AC input)
    - Battery voltage and current (DC output)
    - Temperatures (power stage + 3 external sensors)
    - Power limiting conditions
    - Error and warning flags
  - Automatic error detection and clearing
  - Configurable charging limits
  - Thread-safe operation

### 3. Charging Coordinator (`charging_coordinator.h/cpp`)
- **Purpose**: Safety coordinator between BMS and NLG5
- **Charging States**:
  1. `IDLE` - Not charging
  2. `PRECHECK` - Pre-charge safety checks
  3. `STARTING` - Starting charger
  4. `BULK_CHARGE` - Constant current phase
  5. `ABSORPTION` - Constant voltage phase
  6. `BALANCING` - Optional balancing phase
  7. `COMPLETE` - Charging complete
  8. `ERROR` - Error state

- **Safety Features**:
  - BMS alive check (communication timeout)
  - BMS error monitoring
  - Temperature limits (0-45°C charge range)
  - Cell voltage limits (3.0-4.17V per cell)
  - Cell voltage imbalance detection
  - Charger error monitoring
  - Maximum charge time limit (4 hours)
  - Automatic shutdown on any safety violation

## Configuration Parameters

### Battery (104S LiPo) - [config.h](include/config.h:137-158)
```cpp
NUM_CELLS = 104
CELL_MIN_V = 3.0V
CELL_MAX_V = 4.2V
CELL_NOM_V = 3.7V
PACK_MIN_V = 312V
PACK_MAX_V = 437V
PACK_NOM_V = 385V
MAX_CELL_DELTA = 0.05V  // Max imbalance
BMS_TIMEOUT_MS = 1000ms
```

### Charger (NLG5) - [config.h](include/config.h:195-221)
```cpp
CHARGE_VOLTAGE_MAX = 4.17V per cell (433.68V pack)
CHARGE_VOLTAGE_BULK = 4.10V per cell (426.4V pack)
CHARGE_CURRENT_MAX = 50A
CHARGE_CURRENT_TAPER = 2A  // CV phase end current
CHARGE_POWER_MAX = 3300W
MAINS_CURRENT_MAX = 16A
CHARGE_TEMP_MIN = 0°C
CHARGE_TEMP_MAX = 45°C
CHARGER_TEMP_MAX = 80°C
CHARGE_TIMEOUT_MS = 4 hours
```

## Charging Profile (LiPo CC/CV)

### Phase 1: Bulk Charge (Constant Current)
- **Current**: 50A (or BMS limit, whichever is lower)
- **Voltage**: Increases from current voltage toward 426.4V
- **Duration**: Until pack voltage reaches ~426V
- **Temperature Derating**: Reduces current if temp >35°C

### Phase 2: Absorption (Constant Voltage)
- **Voltage**: 426.4V (4.10V per cell)
- **Current**: Tapers from 50A down to 2A
- **Duration**: Until current drops below 2A
- **End Condition**: Current < 2A = charging complete

### Safety Margins
- Voltage: 99% of BMS limit (1% safety margin)
- Current: 95% of BMS limit (5% safety margin)

## CAN Message IDs

### BMS Messages
| ID (Hex) | ID (Dec) | Direction | Description |
|----------|----------|-----------|-------------|
| 0xF5 | 245 | BMS → VCU | BMS telemetry (84 message groups) |
| 0xF4 | 244 | VCU → BMS | BMS commands |

### NLG5 Messages
| ID (Hex) | ID (Dec) | Direction | Description |
|----------|----------|-----------|-------------|
| 0x618 | 1560 | VCU → NLG5 | Control (enable, voltage, current) |
| 0x610 | 1552 | NLG5 → VCU | Status flags |
| 0x611 | 1553 | NLG5 → VCU | Actuals I (mains V/A, battery V/A) |
| 0x612 | 1554 | NLG5 → VCU | Actuals II (aux, CP, PI limits) |
| 0x613 | 1555 | NLG5 → VCU | Temperatures |
| 0x614 | 1556 | NLG5 → VCU | Errors and warnings |

## Integration Points

### 1. CAN Manager Integration
The existing `CANManager` needs to be updated to route messages to the new managers:

```cpp
// In can_manager.cpp processCANMessage()
case CAN_ID::BMS_TX:        // 0xF5
    bmsManager.processMessage(id, buf, len);
    break;

case CAN_ID::NLG5_ST:       // 0x610
case CAN_ID::NLG5_ACT_I:    // 0x611
case CAN_ID::NLG5_ACT_II:   // 0x612
case CAN_ID::NLG5_TEMP:     // 0x613
case CAN_ID::NLG5_ERR:      // 0x614
    nlg5Manager.processMessage(id, buf, len);
    break;
```

### 2. Main Loop Integration
Add to main task scheduler:

```cpp
// Create managers (global)
BMSManager bmsManager;
NLG5Manager nlg5Manager;
ChargingCoordinator chargingCoordinator(bmsManager, nlg5Manager);

// In setup()
bmsManager.begin();
nlg5Manager.begin();
chargingCoordinator.begin();

// In periodic update tasks (e.g., 100ms)
bmsManager.update();
nlg5Manager.update();
chargingCoordinator.update();
```

### 3. State Manager Integration
Add charging state to vehicle state machine:

```cpp
// When charger connected and vehicle in READY state
if (chargerConnected && !isCharging) {
    chargingCoordinator.startCharging();
}

// When charging complete or user requests stop
if (chargingComplete || stopRequested) {
    chargingCoordinator.stopCharging();
}
```

## Usage Example

### Starting a Charge Session
```cpp
// Check if safe to charge
if (chargingCoordinator.isSafeToCharge()) {
    // Start charging - coordinator handles everything
    bool success = chargingCoordinator.startCharging();
    if (success) {
        Serial.println("Charging started");
    } else {
        Serial.println("Failed to start charging");
    }
}
```

### Monitoring Charging Status
```cpp
ChargingStatus status = chargingCoordinator.getStatus();

Serial.printf("State: %d\n", (int)status.state);
Serial.printf("SOC: %d%%\n", status.socPercent);
Serial.printf("Voltage: %.1fV\n", status.actualVoltage);
Serial.printf("Current: %.1fA\n", status.actualCurrent);
Serial.printf("Power: %.0fW\n", status.chargePower);
Serial.printf("Energy: %.1f Wh\n", status.energyCharged);
Serial.printf("Time remaining: %.0f min\n", status.timeRemaining);

if (!status.safetyOK) {
    Serial.printf("ERROR: %s\n", status.errorMessage.c_str());
}
```

### Stopping Charging
```cpp
chargingCoordinator.stopCharging();
```

## Safety Features Summary

1. **BMS Communication Watchdog**: Stops charging if BMS stops responding (1s timeout)
2. **Charger Communication Watchdog**: Stops charging if charger stops responding (2s timeout)
3. **Temperature Protection**:
   - Low temp lockout: <0°C
   - High temp lockout: >45°C
   - Charger overtemp: >80°C
4. **Voltage Protection**:
   - Cell overvoltage: >4.17V
   - Cell undervoltage: <3.0V (discharge only)
   - Cell imbalance: >50mV delta
5. **Current Protection**: Respects BMS current limits
6. **Time Protection**: 4 hour maximum charge time
7. **BMS Error Monitoring**: Stops on any BMS protection flag
8. **Charger Error Monitoring**: Stops on any critical NLG5 error

## Next Steps

1. **CAN Manager Update**: Route BMS and NLG5 messages to appropriate managers
2. **Display Integration**: Show charging status on Nextion display
3. **Web Interface**: Add charging controls and monitoring to web dashboard
4. **Testing**: Test with real hardware (BMS simulator if needed)
5. **Logging**: Add charging session logging to LittleFS
6. **Balancing**: Implement BMS balancing control during charging

## Files Created

### Headers
- `include/bms_manager.h` - BMS manager class and data structures
- `include/nlg5_manager.h` - NLG5 charger manager class and data structures
- `include/charging_coordinator.h` - Charging coordinator class

### Implementation
- `src/bms_manager.cpp` - BMS message parsing and management
- `src/nlg5_manager.cpp` - NLG5 control and status processing
- `src/charging_coordinator.cpp` - Charging state machine and safety logic

### Configuration
- `include/config.h` - Updated with BMS and charger parameters

## References

- **BMS Protocol**: `BMS/192S CAN port communication protocol.pdf`
- **NLG5 Database**: `NLG/NLG5_DatabaseCAN.dbc`
- **NLG5 Manual**: `NLG/NLG513-U1-xxA_Manual_DE.pdf` (German)
