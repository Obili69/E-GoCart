#include "bms_manager.h"
#include "can_manager.h"

//=============================================================================
// BMS COMMAND CODES (for sendCommand function)
//=============================================================================
namespace BMSCommand {
    constexpr uint8_t DISCHARGE_PROTECTION_V = 0x01;  // Scale: voltage * 100
    constexpr uint8_t OVERCURRENT_PROTECTION = 0x02;  // Scale: current * 10
    constexpr uint8_t BATTERY_CAPACITY       = 0x03;  // Scale: capacity * 10
    constexpr uint8_t NUM_CELLS              = 0x04;  // Number of series cells
    constexpr uint8_t CHARGE_PROTECTION_V    = 0x05;  // Scale: voltage * 100
    constexpr uint8_t OVERTEMP_PROTECTION    = 0x06;  // Scale: temp * 10
    constexpr uint8_t CHANNEL_CONTROL        = 0x07;  // 0x0001=on, 0x0002=off
    constexpr uint8_t BALANCE_CONTROL        = 0x08;  // 0x0002=on, 0x0001=off
    constexpr uint8_t RESET_DISCHARGE_CAP    = 0x09;
    constexpr uint8_t CHARGE_RECOVERY_V      = 0x0A;
    constexpr uint8_t DISCHARGE_RECOVERY_V   = 0x0B;
    constexpr uint8_t CHANNEL_DEFAULT_STATE  = 0x0C;  // 0x0002=default on
    constexpr uint8_t HOST_DISPLAY_SWITCH    = 0x0D;  // 0x0002=off, 0x0001=on
    constexpr uint8_t COMMUNICATION_CONNECT  = 0x10;  // 0x0002=connected
    constexpr uint8_t BALANCE_START_VOLTAGE  = 0x11;  // Scale: voltage * 100
    constexpr uint8_t AUTO_RESET_CAPACITY    = 0x15;  // 0x0001=on
    constexpr uint8_t PRECHARGE_DELAY_TIME   = 0x16;  // Pre-charge delay in seconds
    constexpr uint8_t LOW_TEMP_PROTECTION    = 0x18;  // Scale: temp * 10
    constexpr uint8_t CAN_CONNECTED          = 0x1C;  // 0x0002=connected
}

//=============================================================================
// CONSTRUCTOR
//=============================================================================

BMSManager::BMSManager()
    : canManager(nullptr) {
    dataMutex = xSemaphoreCreateMutex();
    memset(&data, 0, sizeof(data));
    memset(receivedGroups, 0, sizeof(receivedGroups));
}

//=============================================================================
// BEGIN
//=============================================================================

void BMSManager::begin(CANManager* canMgr) {
    DEBUG_PRINTLN("BMSManager: Initializing...");

    canManager = canMgr;

    // Reset data structure
    memset(&data, 0, sizeof(data));
    data.dataValid = false;
    data.numCellsConfigured = Battery::NUM_CELLS_TESTING;  // Start with test config

    // Request connection with BMS
    requestConnection();

    // Wait for BMS to process connection commands
    vTaskDelay(pdMS_TO_TICKS(400));

    // Send full configuration
    sendConfiguration(Battery::NUM_CELLS_TESTING);

    DEBUG_PRINTLN("BMSManager: Initialized (connection + configuration sent)");
}

//=============================================================================
// PROCESS INCOMING CAN MESSAGE
//=============================================================================

void BMSManager::processMessage(uint32_t id, const uint8_t* buf, uint8_t len) {
    // BMS sends on 0xF5 (extended 29-bit), mask for comparison
    uint32_t masked_id = id & 0x1FFFFFFF;

    if (masked_id != CAN_ID::BMS_TX) {
        return;  // Not a BMS message
    }

    if (len < 7) {
        // BMS messages are typically 7-8 bytes (group ID + 6-7 data bytes)
        DEBUG_PRINTF("BMS: Message too short (%d bytes), group 0x%02X\n", len, buf[0]);
        return;
    }

    // Get message group ID from first byte
    uint8_t groupId = buf[0];

#if DEBUG_CAN_MESSAGES
    DEBUG_PRINTF("BMS: Processing group 0x%02X [%d bytes]\n", groupId, len);
#endif

    // Lock mutex for data update
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        // Process message based on group ID
        switch (groupId) {
            case 0x01: processGroup1(buf); break;
            case 0x02: processGroup2(buf); break;
            case 0x03: processGroup3(buf); break;
            case 0x04: processGroup4(buf); break;
            case 0x05: processGroup5(buf); break;
            case 0x06: processGroup6(buf); break;

            // Cell voltages (groups 7-41 for 104 cells)
            case 0x07 ... 0x29:
                processCellVoltages(groupId, buf);
                break;

            // Module temperatures (groups 71-79)
            case 0x47 ... 0x4F:
                processModuleTemps(groupId, buf);
                break;

            // Additional data
            case 0x50: processGroup80(buf); break;
            case 0x51: processGroup81(buf); break;
            case 0x52: processGroup82(buf); break;

            default:
                // Unknown message group, ignore
                break;
        }

        data.lastUpdateTime = millis();
        data.dataValid = true;

        xSemaphoreGive(dataMutex);
    }
}

//=============================================================================
// MESSAGE GROUP PROCESSORS
//=============================================================================

void BMSManager::processGroup1(const uint8_t* buf) {
    // Group 1: Discharge protection voltage, protective current, battery capacity
    data.dischargeProtectionVoltage = parseU16(buf, 1);  // 0.1V
    data.protectionCurrent = parseU16(buf, 3);           // A
    data.packCapacityAh = parseU16(buf, 5);              // Ah
    receivedGroups[0] = 1;
}

void BMSManager::processGroup2(const uint8_t* buf) {
    // Group 2: Number of cells, charge protection voltage, protection temperature
    data.numCellsConfigured = parseU16(buf, 1);
    data.chargeProtectionVoltage = parseU16(buf, 3);     // 0.1V
    data.protectionTemp = parseU16(buf, 5);              // 0.1°C
    receivedGroups[1] = 1;
}

void BMSManager::processGroup3(const uint8_t* buf) {
    // Group 3: Total voltage, current, power
    data.packVoltage = parseU16(buf, 1);                 // 0.1V
    data.packCurrent = parseI16(buf, 3);                 // 0.01A (signed)
    data.packPower = parseU16(buf, 5);                   // W
    receivedGroups[2] = 1;
}

void BMSManager::processGroup4(const uint8_t* buf) {
    // Group 4: Battery usage capacity, capacity percentage, charging capacity
    data.usedCapacityAh = parseU16(buf, 1);              // Ah
    data.capacityPercent = parseU16(buf, 3);             // 0.1% (divide by 10)
    data.chargeCapacityAh = parseU16(buf, 5);            // Ah
    receivedGroups[3] = 1;
}

void BMSManager::processGroup5(const uint8_t* buf) {
    // Group 5: Recovery voltages, remaining capacity
    data.chargeRecoveryVoltage = parseU16(buf, 1);       // 0.1V
    data.dischargeRecoveryVoltage = parseU16(buf, 3);    // 0.1V
    data.remainingCapacityAh = parseU16(buf, 5);         // Ah
    receivedGroups[4] = 1;
}

void BMSManager::processGroup6(const uint8_t* buf) {
    // Group 6: Host temperature, status accounting, equalization voltage
    data.hostTemp = parseI16(buf, 1) * 0.1f;             // 0.1°C
    data.statusFlags = parseU16(buf, 3);                 // Status bits
    data.balanceStartVoltage = parseU16(buf, 5);         // mV

    // Parse status bits (see Table 1 in protocol)
    data.overTemp = parseBit(data.statusFlags, 0);       // Bit 1
    data.overCharge = parseBit(data.statusFlags, 1);     // Bit 2
    data.cellError = parseBit(data.statusFlags, 2);      // Bit 3
    data.overCurrent = parseBit(data.statusFlags, 3);    // Bit 4
    data.overDischarge = parseBit(data.statusFlags, 4);  // Bit 5
    data.balancing = parseBit(data.statusFlags, 5);      // Bit 6
    data.charging = parseBit(data.statusFlags, 6);       // Bit 7 (current polarity)
    data.channelEnabled = parseBit(data.statusFlags, 7); // Bit 8
    data.tempNegative = parseBit(data.statusFlags, 8);   // Bit 9

    receivedGroups[5] = 1;
}

void BMSManager::processCellVoltages(uint8_t group, const uint8_t* buf) {
    // Groups 7-41 contain cell voltages (3 cells per message)
    // Group 7 = cells 1-3, Group 8 = cells 4-6, etc.

    uint8_t baseCellIndex = (group - 0x07) * 3;

    // Only process if within our 104 cell range
    if (baseCellIndex < Battery::NUM_CELLS) {
        // Cell 1
        if (baseCellIndex < Battery::NUM_CELLS) {
            data.cellVoltage[baseCellIndex] = parseU16(buf, 1);  // mV
        }
        // Cell 2
        if (baseCellIndex + 1 < Battery::NUM_CELLS) {
            data.cellVoltage[baseCellIndex + 1] = parseU16(buf, 3);  // mV
        }
        // Cell 3
        if (baseCellIndex + 2 < Battery::NUM_CELLS) {
            data.cellVoltage[baseCellIndex + 2] = parseU16(buf, 5);  // mV
        }
    }
}

void BMSManager::processModuleTemps(uint8_t group, const uint8_t* buf) {
    // Groups 71-79 (0x47-0x4F) contain module temperatures
    // The layout is a bit complex, see groups 71-76 in the protocol

    uint8_t moduleGroup = group - 0x47;  // 0-8

    // Simplified parsing based on protocol structure
    // Groups alternate between temperature values and polarity indicators
    // For now, just extract what we can from the documented format

    switch (group) {
        case 0x47:  // Module 1 temp
            data.moduleTemp[0] = parseI16(buf, 3);  // 0.1°C
            break;
        case 0x48:  // Modules 2 and 3
            data.moduleTemp[1] = parseI16(buf, 1);
            data.moduleTemp[2] = parseI16(buf, 5);
            break;
        case 0x49:  // Module 4
            data.moduleTemp[3] = parseI16(buf, 3);
            break;
        // Continue for remaining modules...
        // (Full implementation would handle all 16 modules)
    }
}

void BMSManager::processGroup80(const uint8_t* buf) {
    // Group 80 (0x50): Low voltage power outage protection, delays, trigger cells
    // Not critical for basic operation, skip for now
}

void BMSManager::processGroup81(const uint8_t* buf) {
    // Group 81 (0x51): Min/max voltages, MOS status
    data.balanceRefVoltage = parseU16(buf, 1);           // mV
    data.minCellVoltage = parseU16(buf, 3) / 1000.0f;    // mV to V
    data.maxCellVoltage = parseU16(buf, 5) / 1000.0f;    // mV to V

    // MOS status byte (byte 7): 0x01=discharge, 0x10=charge
    uint8_t mosStatus = buf[7];
    data.dischargeMOSEnabled = (mosStatus & 0x01) != 0;
    data.chargeMOSEnabled = (mosStatus & 0x10) != 0;
}

void BMSManager::processGroup82(const uint8_t* buf) {
    // Group 82 (0x52): Accumulated total capacity
    // Not critical for basic operation, skip for now
}

//=============================================================================
// UPDATE TASK
//=============================================================================

void BMSManager::update() {
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        // Check timeout
        checkTimeout();

        // Update min/max cell voltages from individual cells
        updateMinMaxCells();

        // Update min/max temperatures
        updateMinMaxTemps();

        // Update shared data structure
        updateSharedData();

        xSemaphoreGive(dataMutex);
    }
}

//=============================================================================
// HELPER FUNCTIONS
//=============================================================================

void BMSManager::updateMinMaxCells() {
    if (Battery::NUM_CELLS == 0) return;

    float minV = 5.0f;
    float maxV = 0.0f;
    uint8_t minIdx = 0;
    uint8_t maxIdx = 0;

    for (uint8_t i = 0; i < Battery::NUM_CELLS; i++) {
        float cellV = data.cellVoltage[i] / 1000.0f;  // mV to V
        if (cellV > 0.1f) {  // Valid voltage
            if (cellV < minV) {
                minV = cellV;
                minIdx = i;
            }
            if (cellV > maxV) {
                maxV = cellV;
                maxIdx = i;
            }
        }
    }

    data.minCellVoltage = minV;
    data.maxCellVoltage = maxV;
    data.minCellIndex = minIdx;
    data.maxCellIndex = maxIdx;
    data.cellVoltageDelta = maxV - minV;
}

void BMSManager::updateMinMaxTemps() {
    float minT = 100.0f;
    float maxT = -100.0f;

    for (uint8_t i = 0; i < Battery::NUM_TEMP_SENSORS; i++) {
        float temp = data.moduleTemp[i] * 0.1f;  // 0.1°C to °C
        if (temp < minT) minT = temp;
        if (temp > maxT) maxT = temp;
    }

    data.minTemp = minT;
    data.maxTemp = maxT;
}

void BMSManager::updateSharedData() {
    // Update the global sharedBMSData structure
    BMSData basicData;
    basicData.soc = data.capacityPercent;
    //DEBUG_PRINTLN(basicData.soc);
    basicData.voltage = data.packVoltage;       // Already in 0.1V
    basicData.current = data.packCurrent;       // Already in 0.01A
    basicData.maxDischarge = 100;               // TODO: Get from BMS limits
    basicData.maxCharge = 50;                   // TODO: Get from BMS limits
    basicData.minCellVoltage = data.minCellVoltage;
    basicData.maxCellVoltage = data.maxCellVoltage;
    basicData.temperature = data.maxTemp;       // Use max temp

    sharedBMSData.set(basicData);

    // Set event flag
    if (data.dataValid) {
        xEventGroupSetBits(systemEvents, EVENT_BMS_ALIVE);
    }
}

void BMSManager::checkTimeout() {
    unsigned long now = millis();
    if (data.dataValid && (now - data.lastUpdateTime > Battery::BMS_TIMEOUT_MS)) {
        data.dataValid = false;
        xEventGroupClearBits(systemEvents, EVENT_BMS_ALIVE);
        DEBUG_PRINTLN("BMSManager: Timeout - BMS not responding");
    }
}

//=============================================================================
// STATUS QUERIES
//=============================================================================

bool BMSManager::isAlive() const {
    return data.dataValid && (millis() - data.lastUpdateTime < Battery::BMS_TIMEOUT_MS);
}

bool BMSManager::hasError() const {
    return data.overTemp || data.overCharge || data.overDischarge ||
           data.overCurrent || data.cellError;
}

bool BMSManager::canCharge() const {
    if (!isAlive()) return false;
    if (hasError()) return false;
    if (!data.chargeMOSEnabled) return false;
    if (data.maxTemp > Charger::CHARGE_TEMP_MAX) return false;
    if (data.minTemp < Charger::CHARGE_TEMP_MIN) return false;
    if (data.maxCellVoltage > Charger::CHARGE_VOLTAGE_MAX) return false;
    return true;
}

bool BMSManager::canDischarge() const {
    if (!isAlive()) return false;
    if (hasError()) return false;
    if (!data.dischargeMOSEnabled) return false;
    if (data.minCellVoltage < Battery::CELL_MIN_V) return false;
    return true;
}

void BMSManager::getChargingLimits(float& maxVoltage, float& maxCurrent, float& maxTemp) const {
    maxVoltage = Charger::CHARGE_VOLTAGE_BULK * Battery::NUM_CELLS;
    maxCurrent = Charger::CHARGE_CURRENT_MAX;
    maxTemp = Charger::CHARGE_TEMP_MAX;

    // Temperature derating
    if (data.maxTemp > 35.0f) {
        float derate = 1.0f - ((data.maxTemp - 35.0f) / 10.0f);
        if (derate < 0.5f) derate = 0.5f;
        maxCurrent *= derate;
    }
}

void BMSManager::populateErrorStatus(SystemErrorStatus& status) {
    unsigned long now = millis();

    // BMS Over Temperature
    if (data.overTemp) {
        status.bmsOverTemp.code = "BMS_OVERTEMP";
        status.bmsOverTemp.message = "Battery over-temperature detected";
        status.bmsOverTemp.severity = ErrorSeverity::CRITICAL;
        status.bmsOverTemp.active = true;
        if (status.bmsOverTemp.timestamp == 0) status.bmsOverTemp.timestamp = now;
    } else {
        status.bmsOverTemp.active = false;
    }

    // BMS Over Charge
    if (data.overCharge) {
        status.bmsOverCharge.code = "BMS_OVERCHARGE";
        status.bmsOverCharge.message = "Battery over-charging detected";
        status.bmsOverCharge.severity = ErrorSeverity::CRITICAL;
        status.bmsOverCharge.active = true;
        if (status.bmsOverCharge.timestamp == 0) status.bmsOverCharge.timestamp = now;
    } else {
        status.bmsOverCharge.active = false;
    }

    // BMS Cell Error
    if (data.cellError) {
        status.bmsCellError.code = "BMS_CELL_ERROR";
        status.bmsCellError.message = "Battery cell/string error detected";
        status.bmsCellError.severity = ErrorSeverity::CRITICAL;
        status.bmsCellError.active = true;
        if (status.bmsCellError.timestamp == 0) status.bmsCellError.timestamp = now;
    } else {
        status.bmsCellError.active = false;
    }

    // BMS Over Current
    if (data.overCurrent) {
        status.bmsOverCurrent.code = "BMS_OVERCURRENT";
        status.bmsOverCurrent.message = "Battery discharge over-current";
        status.bmsOverCurrent.severity = ErrorSeverity::CRITICAL;
        status.bmsOverCurrent.active = true;
        if (status.bmsOverCurrent.timestamp == 0) status.bmsOverCurrent.timestamp = now;
    } else {
        status.bmsOverCurrent.active = false;
    }

    // BMS Over Discharge
    if (data.overDischarge) {
        status.bmsOverDischarge.code = "BMS_OVERDISCHARGE";
        status.bmsOverDischarge.message = "Battery over-discharge detected";
        status.bmsOverDischarge.severity = ErrorSeverity::CRITICAL;
        status.bmsOverDischarge.active = true;
        if (status.bmsOverDischarge.timestamp == 0) status.bmsOverDischarge.timestamp = now;
    } else {
        status.bmsOverDischarge.active = false;
    }

    // BMS Temperature Negative
    if (data.tempNegative) {
        status.bmsTempNegative.code = "BMS_TEMP_NEGATIVE";
        status.bmsTempNegative.message = "Battery temperature below zero";
        status.bmsTempNegative.severity = ErrorSeverity::WARNING;
        status.bmsTempNegative.active = true;
        if (status.bmsTempNegative.timestamp == 0) status.bmsTempNegative.timestamp = now;
    } else {
        status.bmsTempNegative.active = false;
    }

    // BMS Communication Timeout
    if (!isAlive()) {
        status.bmsTimeout.code = "BMS_TIMEOUT";
        status.bmsTimeout.message = "BMS communication lost";
        status.bmsTimeout.severity = ErrorSeverity::CRITICAL;
        status.bmsTimeout.active = true;
        if (status.bmsTimeout.timestamp == 0) status.bmsTimeout.timestamp = now;
    } else {
        status.bmsTimeout.active = false;
    }

    // BMS Charge MOS Disabled
    if (!data.chargeMOSEnabled) {
        status.bmsChargeMOSDisabled.code = "BMS_CHARGE_MOS_OFF";
        status.bmsChargeMOSDisabled.message = "BMS charge MOS disabled";
        status.bmsChargeMOSDisabled.severity = ErrorSeverity::WARNING;
        status.bmsChargeMOSDisabled.active = true;
        if (status.bmsChargeMOSDisabled.timestamp == 0) status.bmsChargeMOSDisabled.timestamp = now;
    } else {
        status.bmsChargeMOSDisabled.active = false;
    }

    // BMS Discharge MOS Disabled
    if (!data.dischargeMOSEnabled) {
        status.bmsDischargeMOSDisabled.code = "BMS_DISCHARGE_MOS_OFF";
        status.bmsDischargeMOSDisabled.message = "BMS discharge MOS disabled";
        status.bmsDischargeMOSDisabled.severity = ErrorSeverity::WARNING;
        status.bmsDischargeMOSDisabled.active = true;
        if (status.bmsDischargeMOSDisabled.timestamp == 0) status.bmsDischargeMOSDisabled.timestamp = now;
    } else {
        status.bmsDischargeMOSDisabled.active = false;
    }
}

//=============================================================================
// SEND COMMAND TO BMS
//=============================================================================

void BMSManager::sendCommand(uint8_t command, uint16_t value) {
    if (canManager == nullptr) {
        DEBUG_PRINTLN("BMSManager: Cannot send command - CAN manager not set");
        return;
    }

    // BMS commands are 3 bytes: [command, value_high, value_low]
    uint8_t buf[3];
    buf[0] = command;
    buf[1] = (value >> 8) & 0xFF;  // High byte
    buf[2] = value & 0xFF;          // Low byte

    // Send on CAN_ID::BMS_RX (0xF4) - 3 bytes only
    bool success = canManager->sendCAN2Message(CAN_ID::BMS_RX, 3, buf);

#if DEBUG_CAN_MESSAGES
    if (success) {
        DEBUG_PRINTF("BMS CMD TX: 0x%02X value=0x%04X [%02X %02X %02X]\n",
                     command, value, buf[0], buf[1], buf[2]);
    } else {
        DEBUG_PRINTF("BMS CMD TX FAILED: 0x%02X\n", command);
    }
#endif
}
/*
void BMSManager::setChannelEnabled(bool enable) {
    uint16_t value = enable ? 0x0001 : 0x0002;
    sendCommand(BMSCommand::CHANNEL_CONTROL, value);
}
*/
void BMSManager::setBalancingEnabled(bool enable) {
    uint16_t value = enable ? 0x0002 : 0x0001;
    sendCommand(BMSCommand::BALANCE_CONTROL, value);
}

void BMSManager::requestConnection() {
    DEBUG_PRINTLN("BMSManager: Sending connection request...");
    sendCommand(BMSCommand::COMMUNICATION_CONNECT, 0x0002);
    vTaskDelay(pdMS_TO_TICKS(50));  // Small delay between commands
    sendCommand(BMSCommand::CAN_CONNECTED, 0x0002);
}

void BMSManager::sendConfiguration(uint8_t numCells) {
    DEBUG_PRINTLN("BMSManager: Sending BMS configuration...");
    #if UPLOAD_BMS_CONFIG
        // Small delay helper between commands
        auto delayCmd = []() { vTaskDelay(pdMS_TO_TICKS(150)); };

        // 0x01: Max discharge voltage (3.0V per cell) - Scale: voltage * 100
        uint16_t dischargeVoltage = (uint16_t)(Battery::CELL_MIN_V * 100);
        sendCommand(BMSCommand::DISCHARGE_PROTECTION_V, dischargeVoltage);
        delayCmd();
        DEBUG_PRINTF("  Discharge protection: %d.%02dV\n", dischargeVoltage / 100, dischargeVoltage % 100);

        // 0x02: Overcurrent protection (350A) - Scale: current * 10
        uint16_t maxCurrent = Battery::MAX_DISCHARGE_CURRENT * 10;
        sendCommand(BMSCommand::OVERCURRENT_PROTECTION, maxCurrent);
        delayCmd();
        DEBUG_PRINTF("  Overcurrent protection: %dA\n", Battery::MAX_DISCHARGE_CURRENT);

        // 0x03: Battery capacity (16Ah) - Scale: capacity * 10
        uint16_t capacity = Battery::CAPACITY_AH * 10;
        sendCommand(BMSCommand::BATTERY_CAPACITY, capacity);
        delayCmd();
        DEBUG_PRINTF("  Battery capacity: %dAh\n", Battery::CAPACITY_AH);

        // 0x04: Number of cells (4S for testing, 104S for production)
        sendCommand(BMSCommand::NUM_CELLS, numCells);
        delayCmd();
        DEBUG_PRINTF("  Number of cells: %dS\n", numCells);

        // 0x05: Max charge voltage (4.2V per cell) - Scale: voltage * 100
        uint16_t chargeVoltage = (uint16_t)(Battery::CELL_MAX_V * 100);
        sendCommand(BMSCommand::CHARGE_PROTECTION_V, chargeVoltage);
        delayCmd();
        DEBUG_PRINTF("  Charge protection: %d.%02dV\n", chargeVoltage / 100, chargeVoltage % 100);

        // 0x06: Max temperature (55°C) - Scale: temp * 10
        uint16_t maxTemp = (uint16_t)(Battery::MAX_TEMP_C * 10);
        sendCommand(BMSCommand::OVERTEMP_PROTECTION, maxTemp);
        delayCmd();
        DEBUG_PRINTF("  Over-temp protection: %d.%d°C\n", maxTemp / 10, maxTemp % 10);

        // 0x08: Auto-balance (0x0002=on, 0x0001=off)
        uint16_t balanceValue = Battery::BMS_AUTO_BALANCE ? 0x0002 : 0x0001;
        sendCommand(BMSCommand::BALANCE_CONTROL, balanceValue);
        delayCmd();
        DEBUG_PRINTF("  Auto-balance: %s\n", Battery::BMS_AUTO_BALANCE ? "ON" : "OFF");

        // 0x0C: Channel default state after power-on (0x0002=default on)
        uint16_t channelDefault = Battery::BMS_CHANNEL_DEFAULT ? 0x0002 : 0x0001;
        sendCommand(BMSCommand::CHANNEL_DEFAULT_STATE, channelDefault);
        delayCmd();
        DEBUG_PRINTF("  Channel default: %s\n", Battery::BMS_CHANNEL_DEFAULT ? "ON" : "OFF");

        // 0x0D: Host display switch (start with ON, will control based on gear state)
        sendCommand(BMSCommand::HOST_DISPLAY_SWITCH, 0x0001);
        delayCmd();
        DEBUG_PRINTLN("  Host display: ON (initial state, controlled by gear)");

        // 0x11: Balance start voltage (4.18V) - Scale: voltage * 100
        uint16_t balanceStartV = (uint16_t)(Battery::CELL_BALANCE_V * 100);
        sendCommand(BMSCommand::BALANCE_START_VOLTAGE, balanceStartV);
        delayCmd();
        DEBUG_PRINTF("  Balance start voltage: %d.%02dV\n", balanceStartV / 100, balanceStartV % 100);

        // 0x15: Auto-reset capacity (0x0001=on)
        uint16_t autoReset = Battery::BMS_AUTO_RESET_CAP ? 0x0001 : 0x0000;
        sendCommand(BMSCommand::AUTO_RESET_CAPACITY, autoReset);
        delayCmd();
        DEBUG_PRINTF("  Auto-reset capacity: %s\n", Battery::BMS_AUTO_RESET_CAP ? "ON" : "OFF");
    
        // 0x16: Pre-charge delay time (3 seconds)
        sendCommand(BMSCommand::PRECHARGE_DELAY_TIME, 0x0000);
        delayCmd();
        DEBUG_PRINTLN("  Pre-charge delay: 0s");

        // 0x18: Low temperature protection (6°C) - Scale: temp * 10
        uint16_t minTemp = (uint16_t)(Battery::MIN_TEMP_C * 10);
        sendCommand(BMSCommand::LOW_TEMP_PROTECTION, minTemp);
        delayCmd();
        DEBUG_PRINTF("  Low-temp protection: %d.%d°C\n", minTemp / 10, minTemp % 10);
        delayCmd();
        delayCmd();
    #endif
    // 0x07: Battery arming - Start DISARMED (will arm later via setChannelEnabled)
    sendCommand(BMSCommand::CHANNEL_CONTROL, 0x0001);  // 0x0002 = DISARMED
    vTaskDelay(pdMS_TO_TICKS(450));
    DEBUG_PRINTLN("  Battery arming: Prearmed (will arm when ready)");

    DEBUG_PRINTLN("BMSManager: Configuration complete!");
    
}

void BMSManager::setDisplayEnabled(bool enable) {
    // 0x0D: Host display switch (0x0001=on, 0x0002=off)
    uint16_t value = enable ? 0x0001 : 0x0002;
    sendCommand(BMSCommand::HOST_DISPLAY_SWITCH, value);
    DEBUG_PRINTF("BMSManager: Display %s\n", enable ? "ON" : "OFF");
}

void BMSManager::setNumCells(uint8_t numCells) {
    // 0x04: Number of cells
    sendCommand(BMSCommand::NUM_CELLS, numCells);
    data.numCellsConfigured = numCells;
    DEBUG_PRINTF("BMSManager: Cell count updated to %dS\n", numCells);
}

//=============================================================================
// PARSE HELPERS
//=============================================================================

uint16_t BMSManager::parseU16(const uint8_t* buf, uint8_t offset) const {
    return (buf[offset] << 8) | buf[offset + 1];
}

int16_t BMSManager::parseI16(const uint8_t* buf, uint8_t offset) const {
    return (int16_t)((buf[offset] << 8) | buf[offset + 1]);
}

bool BMSManager::parseBit(uint16_t value, uint8_t bit) const {
    return (value & (1 << bit)) != 0;
}
