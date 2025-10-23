#include "bms_manager.h"

//=============================================================================
// BMS COMMAND CODES (for sendCommand function)
//=============================================================================
namespace BMSCommand {
    constexpr uint8_t DISCHARGE_PROTECTION_V = 0x01;
    constexpr uint8_t OVERCURRENT_PROTECTION = 0x02;
    constexpr uint8_t BATTERY_CAPACITY       = 0x03;
    constexpr uint8_t NUM_CELLS              = 0x04;
    constexpr uint8_t CHARGE_PROTECTION_V    = 0x05;
    constexpr uint8_t OVERTEMP_PROTECTION    = 0x06;
    constexpr uint8_t CHANNEL_CONTROL        = 0x07;  // 0x0001=on, 0x0002=off
    constexpr uint8_t BALANCE_CONTROL        = 0x08;  // 0x0002=on, 0x0001=off
    constexpr uint8_t RESET_DISCHARGE_CAP    = 0x09;
    constexpr uint8_t CHARGE_RECOVERY_V      = 0x0A;
    constexpr uint8_t DISCHARGE_RECOVERY_V   = 0x0B;
    constexpr uint8_t COMMUNICATION_CONNECT  = 0x10;  // 0x0002=connected
    constexpr uint8_t CAN_OK                 = 0x1C;  // 0x0002=connected
}

//=============================================================================
// CONSTRUCTOR
//=============================================================================

BMSManager::BMSManager() {
    dataMutex = xSemaphoreCreateMutex();
    memset(&data, 0, sizeof(data));
    memset(receivedGroups, 0, sizeof(receivedGroups));
}

//=============================================================================
// BEGIN
//=============================================================================

void BMSManager::begin() {
    DEBUG_PRINTLN("BMSManager: Initializing...");

    // Reset data structure
    memset(&data, 0, sizeof(data));
    data.dataValid = false;
    data.numCellsConfigured = Battery::NUM_CELLS;

    // Request connection with BMS
    requestConnection();

    DEBUG_PRINTLN("BMSManager: Initialized");
}

//=============================================================================
// PROCESS INCOMING CAN MESSAGE
//=============================================================================

void BMSManager::processMessage(uint32_t id, const uint8_t* buf, uint8_t len) {
    // BMS sends on 0xF5, check if this is a BMS message
    if (id != CAN_ID::BMS_TX || len < 8) {
        return;
    }

    // Get message group ID from first byte
    uint8_t groupId = buf[0];

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
    basicData.soc = data.capacityPercent / 10;  // 0.1% to %
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

//=============================================================================
// SEND COMMAND TO BMS
//=============================================================================

void BMSManager::sendCommand(uint8_t command, uint16_t value) {
    uint8_t buf[8];
    buf[0] = command;
    buf[1] = (value >> 8) & 0xFF;  // High byte
    buf[2] = value & 0xFF;          // Low byte
    buf[3] = 0;
    buf[4] = 0;
    buf[5] = 0;
    buf[6] = 0;
    buf[7] = 0;

    // TODO: Send via CAN manager
    // CANManager should handle sending this on CAN_ID::BMS_RX (0xF4)
}

void BMSManager::setChannelEnabled(bool enable) {
    uint16_t value = enable ? 0x0001 : 0x0002;
    sendCommand(BMSCommand::CHANNEL_CONTROL, value);
}

void BMSManager::setBalancingEnabled(bool enable) {
    uint16_t value = enable ? 0x0002 : 0x0001;
    sendCommand(BMSCommand::BALANCE_CONTROL, value);
}

void BMSManager::requestConnection() {
    sendCommand(BMSCommand::COMMUNICATION_CONNECT, 0x0002);
    sendCommand(BMSCommand::CAN_OK, 0x0002);
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
