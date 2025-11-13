#include "nlg5_manager.h"

//=============================================================================
// CONSTRUCTOR
//=============================================================================

NLG5Manager::NLG5Manager() {
    dataMutex = xSemaphoreCreateMutex();
    memset(&data, 0, sizeof(data));
    state = NLG5State::IDLE;
    enableCharger = false;
    errorClearCycle = false;
    targetVoltage = 0.0f;
    targetCurrent = 0.0f;
    maxMainsCurrent = Charger::MAINS_CURRENT_MAX;

    // Initialize new enable control variables
    chargeAllowed = false;
    batteryArmed = false;
    chargeAllowedPrev = false;
    lastDisableTime = 0;
    lastDisableReason = 0;

    // Initialize error clearing state
    errorClearState = ErrorClearState::IDLE;
    errorClearStartTime = 0;
}

//=============================================================================
// BEGIN
//=============================================================================

void NLG5Manager::begin() {
    DEBUG_PRINTLN("NLG5Manager: Initializing...");

    // Reset data structure
    memset(&data, 0, sizeof(data));
    data.dataValid = false;

    state = NLG5State::DISABLED_STATE;
    stateStartTime = millis();
    lastControlSentTime = 0;

    DEBUG_PRINTLN("NLG5Manager: Initialized");
}

//=============================================================================
// PROCESS INCOMING CAN MESSAGES
//=============================================================================

void NLG5Manager::processMessage(uint32_t id, const uint8_t* buf, uint8_t len) {
    if (len < 8) return;

    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        switch (id) {
            case CAN_ID::NLG5_ST:       // 0x610 - Status
                processStatus(buf);
                break;

            case CAN_ID::NLG5_ACT_I:    // 0x611 - Actuals I
                processActualsI(buf);
                break;

            case CAN_ID::NLG5_ACT_II:   // 0x612 - Actuals II
                processActualsII(buf);
                break;

            case CAN_ID::NLG5_TEMP:     // 0x613 - Temperatures
                processTemperatures(buf);
                break;

            case CAN_ID::NLG5_ERR:      // 0x614 - Errors
                processErrors(buf);
                break;

            default:
                // Unknown NLG5 message
                break;
        }

        data.lastUpdateTime = millis();
        data.dataValid = true;

        xSemaphoreGive(dataMutex);
    }
}

//=============================================================================
// MESSAGE PROCESSORS
//=============================================================================

void NLG5Manager::processStatus(const uint8_t* buf) {
    // NLG5_ST message (ID 1552 / 0x610) - 4 bytes
    // See DBC file for bit definitions

    // Combine all status bits (4 bytes = 32 bits)
    data.statusBits = ((uint32_t)buf[0] << 0) |
                      ((uint32_t)buf[1] << 8) |
                      ((uint32_t)buf[2] << 16) |
                      ((uint32_t)buf[3] << 24);

    // Parse individual status flags (see DBC comments)
    data.cpDetected = parseBit(data.statusBits, 0);         // S_CP_DT
    data.bypassDetected = parseBit(data.statusBits, 14);    // S_BPD_I/II
    data.errorActive = parseBit(data.statusBits, 6);        // S_ERR
    data.europeanMains = parseBit(data.statusBits, 3);      // S_EUM
    data.usMains1 = parseBit(data.statusBits, 2);           // S_UM_I
    data.usMains2 = parseBit(data.statusBits, 1);           // S_UM_II
    data.fanActive = parseBit(data.statusBits, 4);          // S_FAN
    data.warningActive = parseBit(data.statusBits, 5);      // S_WAR
    data.aux12VCharging = parseBit(data.statusBits, 24);    // S_AAC
    data.hardwareEnabled = parseBit(data.statusBits, 7);    // S_HE

    // Power limiting flags
    data.limitedBy_CP = parseBit(data.statusBits, 9);       // S_L_CP
    data.limitedBy_MainsCurrent = parseBit(data.statusBits, 11);  // S_L_MC
    data.limitedBy_MaxCurrent = parseBit(data.statusBits, 23);    // S_L_MC_MAX
    data.limitedBy_BatteryCurrent = parseBit(data.statusBits, 12); // S_L_OC
    data.limitedBy_BatteryVoltage = parseBit(data.statusBits, 13); // S_L_OV
    data.limitedBy_MaxPower = parseBit(data.statusBits, 8);       // S_L_PMAX
    data.limitedBy_PowerIndicator = parseBit(data.statusBits, 10); // S_L_PI

    // Temperature limiting flags
    data.limitedBy_TempBattery = parseBit(data.statusBits, 16);    // S_L_T_BATT
    data.limitedBy_TempCapacitor = parseBit(data.statusBits, 20);  // S_L_T_CPRIM
    data.limitedBy_TempDiode = parseBit(data.statusBits, 18);      // S_L_T_DIO
    data.limitedBy_TempPower = parseBit(data.statusBits, 19);      // S_L_T_POW
    data.limitedBy_TempTransformer = parseBit(data.statusBits, 17); // S_L_T_TR
}

void NLG5Manager::processActualsI(const uint8_t* buf) {
    // NLG5_ACT_I message (ID 1553 / 0x611) - 8 bytes
    // MC_ACT, MV_ACT, OC_ACT, OV_ACT

    // Mains current actual (0.01A resolution)
    data.mainsCurrentActual = parseU16(buf, 0) * 0.01f;

    // Mains voltage actual (0.1V resolution)
    data.mainsVoltageActual = parseU16(buf, 2) * 0.1f;

    // Output current actual (0.01A resolution, signed)
    data.batteryCurrentActual = parseI16(buf, 6) * 0.01f;

    // Output voltage actual (0.1V resolution)
    data.batteryVoltageActual = parseU16(buf, 4) * 0.1f;
}

void NLG5Manager::processActualsII(const uint8_t* buf) {
    // NLG5_ACT_II message (ID 1554 / 0x612) - 8 bytes
    // ABV, AHC_EXT, OC_BO, S_MC_M_CP, S_MC_M_PI

    // Aux battery voltage (0.1V resolution)
    data.auxBatteryVoltage = (buf[3] * 0.1f);

    // Ah counter external (0.01Ah resolution, signed)
    data.ahCounter = parseI16(buf, 4);

    // Output current booster (0.01A resolution)
    data.boosterCurrent = parseU16(buf, 6) * 0.01f;

    // Mains current limit from CP (0.1A resolution)
    data.mainsCurrent_CP = parseU16(buf, 0) * 0.1f;

    // Mains current limit from PI (0.1A resolution)
    data.mainsCurrent_PI = (buf[2] * 0.1f);
}

void NLG5Manager::processTemperatures(const uint8_t* buf) {
    // NLG5_TEMP message (ID 1555 / 0x613) - 8 bytes
    // All temps are signed 16-bit, 0.1°C resolution

    data.tempPowerStage = parseI16(buf, 0) * 0.1f;
    data.tempExt1 = parseI16(buf, 2) * 0.1f;
    data.tempExt2 = parseI16(buf, 4) * 0.1f;
    data.tempExt3 = parseI16(buf, 6) * 0.1f;
}

void NLG5Manager::processErrors(const uint8_t* buf) {
    // NLG5_ERR message (ID 1556 / 0x614) - 5 bytes (40 bits of error flags)

    // Combine error bits (5 bytes = 40 bits, store in 64-bit for easier access)
    data.errorBits = ((uint64_t)buf[0] << 0) |
                     ((uint64_t)buf[1] << 8) |
                     ((uint64_t)buf[2] << 16) |
                     ((uint64_t)buf[3] << 24) |
                     ((uint64_t)buf[4] << 32);

    // Parse critical error flags
    data.error_MainsFuse = parseBit64(data.errorBits, 0);           // E_MF
    data.error_OutputFuse = parseBit64(data.errorBits, 1);          // E_OF
    data.error_MainsOvervoltage1 = parseBit64(data.errorBits, 5);   // E_MOV_I
    data.error_MainsOvervoltage2 = parseBit64(data.errorBits, 6);   // E_MOV_II
    data.error_BatteryOvervoltage = parseBit64(data.errorBits, 7);  // E_OOV
    data.error_BatteryPolarity = parseBit64(data.errorBits, 15);    // E_B_P
    data.error_ShortCircuit = parseBit64(data.errorBits, 4);        // E_SC
    data.error_CANTimeout = parseBit64(data.errorBits, 17);         // E_C_TO
    data.error_CANOff = parseBit64(data.errorBits, 16);             // E_C_OFF

    // Temperature sensor errors
    data.error_TempSensor = parseBit64(data.errorBits, 14) ||       // E_T_C
                           parseBit64(data.errorBits, 12) ||       // E_T_DIO
                           parseBit64(data.errorBits, 10) ||       // E_T_EXT1
                           parseBit64(data.errorBits, 9) ||        // E_T_EXT2
                           parseBit64(data.errorBits, 8) ||        // E_T_EXT3
                           parseBit64(data.errorBits, 13) ||       // E_T_POW
                           parseBit64(data.errorBits, 11);         // E_T_TR

    // Checksum errors
    data.error_CRCChecksum = parseBit64(data.errorBits, 20) ||      // E_EP_CRC
                            parseBit64(data.errorBits, 21) ||      // E_ES_CRC
                            parseBit64(data.errorBits, 23) ||      // E_F_CRC
                            parseBit64(data.errorBits, 22);        // E_NV_CRC

    // Warning flags (from error message)
    data.warning_LowMainsVoltage = parseBit64(data.errorBits, 39);  // W_PL_MV
    data.warning_LowBatteryVoltage = parseBit64(data.errorBits, 38); // W_PL_BV
    data.warning_HighTemperature = parseBit64(data.errorBits, 37);  // W_PL_IT
    data.warning_ControlOutOfRange = parseBit64(data.errorBits, 36); // W_C_VOR
}

//=============================================================================
// UPDATE TASK
//=============================================================================

void NLG5Manager::update() {
    unsigned long now = millis();

    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        // Check timeout
        checkTimeout();

        // Handle error clearing state machine
        handleErrorClearing();

        // Determine if charger should be enabled based on all conditions
        bool shouldEnable = shouldEnableCharger();

        // Update enableCharger flag for use by external code (main.cpp/CANManager)
        enableCharger = shouldEnable;

        // Update state machine
        updateStateMachine();

        // Update shared data
        updateSharedData();

        xSemaphoreGive(dataMutex);
    }
}

//=============================================================================
// STATE MACHINE
//=============================================================================

void NLG5Manager::updateStateMachine() {
    unsigned long stateTime = millis() - stateStartTime;

    switch (state) {
        case NLG5State::IDLE:
            // Do nothing, waiting for start command
            break;

        case NLG5State::STARTING:
            // Wait for charger to become ready
            if (data.hardwareEnabled && !data.errorActive) {
                transitionToState(NLG5State::BULK);
            } else if (stateTime > 5000) {
                // Timeout waiting for charger ready
                DEBUG_PRINTLN("NLG5: Timeout waiting for ready");
                transitionToState(NLG5State::ERROR);
            }
            break;

        case NLG5State::BULK:
            // Constant current bulk charging
            // Transition to absorption when voltage reaches target
            if (data.batteryVoltageActual >= (targetVoltage - 1.0f)) {
                transitionToState(NLG5State::ABSORPTION);
            }
            break;

        case NLG5State::ABSORPTION:
            // Constant voltage absorption
            // Complete when current tapers below threshold
            if (data.batteryCurrentActual < Charger::CHARGE_CURRENT_TAPER) {
                transitionToState(NLG5State::COMPLETED);
            }
            break;

        case NLG5State::COMPLETED:
            // Charging complete
            stopCharging();
            break;

        case NLG5State::ERROR:
            // Error state - disable charger
            enableCharger = false;
            break;

        case NLG5State::DISABLED_STATE:
            // Disabled state
            break;
    }

    // Check for errors in any state
    if (state != NLG5State::ERROR && state != NLG5State::DISABLED_STATE) {
        if (data.errorActive || hasError()) {
            DEBUG_PRINTF("NLG5: Error detected (bits: 0x%llX)\n", data.errorBits);
            transitionToState(NLG5State::ERROR);
        }
    }
}

void NLG5Manager::transitionToState(NLG5State newState) {
    if (newState != state) {
        DEBUG_PRINTF("NLG5: State %d -> %d\n", (int)state, (int)newState);
        state = newState;
        stateStartTime = millis();
    }
}

//=============================================================================
// CONTROL FUNCTIONS
//=============================================================================

void NLG5Manager::startCharging(float targetV, float maxI, float maxMainsI) {
    DEBUG_PRINTF("NLG5: Start charging - %.1fV @ %.1fA\n", targetV, maxI);

    targetVoltage = targetV;
    targetCurrent = maxI;
    maxMainsCurrent = maxMainsI;
    enableCharger = true;

    transitionToState(NLG5State::STARTING);
}

void NLG5Manager::stopCharging() {
    DEBUG_PRINTLN("NLG5: Stop charging");

    enableCharger = false;
    targetVoltage = 0.0f;
    targetCurrent = 0.0f;

    transitionToState(NLG5State::IDLE);
}

void NLG5Manager::clearErrors() {
    DEBUG_PRINTLN("NLG5: Clearing errors");
    errorClearCycle = true;
    // The sendControl function will cycle the error clear bit 0-1-0
}

//=============================================================================
// SEND CONTROL MESSAGE
//=============================================================================

void NLG5Manager::sendControl() {
    // NLG5_CTL message (ID 1560 / 0x618) - 7 bytes
    uint8_t buf[8];
    memset(buf, 0, sizeof(buf));

    // Byte 0: Control flags
    uint8_t flags = 0;
    if (enableCharger) {
        flags |= Charger::CTL_FLAG_ENABLE;  // Bit 7: C_C_EN
    }

    if (errorClearCycle) {
        flags |= Charger::CTL_FLAG_CLEAR_ERROR;  // Bit 6: C_C_EL (cycle this)
        errorClearCycle = false;  // Clear after one send
    }

    buf[0] = flags;

    // Bytes 1-2: MC_MAX - Maximum mains current (0.1A resolution)
    uint16_t mainsCurrentCmd = (uint16_t)(maxMainsCurrent * 10.0f);
    buf[1] = (mainsCurrentCmd >> 8) & 0xFF;
    buf[2] = mainsCurrentCmd & 0xFF;

    // Bytes 3-4: OV_COM - Desired output voltage (0.1V resolution)
    uint16_t voltageCmd = (uint16_t)(targetVoltage * 10.0f);
    buf[3] = (voltageCmd >> 8) & 0xFF;
    buf[4] = voltageCmd & 0xFF;

    // Bytes 5-6: OC_COM - Desired output current (0.1A resolution)
    uint16_t currentCmd = (uint16_t)(targetCurrent * 10.0f);
    buf[5] = (currentCmd >> 8) & 0xFF;
    buf[6] = currentCmd & 0xFF;

    // Byte 7: Reserved
    buf[7] = 0;

    // Store what we commanded for diagnostics
    data.commandedVoltage = targetVoltage;
    data.commandedCurrent = targetCurrent;
    data.commandedMaxMainsCurrent = maxMainsCurrent;

    // TODO: Send via CAN manager
    // CANManager should handle sending this on CAN_ID::NLG5_CTL
}

//=============================================================================
// HELPER FUNCTIONS
//=============================================================================

void NLG5Manager::updateSharedData() {
    // Update the global sharedNLGData structure
    NLGData basicData;
    basicData.state = (uint8_t)state;
    basicData.dcVoltage = (uint16_t)data.batteryVoltageActual;
    basicData.dcCurrent = (uint16_t)data.batteryCurrentActual;
    basicData.connectorLocked = false;  // NLG5 doesn't have this
    basicData.temperature = data.tempPowerStage;

    sharedNLGData.set(basicData);

    // Set/clear event flag
    if (isCharging()) {
        xEventGroupSetBits(systemEvents, EVENT_CHARGER_CONNECTED);
    } else {
        xEventGroupClearBits(systemEvents, EVENT_CHARGER_CONNECTED);
    }
}

void NLG5Manager::checkTimeout() {
    unsigned long now = millis();
    if (data.dataValid && (now - data.lastUpdateTime > Charger::CHARGER_TIMEOUT_MS)) {
        data.dataValid = false;
        DEBUG_PRINTLN("NLG5Manager: Timeout - charger not responding");
        if (state != NLG5State::DISABLED_STATE && state != NLG5State::IDLE) {
            transitionToState(NLG5State::ERROR);
        }
    }
}

//=============================================================================
// STATUS QUERIES
//=============================================================================

bool NLG5Manager::isAlive() const {
    return data.dataValid &&
           (millis() - data.lastUpdateTime < Charger::CHARGER_TIMEOUT_MS);
}

bool NLG5Manager::hasError() const {
    return data.error_MainsFuse ||
           data.error_OutputFuse ||
           data.error_MainsOvervoltage1 ||
           data.error_MainsOvervoltage2 ||
           data.error_BatteryOvervoltage ||
           data.error_BatteryPolarity ||
           data.error_ShortCircuit ||
           data.error_CANTimeout ||
           data.error_CANOff;
}

bool NLG5Manager::isCharging() const {
    return (state == NLG5State::BULK || state == NLG5State::ABSORPTION) &&
           data.batteryCurrentActual > 1.0f;  // >1A means actively charging
}

float NLG5Manager::getChargingPower() const {
    return data.batteryVoltageActual * data.batteryCurrentActual;
}

//=============================================================================
// ENABLE CONTROL AND ERROR RECOVERY
//=============================================================================

void NLG5Manager::setChargeConditions(bool allowed, bool armed) {
    chargeAllowedPrev = chargeAllowed;
    chargeAllowed = allowed;
    batteryArmed = armed;

    // Detect when charge allowance is lost
    if (chargeAllowedPrev && !chargeAllowed) {
        // Charge allowance just went away - record disable time
        lastDisableTime = millis();
        lastDisableReason = CHARGE_ALLOW_TIMEOUT;
#if DEBUG_CHARGING
        DEBUG_PRINTLN("NLG5: Charge allowance lost, 2min timeout started");
#endif
    }
}

bool NLG5Manager::shouldEnableCharger() {
    // Check all preconditions for enabling charger

    // 1. Check charge allowance (from MCP A0 pin)
    if (!chargeAllowed) {
        return false;
    }

    // 2. Check battery armed state (contactors closed)
    if (!batteryArmed) {
        return false;
    }

    // 3. Check for active errors
    if (hasError()) {
        // Errors present - can't enable yet
        // Error clearing will be handled by handleErrorClearing()
        return false;
    }

    // 4. Check anti-oscillation timeout
    unsigned long now = millis();
    if (lastDisableTime > 0) {
        unsigned long timeSinceDisable = now - lastDisableTime;
        if (timeSinceDisable < lastDisableReason) {
            // Still within timeout period
            return false;
        } else {
            // Timeout expired, clear the disable time
            lastDisableTime = 0;
        }
    }

    // 5. Check if we're in error clearing state
    if (errorClearState == ErrorClearState::CLEARING ||
        errorClearState == ErrorClearState::WAITING) {
        // Still clearing errors, can't enable yet
        return false;
    }

    // 6. Check BMS data is valid (using basic BMSData from shared data)
    BMSData bms = sharedBMSData.get();
    // Note: Simple BMSData doesn't have dataValid flag, assume valid if we got here

    // 7. Check temperature limits (use basic temperature from BMSData)
    RuntimeConfigData config = sharedRuntimeConfig.get();
    if (bms.temperature > config.maxChargeTemp || bms.temperature < config.minChargeTemp) {
        // Temperature out of range - record timeout
        if (lastDisableTime == 0 || lastDisableReason != TEMP_RETRY_TIMEOUT) {
            lastDisableTime = millis();
            lastDisableReason = TEMP_RETRY_TIMEOUT;
        }
        return false;
    }

    // All conditions met - can enable
    return true;
}

void NLG5Manager::handleErrorClearing() {
    // Manage the error clearing state machine

    switch (errorClearState) {
        case ErrorClearState::IDLE:
            // Check if errors are present
            if (hasError()) {
                // Errors detected - start clearing process
                errorClearState = ErrorClearState::CLEARING;
                errorClearStartTime = millis();
#if DEBUG_CHARGING
                DEBUG_PRINTLN("NLG5: Starting error clear cycle");
#endif
            }
            break;

        case ErrorClearState::CLEARING:
            // Error clear bit is set to 1 in sendControl()
            // Wait for 100ms minimum before clearing bit
            if (millis() - errorClearStartTime >= ERROR_CLEAR_DURATION) {
                errorClearState = ErrorClearState::WAITING;
#if DEBUG_CHARGING
                DEBUG_PRINTLN("NLG5: Error clear bit cycling complete, waiting for next cycle");
#endif
            }
            break;

        case ErrorClearState::WAITING:
            // Error clear bit has been cycled back to 0
            // Check if errors are cleared on next update cycle
            if (!hasError()) {
                // Errors cleared successfully
                errorClearState = ErrorClearState::IDLE;
                lastDisableTime = millis();
                lastDisableReason = ERROR_RETRY_TIMEOUT;
#if DEBUG_CHARGING
                DEBUG_PRINTLN("NLG5: Errors cleared, 30s retry timeout started");
#endif
            } else {
                // Errors still present after clearing attempt
                errorClearState = ErrorClearState::FAILED;
#if DEBUG_CHARGING
                DEBUG_PRINTLN("NLG5: Error clear failed, persistent errors detected");
#endif
            }
            break;

        case ErrorClearState::FAILED:
            // Error clearing failed - errors persist
            // Transition to ERROR state will be handled by state machine
            // Reset to IDLE so we can try again if conditions change
            errorClearState = ErrorClearState::IDLE;
            break;
    }
}

void NLG5Manager::populateErrorStatus(SystemErrorStatus& status) {
    unsigned long now = millis();

    // Critical Errors
    status.chargerMainsFuse.active = data.error_MainsFuse;
    if (data.error_MainsFuse) {
        status.chargerMainsFuse.code = "CHG_MAINS_FUSE";
        status.chargerMainsFuse.message = "Charger mains fuse defective";
        status.chargerMainsFuse.severity = ErrorSeverity::CRITICAL;
        if (status.chargerMainsFuse.timestamp == 0) status.chargerMainsFuse.timestamp = now;
    }

    status.chargerOutputFuse.active = data.error_OutputFuse;
    if (data.error_OutputFuse) {
        status.chargerOutputFuse.code = "CHG_OUTPUT_FUSE";
        status.chargerOutputFuse.message = "Charger output fuse defective";
        status.chargerOutputFuse.severity = ErrorSeverity::CRITICAL;
        if (status.chargerOutputFuse.timestamp == 0) status.chargerOutputFuse.timestamp = now;
    }

    status.chargerShortCircuit.active = data.error_ShortCircuit;
    if (data.error_ShortCircuit) {
        status.chargerShortCircuit.code = "CHG_SHORT";
        status.chargerShortCircuit.message = "Charger short circuit";
        status.chargerShortCircuit.severity = ErrorSeverity::CRITICAL;
        if (status.chargerShortCircuit.timestamp == 0) status.chargerShortCircuit.timestamp = now;
    }

    status.chargerMainsOV.active = data.error_MainsOvervoltage1 || data.error_MainsOvervoltage2;
    if (status.chargerMainsOV.active) {
        status.chargerMainsOV.code = "CHG_MAINS_OV";
        status.chargerMainsOV.message = "Charger mains overvoltage";
        status.chargerMainsOV.severity = ErrorSeverity::CRITICAL;
        if (status.chargerMainsOV.timestamp == 0) status.chargerMainsOV.timestamp = now;
    }

    status.chargerBatteryOV.active = data.error_BatteryOvervoltage;
    if (data.error_BatteryOvervoltage) {
        status.chargerBatteryOV.code = "CHG_BATT_OV";
        status.chargerBatteryOV.message = "Charger battery overvoltage";
        status.chargerBatteryOV.severity = ErrorSeverity::CRITICAL;
        if (status.chargerBatteryOV.timestamp == 0) status.chargerBatteryOV.timestamp = now;
    }

    status.chargerPolarity.active = data.error_BatteryPolarity;
    if (data.error_BatteryPolarity) {
        status.chargerPolarity.code = "CHG_POLARITY";
        status.chargerPolarity.message = "Charger wrong polarity";
        status.chargerPolarity.severity = ErrorSeverity::CRITICAL;
        if (status.chargerPolarity.timestamp == 0) status.chargerPolarity.timestamp = now;
    }

    status.chargerCANTimeout.active = data.error_CANTimeout;
    if (data.error_CANTimeout) {
        status.chargerCANTimeout.code = "CHG_CAN_TO";
        status.chargerCANTimeout.message = "Charger CAN timeout";
        status.chargerCANTimeout.severity = ErrorSeverity::CRITICAL;
        if (status.chargerCANTimeout.timestamp == 0) status.chargerCANTimeout.timestamp = now;
    }

    status.chargerCANOff.active = data.error_CANOff;
    if (data.error_CANOff) {
        status.chargerCANOff.code = "CHG_CAN_OFF";
        status.chargerCANOff.message = "Charger CAN bus off";
        status.chargerCANOff.severity = ErrorSeverity::CRITICAL;
        if (status.chargerCANOff.timestamp == 0) status.chargerCANOff.timestamp = now;
    }

    status.chargerTempSensor.active = data.error_TempSensor;
    if (data.error_TempSensor) {
        status.chargerTempSensor.code = "CHG_TEMP_SNS";
        status.chargerTempSensor.message = "Charger temp sensor error";
        status.chargerTempSensor.severity = ErrorSeverity::CRITICAL;
        if (status.chargerTempSensor.timestamp == 0) status.chargerTempSensor.timestamp = now;
    }

    status.chargerCRCError.active = data.error_CRCChecksum;
    if (data.error_CRCChecksum) {
        status.chargerCRCError.code = "CHG_CRC";
        status.chargerCRCError.message = "Charger CRC error";
        status.chargerCRCError.severity = ErrorSeverity::CRITICAL;
        if (status.chargerCRCError.timestamp == 0) status.chargerCRCError.timestamp = now;
    }

    status.chargerTimeout.active = !isAlive();
    if (!isAlive()) {
        status.chargerTimeout.code = "CHG_TIMEOUT";
        status.chargerTimeout.message = "Charger communication lost";
        status.chargerTimeout.severity = ErrorSeverity::CRITICAL;
        if (status.chargerTimeout.timestamp == 0) status.chargerTimeout.timestamp = now;
    }

    // Warnings
    status.chargerLowMainsV.active = data.warning_LowMainsVoltage;
    if (data.warning_LowMainsVoltage) {
        status.chargerLowMainsV.code = "CHG_LOW_MAINS";
        status.chargerLowMainsV.message = "Charger mains voltage low";
        status.chargerLowMainsV.severity = ErrorSeverity::WARNING;
        if (status.chargerLowMainsV.timestamp == 0) status.chargerLowMainsV.timestamp = now;
    }

    status.chargerLowBattV.active = data.warning_LowBatteryVoltage;
    if (data.warning_LowBatteryVoltage) {
        status.chargerLowBattV.code = "CHG_LOW_BATT";
        status.chargerLowBattV.message = "Charger battery voltage low";
        status.chargerLowBattV.severity = ErrorSeverity::WARNING;
        if (status.chargerLowBattV.timestamp == 0) status.chargerLowBattV.timestamp = now;
    }

    status.chargerHighTemp.active = data.warning_HighTemperature;
    if (data.warning_HighTemperature) {
        status.chargerHighTemp.code = "CHG_HIGH_TEMP";
        status.chargerHighTemp.message = "Charger overtemperature";
        status.chargerHighTemp.severity = ErrorSeverity::WARNING;
        if (status.chargerHighTemp.timestamp == 0) status.chargerHighTemp.timestamp = now;
    }

    status.chargerControlOOR.active = data.warning_ControlOutOfRange;
    if (data.warning_ControlOutOfRange) {
        status.chargerControlOOR.code = "CHG_CTRL_OOR";
        status.chargerControlOOR.message = "Charger control out of range";
        status.chargerControlOOR.severity = ErrorSeverity::WARNING;
        if (status.chargerControlOOR.timestamp == 0) status.chargerControlOOR.timestamp = now;
    }
}

//=============================================================================
// PARSE HELPERS
//=============================================================================

uint16_t NLG5Manager::parseU16(const uint8_t* buf, uint8_t offset) const {
    return (buf[offset] << 8) | buf[offset + 1];
}

int16_t NLG5Manager::parseI16(const uint8_t* buf, uint8_t offset) const {
    return (int16_t)((buf[offset] << 8) | buf[offset + 1]);
}

bool NLG5Manager::parseBit(uint32_t value, uint8_t bit) const {
    return (value & (1UL << bit)) != 0;
}

bool NLG5Manager::parseBit64(uint64_t value, uint8_t bit) const {
    return (value & (1ULL << bit)) != 0;
}
