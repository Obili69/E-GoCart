#include "charging_coordinator.h"

//=============================================================================
// CONSTRUCTOR
//=============================================================================

ChargingCoordinator::ChargingCoordinator(BMSManager& bmsRef, NLG5Manager& nlg5Ref)
    : bms(bmsRef), nlg5(nlg5Ref) {
    statusMutex = xSemaphoreCreateMutex();
    memset(&status, 0, sizeof(status));
    status.state = ChargingState::IDLE;
}

//=============================================================================
// BEGIN
//=============================================================================

void ChargingCoordinator::begin() {
    DEBUG_PRINTLN("ChargingCoordinator: Initializing...");

    status.state = ChargingState::IDLE;
    status.safetyOK = false;
    status.errorMessage = "";

    lastSafetyCheckTime = 0;
    chargeStartTime = 0;
    energyAccumulator = 0.0f;

    DEBUG_PRINTLN("ChargingCoordinator: Initialized");
}

//=============================================================================
// UPDATE TASK
//=============================================================================

void ChargingCoordinator::update() {
    if (xSemaphoreTake(statusMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        // Perform safety checks every 500ms
        unsigned long now = millis();
        if (now - lastSafetyCheckTime >= 500) {
            status.safetyOK = performSafetyChecks();
            lastSafetyCheckTime = now;
        }

        // Update state machine
        updateStateMachine();

        // Update statistics
        updateStatistics();

        xSemaphoreGive(statusMutex);
    }
}

//=============================================================================
// CHARGING CONTROL
//=============================================================================

bool ChargingCoordinator::startCharging() {
    DEBUG_PRINTLN("ChargingCoordinator: Start charging requested");

    // Perform initial safety checks
    if (!performSafetyChecks()) {
        DEBUG_PRINTF("ChargingCoordinator: Safety check failed: %s\n",
                    status.errorMessage.c_str());
        return false;
    }

    // Calculate charging parameters from BMS
    calculateChargingParameters();

    // Reset session statistics
    chargeStartTime = millis();
    energyAccumulator = 0.0f;
    status.sessionStartTime = chargeStartTime;
    status.energyCharged = 0.0f;

    // Transition to precheck state
    transitionToState(ChargingState::PRECHECK);

    DEBUG_PRINTF("ChargingCoordinator: Starting - Target %.1fV @ %.1fA\n",
                maxChargeVoltage, maxChargeCurrent);

    return true;
}

void ChargingCoordinator::stopCharging() {
    DEBUG_PRINTLN("ChargingCoordinator: Stop charging requested");

    // Stop the charger
    nlg5.stopCharging();

    // Transition to idle
    transitionToState(ChargingState::IDLE);
}

//=============================================================================
// STATE MACHINE
//=============================================================================

void ChargingCoordinator::updateStateMachine() {
    unsigned long stateTime = millis() - status.sessionStartTime;

    switch (status.state) {
        case ChargingState::IDLE:
            // Nothing to do
            break;

        case ChargingState::PRECHECK:
            // Perform final safety checks before starting
            if (!status.safetyOK) {
                transitionToState(ChargingState::ERROR);
                break;
            }

            // All good, start the charger
            calculateChargingParameters();
            {
                RuntimeConfigData config = sharedRuntimeConfig.get();
                nlg5.startCharging(maxChargeVoltage, maxChargeCurrent, config.mainsCurrentLimit);
            }
            transitionToState(ChargingState::STARTING);
            break;

        case ChargingState::STARTING:
            // Wait for charger to start
            if (nlg5.getState() == NLG5State::BULK) {
                transitionToState(ChargingState::BULK_CHARGE);
            } else if (nlg5.getState() == NLG5State::ERROR) {
                status.errorMessage = "Charger failed to start";
                transitionToState(ChargingState::ERROR);
            }
            break;

        case ChargingState::BULK_CHARGE:
            // Constant current phase
            updateChargingSetpoints();

            // Check if charger transitioned to absorption
            if (nlg5.getState() == NLG5State::ABSORPTION) {
                transitionToState(ChargingState::ABSORPTION);
            }

            // Safety check
            if (!status.safetyOK) {
                transitionToState(ChargingState::ERROR);
            }
            break;

        case ChargingState::ABSORPTION:
            // Constant voltage phase
            updateChargingSetpoints();

            // Check if charger reports complete
            if (nlg5.getState() == NLG5State::COMPLETED) {
                transitionToState(ChargingState::COMPLETE);
            }

            // Safety check
            if (!status.safetyOK) {
                transitionToState(ChargingState::ERROR);
            }
            break;

        case ChargingState::BALANCING:
            // Optional balancing phase (not implemented yet)
            transitionToState(ChargingState::COMPLETE);
            break;

        case ChargingState::COMPLETE:
            // Charging complete - stop charger
            nlg5.stopCharging();
            DEBUG_PRINTF("ChargingCoordinator: Charging complete - %.1f Wh in %.1f min\n",
                        status.energyCharged, (stateTime / 60000.0f));
            transitionToState(ChargingState::IDLE);
            break;

        case ChargingState::ERROR:
            // Error state - stop charger
            nlg5.stopCharging();
            DEBUG_PRINTF("ChargingCoordinator: Error - %s\n",
                        status.errorMessage.c_str());
            break;

        case ChargingState::DISABLED_STATE:
            // Disabled
            break;
    }
}

void ChargingCoordinator::transitionToState(ChargingState newState) {
    if (newState != status.state) {
        DEBUG_PRINTF("ChargingCoordinator: State %d -> %d\n",
                    (int)status.state, (int)newState);
        status.state = newState;
    }
}

//=============================================================================
// SAFETY CHECKS
//=============================================================================

bool ChargingCoordinator::performSafetyChecks() {
    // Check BMS safety
    if (!checkBMSSafety()) {
        return false;
    }

    // Check charger safety
    if (!checkChargerSafety()) {
        return false;
    }

    // Check temperature safety
    if (!checkTemperatureSafety()) {
        return false;
    }

    // Check voltage safety
    if (!checkVoltageSafety()) {
        return false;
    }

    // Check timeout safety
    if (!checkTimeoutSafety()) {
        return false;
    }

    // All checks passed
    status.errorMessage = "";
    return true;
}

bool ChargingCoordinator::checkBMSSafety() {
    if (!bms.isAlive()) {
        status.errorMessage = "BMS not responding";
        return false;
    }

    if (bms.hasError()) {
        status.errorMessage = "BMS error detected";
        return false;
    }

    if (!bms.canCharge()) {
        status.errorMessage = "BMS does not allow charging";
        return false;
    }

    return true;
}

bool ChargingCoordinator::checkChargerSafety() {
    // Only check charger if we're actively charging
    if (status.state == ChargingState::IDLE ||
        status.state == ChargingState::PRECHECK) {
        return true;
    }

    if (!nlg5.isAlive()) {
        status.errorMessage = "Charger not responding";
        return false;
    }

    if (nlg5.hasError()) {
        status.errorMessage = "Charger error detected";
        return false;
    }

    return true;
}

bool ChargingCoordinator::checkTemperatureSafety() {
    BMSDataExtended bmsData = bms.getData();

    // Check BMS temperature
    if (bmsData.maxTemp > Charger::CHARGE_TEMP_MAX) {
        status.errorMessage = "Battery temperature too high";
        return false;
    }

    if (bmsData.minTemp < Charger::CHARGE_TEMP_MIN) {
        status.errorMessage = "Battery temperature too low";
        return false;
    }

    // Check charger temperature
    NLG5DataExtended nlg5Data = nlg5.getData();
    if (nlg5Data.tempPowerStage > Charger::CHARGER_TEMP_MAX) {
        status.errorMessage = "Charger temperature too high";
        return false;
    }

    return true;
}

bool ChargingCoordinator::checkVoltageSafety() {
    BMSDataExtended bmsData = bms.getData();

    // Check for overvoltage
    if (bmsData.maxCellVoltage > Charger::CHARGE_VOLTAGE_MAX) {
        status.errorMessage = "Cell overvoltage detected";
        return false;
    }

    // Check for cell imbalance
    if (bmsData.cellVoltageDelta > Battery::MAX_CELL_DELTA) {
        status.errorMessage = "Excessive cell voltage imbalance";
        return false;
    }

    return true;
}

bool ChargingCoordinator::checkTimeoutSafety() {
    // Check if charging has exceeded maximum time
    if (status.state != ChargingState::IDLE &&
        status.state != ChargingState::PRECHECK) {
        unsigned long chargeDuration = millis() - chargeStartTime;
        if (chargeDuration > Charger::CHARGE_TIMEOUT_MS) {
            status.errorMessage = "Charge timeout exceeded";
            return false;
        }
    }

    return true;
}

//=============================================================================
// CHARGING LOGIC
//=============================================================================

void ChargingCoordinator::calculateChargingParameters() {
    // Get runtime configuration for charging limits
    RuntimeConfigData config = sharedRuntimeConfig.get();

    // Get limits from BMS
    float bmsMaxVoltage, bmsMaxCurrent, bmsMaxTemp;
    bms.getChargingLimits(bmsMaxVoltage, bmsMaxCurrent, bmsMaxTemp);

    // Use the more restrictive limits between BMS and user configuration
    maxChargeVoltage = getMaxChargePackVoltage(config);  // From runtime config
    if (bmsMaxVoltage < maxChargeVoltage) {
        maxChargeVoltage = bmsMaxVoltage;  // BMS is more restrictive
    }

    maxChargeCurrent = config.maxChargeCurrent;  // From runtime config
    if (bmsMaxCurrent < maxChargeCurrent) {
        maxChargeCurrent = bmsMaxCurrent;  // BMS is more restrictive
    }

    maxChargeTemp = config.maxChargeTemp;  // From runtime config
    if (bmsMaxTemp < maxChargeTemp) {
        maxChargeTemp = bmsMaxTemp;  // BMS is more restrictive
    }

    // Apply safety margins
    maxChargeVoltage *= 0.99f;  // 1% margin on voltage
    maxChargeCurrent *= 0.95f;  // 5% margin on current

    // Clamp to NLG5 hardware limits
    if (maxChargeVoltage > 520.0f) {  // NLG5 max output voltage
        maxChargeVoltage = 520.0f;
    }

    if (maxChargeCurrent > 12.5f) {  // NLG5 max output current
        maxChargeCurrent = 12.5f;
    }

    DEBUG_PRINTF("ChargingCoordinator: Calculated limits - %.1fV @ %.1fA (Mains: %.1fA)\n",
                maxChargeVoltage, maxChargeCurrent, config.mainsCurrentLimit);
}

void ChargingCoordinator::updateChargingSetpoints() {
    // Recalculate parameters in case BMS limits changed (temp derating, etc.)
    calculateChargingParameters();

    // Update charger setpoints if they changed significantly
    float currentTarget = status.targetCurrent;
    float voltageTarget = status.targetVoltage;

    if (fabs(maxChargeCurrent - currentTarget) > 1.0f ||
        fabs(maxChargeVoltage - voltageTarget) > 1.0f) {
        RuntimeConfigData config = sharedRuntimeConfig.get();
        nlg5.startCharging(maxChargeVoltage, maxChargeCurrent, config.mainsCurrentLimit);
    }
}

void ChargingCoordinator::updateStatistics() {
    // Get actual values from charger
    NLG5DataExtended nlg5Data = nlg5.getData();
    BMSDataExtended bmsData = bms.getData();

    status.actualVoltage = nlg5Data.batteryVoltageActual;
    status.actualCurrent = nlg5Data.batteryCurrentActual;
    status.chargePower = nlg5.getChargingPower();
    status.socPercent = bmsData.capacityPercent / 10;  // 0.1% to %
    status.targetVoltage = maxChargeVoltage;
    status.targetCurrent = maxChargeCurrent;

    // Integrate energy (trapezoidal integration)
    if (status.state == ChargingState::BULK_CHARGE ||
        status.state == ChargingState::ABSORPTION) {
        // Power (W) * time (ms) / 1000 / 3600 = Wh
        float deltaEnergy = status.chargePower * 0.1f / 3600.0f;  // 100ms update rate
        energyAccumulator += deltaEnergy;
        status.energyCharged = energyAccumulator;
    }

    // Estimate time remaining
    status.timeRemaining = estimateTimeRemaining();
}

float ChargingCoordinator::estimateTimeRemaining() const {
    BMSDataExtended bmsData = bms.getData();

    // Calculate remaining capacity
    float remainingAh = bmsData.packCapacityAh - bmsData.usedCapacityAh;

    // Estimate time based on current charge rate
    if (status.actualCurrent > 0.1f) {
        return (remainingAh / status.actualCurrent) * 60.0f;  // minutes
    }

    return 0.0f;
}

//=============================================================================
// STATUS QUERIES
//=============================================================================

bool ChargingCoordinator::isCharging() const {
    return status.state == ChargingState::BULK_CHARGE ||
           status.state == ChargingState::ABSORPTION ||
           status.state == ChargingState::BALANCING;
}

bool ChargingCoordinator::isSafeToCharge() const {
    return status.safetyOK;
}
