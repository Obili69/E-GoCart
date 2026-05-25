#include "contactor_manager.h"
#include "bms_manager.h"
#include "input_manager.h"
#include "nlg5_manager.h"
#include "data_structures.h"

//=============================================================================
// CONSTRUCTOR
//=============================================================================

ContactorManager::ContactorManager()
    : bmsManager(nullptr)
    , inputManager(nullptr)
    , nlg5Manager(nullptr)
    , currentState(ContactorState::OPEN)
    , previousState(ContactorState::OPEN)
    , lastError(ContactorError::NONE)
    , stateEntryTime(0)
    , currentCheckStartTime(0)
{
}

//=============================================================================
// INITIALIZATION
//=============================================================================

void ContactorManager::begin(BMSManager* bmsMgr, InputManager* inputMgr, NLG5Manager* nlgMgr) {
    DEBUG_PRINTLN("ContactorManager: Initializing...");

    bmsManager = bmsMgr;
    inputManager = inputMgr;
    nlg5Manager = nlgMgr;

    pinMode(Pins::HV_MINUS_CONTACTOR,  OUTPUT);
    pinMode(Pins::HV_PLUS_CONTACTOR,   OUTPUT);
    pinMode(Pins::PRECHARGE_CONTACTOR, OUTPUT);
    pinMode(Pins::CHARGE_CONTACTOR,    OUTPUT);

    openAllHardware();

    DEBUG_PRINTLN("ContactorManager: Initialized - all contactors OPEN");
}

//=============================================================================
// UPDATE STATE MACHINE
//=============================================================================

void ContactorManager::update() {
    switch (currentState) {
        case ContactorState::OPEN:
            handleOpen();
            break;

        case ContactorState::CHARGE_PRECHARGING:
            handleChargePrecharging();
            break;

        case ContactorState::CHARGE_ARMED:
            handleChargeArmed();
            break;

        case ContactorState::DISCHARGE_PRECHARGING:
            handleDischargePrecharging();
            break;

        case ContactorState::DISCHARGE_ARMED:
            handleDischargeArmed();
            break;

        case ContactorState::ERROR:
            handleError();
            break;
    }
}

//=============================================================================
// STATE HANDLERS
//=============================================================================

void ContactorManager::handleOpen() {
    // Idle state - all contactors open
    // Waiting for startChargeSequence() or startDischargeSequence()
}

void ContactorManager::handleChargePrecharging() {
    unsigned long elapsed = millis() - stateEntryTime;

    if (!inputManager->isChargeAllowed()) {
        DEBUG_PRINTLN("ContactorManager: Charge allowance violated during precharge!");
        setError(ContactorError::CHARGE_ALLOW_VIOLATED);
        return;
    }

    if (elapsed >= Timing::PRECHARGE_DELAY_MS) {
        if (!verifyPrechargeVoltage(true)) {
            if (elapsed >= Timing::PRECHARGE_TIMEOUT) {
                DEBUG_PRINTLN("ContactorManager: Charge precharge TIMEOUT - bus voltage not reached!");
                setError(ContactorError::PRECHARGE_TIMEOUT);
            }
            return;
        }

        DEBUG_PRINTLN("ContactorManager: Charge precharge verified, closing HV+...");
        closeHVPlus();
        delay(100);
        openPrecharge();

        DEBUG_PRINTLN("ContactorManager: Charge path ARMED");
        transitionTo(ContactorState::CHARGE_ARMED);
    }
}

void ContactorManager::handleChargeArmed() {
    if (!inputManager->isChargeAllowed()) {
        DEBUG_PRINTLN("ContactorManager: Charge allowance violated! Disabling charge...");
        DEBUG_PRINTLN("ContactorManager: Waiting for charge current to reach zero...");

        if (!waitForCurrentZero(Timing::CURRENT_ZERO_TIMEOUT)) {
            DEBUG_PRINTLN("ContactorManager: ERROR - Current did not reach zero in time!");
            openAllHardware();
            transitionTo(ContactorState::OPEN);
            setError(ContactorError::CURRENT_NOT_ZERO);
        }
    }
}

void ContactorManager::handleDischargePrecharging() {
    unsigned long elapsed = millis() - stateEntryTime;

    if (!inputManager->isDischargeAllowed()) {
        DEBUG_PRINTLN("ContactorManager: Discharge allowance violated during precharge!");
        setError(ContactorError::DISCHARGE_ALLOW_VIOLATED);
        return;
    }

    if (elapsed >= Timing::PRECHARGE_DELAY_MS) {
        if (!verifyPrechargeVoltage(false)) {
            if (elapsed >= Timing::PRECHARGE_TIMEOUT) {
                DEBUG_PRINTLN("ContactorManager: Drive precharge TIMEOUT - bus voltage not reached!");
                setError(ContactorError::PRECHARGE_TIMEOUT);
            }
            return;
        }

        DEBUG_PRINTLN("ContactorManager: Drive precharge verified, closing HV+...");
        closeHVPlus();
        delay(100);
        openPrecharge();

        DEBUG_PRINTLN("ContactorManager: Drive path ARMED");
        transitionTo(ContactorState::DISCHARGE_ARMED);
    }
}

void ContactorManager::handleDischargeArmed() {
    if (!inputManager->isDischargeAllowed()) {
        DEBUG_PRINTLN("ContactorManager: Discharge allowance violated! Disabling discharge...");
        DEBUG_PRINTLN("ContactorManager: Waiting for discharge current to reach zero...");

        if (!waitForCurrentZero(Timing::CURRENT_ZERO_TIMEOUT)) {
            DEBUG_PRINTLN("ContactorManager: ERROR - Current did not reach zero in time! Opening contactors");
            openAllHardware();
            transitionTo(ContactorState::OPEN);
            setError(ContactorError::CURRENT_NOT_ZERO);
        }
    }
}

void ContactorManager::handleError() {
    static unsigned long lastPrint = 0;
    unsigned long now = millis();
    if (now - lastPrint >= 1000) {
        DEBUG_PRINTLN("ContactorManager: ERROR state - manual reset required");
        lastPrint = now;
    }
}

//=============================================================================
// CONTACTOR SEQUENCES
//=============================================================================

bool ContactorManager::startChargeSequence() {
    DEBUG_PRINTLN("ContactorManager: Starting CHARGE sequence...");

    if (currentState != ContactorState::OPEN) {
        DEBUG_PRINTLN("  ERROR: Contactors not in OPEN state!");
        return false;
    }

    if (!inputManager->isChargeAllowed()) {
        DEBUG_PRINTLN("  ERROR: Charge not allowed!");
        setError(ContactorError::CHARGE_ALLOW_VIOLATED);
        return false;
    }

    if (!isBMSArmed()) {
        DEBUG_PRINTLN("  ERROR: BMS not armed!");
        setError(ContactorError::BMS_NOT_ARMED);
        return false;
    }

    // Charge sequence: HV- → Charge Cont → Precharge → [wait] → HV+ → open Precharge
    DEBUG_PRINTLN("  Step 1: Closing HV-...");
    closeHVMinus();
    delay(150);

    DEBUG_PRINTLN("  Step 2: Closing Charge Contactor...");
    closeChargeContactor();
    delay(150);

    DEBUG_PRINTLN("  Step 3: Closing Precharge...");
    closePrecharge();

    transitionTo(ContactorState::CHARGE_PRECHARGING);
    return true;
}

bool ContactorManager::startDischargeSequence() {
    DEBUG_PRINTLN("ContactorManager: Starting DRIVE sequence...");
    inputManager->update();

    if (currentState != ContactorState::OPEN) {
        DEBUG_PRINTLN("  ERROR: Contactors not in OPEN state!");
        return false;
    }

    if (!inputManager->isDischargeAllowed()) {
        DEBUG_PRINTLN("  ERROR: Discharge not allowed!");
        setError(ContactorError::DISCHARGE_ALLOW_VIOLATED);
        return false;
    }

    if (!isBMSArmed()) {
        DEBUG_PRINTLN("  ERROR: BMS not armed!");
        setError(ContactorError::BMS_NOT_ARMED);
        return false;
    }

    // Drive sequence: HV- → Precharge → [wait] → HV+ → open Precharge
    DEBUG_PRINTLN("  Step 1: Closing HV-...");
    closeHVMinus();
    delay(150);

    DEBUG_PRINTLN("  Step 2: Closing Precharge...");
    closePrecharge();

    transitionTo(ContactorState::DISCHARGE_PRECHARGING);
    return true;
}

void ContactorManager::openAllContactors(bool immediate) {
    DEBUG_PRINTLN("ContactorManager: Opening all contactors...");

    if (!immediate) {
        DEBUG_PRINTLN("  Waiting for current to reach zero...");
        if (!waitForCurrentZero(Timing::CURRENT_ZERO_TIMEOUT)) {
            DEBUG_PRINTLN("  WARNING: Current did not reach zero, opening anyway!");
        }
    }

    openAllHardware();
    transitionTo(ContactorState::OPEN);
    DEBUG_PRINTLN("ContactorManager: All contactors OPEN");
}

void ContactorManager::emergencyShutdown() {
    DEBUG_PRINTLN("ContactorManager: EMERGENCY SHUTDOWN!");
    openAllHardware();
    setError(ContactorError::CURRENT_NOT_ZERO);
}

//=============================================================================
// HARDWARE CONTROL
//=============================================================================

void ContactorManager::closeHVMinus() {
    digitalWrite(Pins::HV_MINUS_CONTACTOR, HIGH);
    DEBUG_PRINTLN("  HV- contactor: CLOSED");
}

void ContactorManager::openHVMinus() {
    digitalWrite(Pins::HV_MINUS_CONTACTOR, LOW);
    DEBUG_PRINTLN("  HV- contactor: OPEN");
}

void ContactorManager::closeHVPlus() {
    digitalWrite(Pins::HV_PLUS_CONTACTOR, HIGH);
    DEBUG_PRINTLN("  HV+ contactor: CLOSED");
}

void ContactorManager::openHVPlus() {
    digitalWrite(Pins::HV_PLUS_CONTACTOR, LOW);
    DEBUG_PRINTLN("  HV+ contactor: OPEN");
}

void ContactorManager::closePrecharge() {
    digitalWrite(Pins::PRECHARGE_CONTACTOR, HIGH);
    DEBUG_PRINTLN("  Precharge relay: CLOSED");
}

void ContactorManager::openPrecharge() {
    digitalWrite(Pins::PRECHARGE_CONTACTOR, LOW);
    DEBUG_PRINTLN("  Precharge relay: OPEN");
}

void ContactorManager::closeChargeContactor() {
    digitalWrite(Pins::CHARGE_CONTACTOR, HIGH);
    DEBUG_PRINTLN("  Charge contactor: CLOSED");
}

void ContactorManager::openChargeContactor() {
    digitalWrite(Pins::CHARGE_CONTACTOR, LOW);
    DEBUG_PRINTLN("  Charge contactor: OPEN");
}

void ContactorManager::openAllHardware() {
    digitalWrite(Pins::HV_MINUS_CONTACTOR,  LOW);
    digitalWrite(Pins::HV_PLUS_CONTACTOR,   LOW);
    digitalWrite(Pins::PRECHARGE_CONTACTOR, LOW);
    digitalWrite(Pins::CHARGE_CONTACTOR,    LOW);
    DEBUG_PRINTLN("  All contactor hardware: OPEN");
}

//=============================================================================
// SAFETY CHECKS
//=============================================================================

bool ContactorManager::verifyCurrentZero() {
    if (bmsManager == nullptr) {
        return false;
    }

    BMSDataExtended bmsData = bmsManager->getData();

    float currentAmps = bmsData.packCurrent * 0.1f;
    bool isZero = (fabs(currentAmps) < Battery::CURRENT_ZERO_THRESHOLD);

    DEBUG_PRINTF("  BMS Current: %.2fA %s\n", currentAmps, isZero ? "(ZERO)" : "(FLOWING)");

    return isZero;
}

bool ContactorManager::verifyPrechargeVoltage(bool isCharging) {
    if (bmsManager == nullptr) return false;

    float packVoltage = bmsManager->getData().packVoltage * 0.1f;

    float busVoltage = 0.0f;
    if (isCharging) {
        if (nlg5Manager == nullptr) return false;
        busVoltage = nlg5Manager->getData().batteryVoltageActual;
    } else {
        busVoltage = sharedDMCData.get().dcVoltage;
    }

    bool ok = (busVoltage >= (packVoltage - Battery::PRECHARGE_TOLERANCE));
    DEBUG_PRINTF("  Precharge verify: bus=%.1fV pack=%.1fV threshold=%.1fV %s\n",
                 busVoltage, packVoltage, packVoltage - Battery::PRECHARGE_TOLERANCE,
                 ok ? "OK" : "WAIT");
    return ok;
}

bool ContactorManager::waitForCurrentZero(uint32_t timeoutMs) {
    unsigned long startTime = millis();

    while ((millis() - startTime) < timeoutMs) {
        if (verifyCurrentZero()) {
            return true;
        }
        delay(Timing::CURRENT_ZERO_CHECK_INTERVAL);
    }

    return false;
}

bool ContactorManager::isBMSArmed() {
    if (bmsManager == nullptr) {
        return false;
    }

    BMSDataExtended bmsData = bmsManager->getData();
    bool armed = bmsData.channelEnabled;

    DEBUG_PRINTF("  BMS Armed: %s\n", armed ? "YES" : "NO");

    return armed;
}

bool ContactorManager::isChargingSafe() {
    if (!inputManager->isChargeAllowed()) {
        return false;
    }
    return true;
}

bool ContactorManager::isDischargingSafe() {
    if (!inputManager->isDischargeAllowed()) {
        return false;
    }
    return true;
}

//=============================================================================
// HELPERS
//=============================================================================

void ContactorManager::transitionTo(ContactorState newState) {
    if (newState == currentState) {
        return;
    }

    DEBUG_PRINTF("ContactorManager: State %d -> %d\n", (int)currentState, (int)newState);

    previousState = currentState;
    currentState = newState;
    stateEntryTime = millis();

    if (previousState == ContactorState::ERROR && newState != ContactorState::ERROR) {
        lastError = ContactorError::NONE;
    }
}

void ContactorManager::setError(ContactorError error) {
    DEBUG_PRINTF("ContactorManager: ERROR - Code %d\n", (int)error);

    lastError = error;
    openAllHardware();
    transitionTo(ContactorState::ERROR);
}
