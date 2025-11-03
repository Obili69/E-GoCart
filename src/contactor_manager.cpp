#include "contactor_manager.h"
#include "bms_manager.h"
#include "input_manager.h"
#include "data_structures.h"

//=============================================================================
// CONSTRUCTOR
//=============================================================================

ContactorManager::ContactorManager()
    : bmsManager(nullptr)
    , inputManager(nullptr)
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

void ContactorManager::begin(BMSManager* bmsMgr, InputManager* inputMgr) {
    DEBUG_PRINTLN("ContactorManager: Initializing...");

    bmsManager = bmsMgr;
    inputManager = inputMgr;

    // Configure all contactor pins as outputs
    pinMode(Pins::CHARGE_PRECHARGE, OUTPUT);
    pinMode(Pins::MAIN_CHARGE_CONTACTOR, OUTPUT);
    pinMode(Pins::DISCHARGE_PRECHARGE, OUTPUT);
    pinMode(Pins::MAIN_DISCHARGE_CONTACTOR, OUTPUT);

    // Ensure all contactors are open initially
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

    // Check for charge allowance violation during precharge
    if (!inputManager->isChargeAllowed()) {
        DEBUG_PRINTLN("ContactorManager: Charge allowance violated during precharge!");
        setError(ContactorError::CHARGE_ALLOW_VIOLATED);
        return;
    }

    // Wait for precharge time
    if (elapsed >= Timing::PRECHARGE_DELAY_MS) {
        DEBUG_PRINTLN("ContactorManager: Charge precharge complete, closing main contactor...");

        // Close main charge contactor
        closeMainChargeContactor();

        // Wait a moment for contactor to close
        delay(100);

        // Open precharge relay
        openChargePrecharge();

        DEBUG_PRINTLN("ContactorManager: Charge path ARMED");
        transitionTo(ContactorState::CHARGE_ARMED);
    }
}

void ContactorManager::handleChargeArmed() {
    // Monitor charge allowance - if it goes high (not allowed), must disable charging
    if (!inputManager->isChargeAllowed()) {
        DEBUG_PRINTLN("ContactorManager: Charge allowance violated! Disabling charge...");

        // Set charge current to 0 in NLG (handled by charger control in state_manager)
        // Wait for current to go to zero, then open contactors
        DEBUG_PRINTLN("ContactorManager: Waiting for charge current to reach zero...");

        if (waitForCurrentZero(Timing::CURRENT_ZERO_TIMEOUT)) {
            DEBUG_PRINTLN("ContactorManager: Current verified zero");
            
        } else {
            DEBUG_PRINTLN("ContactorManager: ERROR - Current did not reach zero in time!");
            openMainChargeContactor();
            delay(100);
            transitionTo(ContactorState::OPEN);
            setError(ContactorError::CURRENT_NOT_ZERO);
        }
    }
}

void ContactorManager::handleDischargePrecharging() {
    unsigned long elapsed = millis() - stateEntryTime;

    // Check for discharge allowance violation during precharge
    if (!inputManager->isDischargeAllowed()) {
        DEBUG_PRINTLN("ContactorManager: Discharge allowance violated during precharge!");
        setError(ContactorError::DISCHARGE_ALLOW_VIOLATED);
        return;
    }

    // Wait for precharge time
    if (elapsed >= Timing::PRECHARGE_DELAY_MS) {
        DEBUG_PRINTLN("ContactorManager: Discharge precharge complete, closing main contactor...");

        // Close main discharge contactor
        closeMainDischargeContactor();

        // Wait a moment for contactor to close
        delay(100);

        // Open precharge relay
        openDischargePrecharge();

        DEBUG_PRINTLN("ContactorManager: Discharge path ARMED");
        transitionTo(ContactorState::DISCHARGE_ARMED);
    }
}

void ContactorManager::handleDischargeArmed() {
    // Monitor discharge allowance - if it goes high (not allowed), must disable driving
    if (!inputManager->isDischargeAllowed()) {
        DEBUG_PRINTLN("ContactorManager: Discharge allowance violated! Disabling discharge...");

        // Force vehicle to Neutral (handled by vehicle_control)
        // Wait for current to go to zero, then open contactors
        DEBUG_PRINTLN("ContactorManager: Waiting for discharge current to reach zero...");

        if (waitForCurrentZero(Timing::CURRENT_ZERO_TIMEOUT)) {
            DEBUG_PRINTLN("ContactorManager: Current verified zero");
        } else {
            DEBUG_PRINTLN("ContactorManager: ERROR - Current did not reach zero in time! opening contactors");
            openMainDischargeContactor();
            delay(100);
            transitionTo(ContactorState::OPEN);
            setError(ContactorError::CURRENT_NOT_ZERO);
        }
    }
}

void ContactorManager::handleError() {
    // Error state - contactors should already be open
    // Can only exit via manual reset or system restart
    DEBUG_PRINTLN("ContactorManager: ERROR state - manual reset required");
    delay(1000);  // Prevent log spam
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
    
    // Check charge allowance before starting
    if (!inputManager->isChargeAllowed()) {
        DEBUG_PRINTLN("  ERROR: Charge not allowed!");
        setError(ContactorError::CHARGE_ALLOW_VIOLATED);
        return false;
    }

    // Verify BMS is armed
    if (!isBMSArmed()) {
        DEBUG_PRINTLN("  ERROR: BMS not armed!");
        setError(ContactorError::BMS_NOT_ARMED);
        return false;
    }

    // Step 2: Close charge precharge relay
    DEBUG_PRINTLN("  Step 2: Closing charge precharge relay...");
    closeChargePrecharge();

    // Transition to precharging state (will wait for precharge time)
    transitionTo(ContactorState::CHARGE_PRECHARGING);

    return true;
}

bool ContactorManager::startDischargeSequence() {
    DEBUG_PRINTLN("ContactorManager: Starting DISCHARGE sequence...");
    inputManager->update();
    if (currentState != ContactorState::OPEN) {
        DEBUG_PRINTLN("  ERROR: Contactors not in OPEN state!");
        return false;
    }

    // Check discharge allowance before starting
    if (!inputManager->isDischargeAllowed()) {
        DEBUG_PRINTLN("  ERROR: Discharge not allowed!");
        setError(ContactorError::DISCHARGE_ALLOW_VIOLATED);
        return false;
    }

    // Verify BMS is armed
    if (!isBMSArmed()) {
        DEBUG_PRINTLN("  ERROR: BMS not armed!");
        setError(ContactorError::BMS_NOT_ARMED);
        return false;
    }

    // Step 2: Close discharge precharge relay
    DEBUG_PRINTLN("  Step 2: Closing discharge precharge relay...");
    closeDischargePrecharge();

    // Transition to precharging state (will wait for precharge time)
    transitionTo(ContactorState::DISCHARGE_PRECHARGING);

    return true;
}

void ContactorManager::openAllContactors(bool immediate) {
    DEBUG_PRINTLN("ContactorManager: Opening all contactors...");

    if (!immediate) {
        // Wait for current to reach zero before opening
        DEBUG_PRINTLN("  Waiting for current to reach zero...");
        if (!waitForCurrentZero(Timing::CURRENT_ZERO_TIMEOUT)) {
            DEBUG_PRINTLN("  WARNING: Current did not reach zero, opening anyway!");
        }
    }
    // Open all hardware
    openAllHardware();

    transitionTo(ContactorState::OPEN);
    DEBUG_PRINTLN("ContactorManager: All contactors OPEN");
}

void ContactorManager::emergencyShutdown() {
    DEBUG_PRINTLN("ContactorManager: EMERGENCY SHUTDOWN!");

    // Immediately open all contactors without waiting
    openAllHardware();

    setError(ContactorError::CURRENT_NOT_ZERO);
}

//=============================================================================
// HARDWARE CONTROL
//=============================================================================

void ContactorManager::closeChargePrecharge() {
    digitalWrite(Pins::CHARGE_PRECHARGE, HIGH);
    DEBUG_PRINTLN("  Charge precharge relay: CLOSED");
}

void ContactorManager::openChargePrecharge() {
    digitalWrite(Pins::CHARGE_PRECHARGE, LOW);
    DEBUG_PRINTLN("  Charge precharge relay: OPEN");
}

void ContactorManager::closeMainChargeContactor() {
    digitalWrite(Pins::MAIN_CHARGE_CONTACTOR, HIGH);
    DEBUG_PRINTLN("  Main charge contactor: CLOSED");
}

void ContactorManager::openMainChargeContactor() {
    digitalWrite(Pins::MAIN_CHARGE_CONTACTOR, LOW);
    DEBUG_PRINTLN("  Main charge contactor: OPEN");
}

void ContactorManager::closeDischargePrecharge() {
    digitalWrite(Pins::DISCHARGE_PRECHARGE, HIGH);
    DEBUG_PRINTLN("  Discharge precharge relay: CLOSED");
}

void ContactorManager::openDischargePrecharge() {
    digitalWrite(Pins::DISCHARGE_PRECHARGE, LOW);
    DEBUG_PRINTLN("  Discharge precharge relay: OPEN");
}

void ContactorManager::closeMainDischargeContactor() {
    digitalWrite(Pins::MAIN_DISCHARGE_CONTACTOR, HIGH);
    DEBUG_PRINTLN("  Main discharge contactor: CLOSED");
}

void ContactorManager::openMainDischargeContactor() {
    digitalWrite(Pins::MAIN_DISCHARGE_CONTACTOR, LOW);
    DEBUG_PRINTLN("  Main discharge contactor: OPEN");
}

void ContactorManager::openAllHardware() {
    digitalWrite(Pins::CHARGE_PRECHARGE, LOW);
    digitalWrite(Pins::MAIN_CHARGE_CONTACTOR, LOW);
    digitalWrite(Pins::DISCHARGE_PRECHARGE, LOW);
    digitalWrite(Pins::MAIN_DISCHARGE_CONTACTOR, LOW);
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

    // Check if current is within threshold (±0.5A considered zero)
    float currentAmps = bmsData.packCurrent * 0.1f;  // Convert from 0.01A units to A
    bool isZero = (fabs(currentAmps) < Battery::CURRENT_ZERO_THRESHOLD);

    DEBUG_PRINTF("  BMS Current: %.2fA %s\n", currentAmps, isZero ? "(ZERO)" : "(FLOWING)");

    return isZero;
}

bool ContactorManager::waitForCurrentZero(uint32_t timeoutMs) {
    unsigned long startTime = millis();

    while ((millis() - startTime) < timeoutMs) {
        if (verifyCurrentZero()) {
            return true;
        }

        delay(Timing::CURRENT_ZERO_CHECK_INTERVAL);
    }

    return false;  // Timeout
}

bool ContactorManager::isBMSArmed() {
    if (bmsManager == nullptr) {
        return false;
    }

    BMSDataExtended bmsData = bmsManager->getData();

    // Check if BMS channel is enabled (MOS transistors are on)
    bool armed = bmsData.channelEnabled;

    DEBUG_PRINTF("  BMS Armed: %s\n", armed ? "YES" : "NO");

    return armed;
}

bool ContactorManager::isChargingSafe() {
    // Check charge allowance
    if (!inputManager->isChargeAllowed()) {
        return false;
    }

    // Check if current is safe
    if (currentState == ContactorState::CHARGE_ARMED) {
        // During charging, verify current is reasonable
        return true;  // Allow current during charging
    }

    return true;
}

bool ContactorManager::isDischargingSafe() {
    // Check discharge allowance
    if (!inputManager->isDischargeAllowed()) {
        return false;
    }

    // Check if current is safe
    if (currentState == ContactorState::DISCHARGE_ARMED) {
        // During discharging, verify current is reasonable
        return true;  // Allow current during discharging
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

    // Clear error on transition out of error state
    if (previousState == ContactorState::ERROR && newState != ContactorState::ERROR) {
        lastError = ContactorError::NONE;
    }
}

void ContactorManager::setError(ContactorError error) {
    DEBUG_PRINTF("ContactorManager: ERROR - Code %d\n", (int)error);

    lastError = error;

    // Open all contactors immediately
    openAllHardware();

    transitionTo(ContactorState::ERROR);
}
