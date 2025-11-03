#pragma once
#include <Arduino.h>
#include "config.h"

// Forward declarations
class BMSManager;
class InputManager;

//=============================================================================
// CONTACTOR MANAGER
// Handles: Separate charge/discharge contactor sequences with precharge
// Safety: Monitors charge/discharge allowance and current verification
//=============================================================================

enum class ContactorState : uint8_t {
    OPEN,                    // All contactors open
    CHARGE_PRECHARGING,      // Charge precharge relay closed, waiting
    CHARGE_ARMED,            // Charge contactors closed, ready to charge
    DISCHARGE_PRECHARGING,   // Discharge precharge relay closed, waiting
    DISCHARGE_ARMED,         // Discharge contactors closed, ready to drive
    ERROR                    // Safety violation detected
};

enum class ContactorError : uint8_t {
    NONE,
    PRECHARGE_TIMEOUT,
    CHARGE_ALLOW_VIOLATED,
    DISCHARGE_ALLOW_VIOLATED,
    CURRENT_NOT_ZERO,
    BMS_NOT_ARMED
};

class ContactorManager {
public:
    ContactorManager();

    /**
     * @brief Initialize contactor manager
     * @param bmsMgr Pointer to BMSManager for BMS arming and current checks
     * @param inputMgr Pointer to InputManager for allowance monitoring
     */
    void begin(BMSManager* bmsMgr, InputManager* inputMgr);

    /**
     * @brief Update contactor state machine (call periodically)
     */
    void update();

    // -------------------------------------------------------------------------
    // CONTACTOR CONTROL
    // -------------------------------------------------------------------------

    /**
     * @brief Start charge path precharge sequence
     * Steps: 1) Arm BMS, 2) Close charge precharge, 3) Wait, 4) Close main charge, 5) Open precharge
     * @return true if sequence started successfully
     */
    bool startChargeSequence();

    /**
     * @brief Start discharge path precharge sequence
     * Steps: 1) Arm BMS, 2) Close discharge precharge, 3) Wait, 4) Close main discharge, 5) Open precharge
     * @return true if sequence started successfully
     */
    bool startDischargeSequence();

    /**
     * @brief Open all contactors and disarm BMS
     * @param immediate If true, opens contactors without waiting for current verification
     */
    void openAllContactors(bool immediate = false);

    /**
     * @brief Emergency shutdown - immediately open all contactors
     */
    void emergencyShutdown();

    // -------------------------------------------------------------------------
    // STATE QUERIES
    // -------------------------------------------------------------------------
    ContactorState getState() const { return currentState; }
    ContactorError getError() const { return lastError; }
    bool isChargeArmed() const { return currentState == ContactorState::CHARGE_ARMED; }
    bool isDischargeArmed() const { return currentState == ContactorState::DISCHARGE_ARMED; }
    bool hasError() const { return currentState == ContactorState::ERROR; }

    // -------------------------------------------------------------------------
    // SAFETY MONITORING
    // -------------------------------------------------------------------------

    /**
     * @brief Check if charging is safe based on allowance and current
     * @return true if safe to charge
     */
    bool isChargingSafe();

    /**
     * @brief Check if discharging is safe based on allowance and current
     * @return true if safe to discharge
     */
    bool isDischargingSafe();

private:
    // Manager references
    BMSManager* bmsManager;
    InputManager* inputManager;

    // State
    ContactorState currentState;
    ContactorState previousState;
    ContactorError lastError;

    // Timing
    unsigned long stateEntryTime;
    unsigned long currentCheckStartTime;

    // Helper functions
    void transitionTo(ContactorState newState);
    void setError(ContactorError error);

    // State handlers
    void handleOpen();
    void handleChargePrecharging();
    void handleChargeArmed();
    void handleDischargePrecharging();
    void handleDischargeArmed();
    void handleError();

    // Hardware control
    void closeChargePrecharge();
    void openChargePrecharge();
    void closeMainChargeContactor();
    void openMainChargeContactor();
    void closeDischargePrecharge();
    void openDischargePrecharge();
    void closeMainDischargeContactor();
    void openMainDischargeContactor();
    void openAllHardware();

    // Safety checks
    bool verifyCurrentZero();
    bool isBMSArmed();
    bool waitForCurrentZero(uint32_t timeoutMs);
};
