#pragma once
#include <Arduino.h>
#include "config.h"

// Forward declarations
class BMSManager;
class InputManager;
class NLG5Manager;

//=============================================================================
// CONTACTOR MANAGER
// Handles: Unified HV bus with shared HV+/HV-/precharge contactors
// Drive:  HV- → Precharge → wait → HV+ → open Precharge
// Charge: HV- → Charge Cont → Precharge → wait → HV+ → open Precharge
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
     * @param bmsMgr   BMS for arming, current, and pack voltage
     * @param inputMgr InputManager for allowance monitoring
     * @param nlgMgr   NLG5Manager for charger-side bus voltage (charge precharge verify)
     */
    void begin(BMSManager* bmsMgr, InputManager* inputMgr, NLG5Manager* nlgMgr);

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
    NLG5Manager* nlg5Manager;

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

    // Hardware control (shared bus)
    void closeHVMinus();
    void openHVMinus();
    void closeHVPlus();
    void openHVPlus();
    void closePrecharge();
    void openPrecharge();
    void closeChargeContactor();
    void openChargeContactor();
    void openAllHardware();

    // Safety checks
    bool verifyCurrentZero();
    bool isBMSArmed();
    bool waitForCurrentZero(uint32_t timeoutMs);
    bool verifyPrechargeVoltage(bool isCharging);
};
