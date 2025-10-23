#pragma once
#include <Arduino.h>
#include "config.h"
#include "bms_manager.h"
#include "nlg5_manager.h"

//=============================================================================
// CHARGING COORDINATOR
// Manages the charging process with safety interlocks between BMS and NLG5
//=============================================================================

enum class ChargingState : uint8_t {
    IDLE,               // Not charging
    PRECHECK,           // Pre-charge safety checks
    STARTING,           // Starting charger
    BULK_CHARGE,        // CC phase - constant current
    ABSORPTION,         // CV phase - constant voltage
    BALANCING,          // Optional balancing phase
    COMPLETE,           // Charging complete
    ERROR,              // Error state
    DISABLED_STATE      // Manually disabled (renamed to avoid ESP32 HAL macro conflict)
};

struct ChargingStatus {
    ChargingState state;
    float targetVoltage;            // Target pack voltage (V)
    float targetCurrent;            // Target charge current (A)
    float actualVoltage;            // Actual pack voltage (V)
    float actualCurrent;            // Actual charge current (A)
    float chargePower;              // Charging power (W)
    uint8_t socPercent;             // State of charge (%)
    float timeRemaining;            // Estimated time remaining (minutes)
    float energyCharged;            // Energy charged this session (Wh)
    bool safetyOK;                  // All safety checks pass
    String errorMessage;            // Error description
    unsigned long sessionStartTime; // Session start timestamp
};

class ChargingCoordinator {
public:
    ChargingCoordinator(BMSManager& bms, NLG5Manager& nlg5);

    /**
     * @brief Initialize charging coordinator
     */
    void begin();

    /**
     * @brief Update task - manage charging state machine
     * Call this periodically (e.g., every 100ms)
     */
    void update();

    /**
     * @brief Start charging session
     * @return true if charging started successfully
     */
    bool startCharging();

    /**
     * @brief Stop charging session
     */
    void stopCharging();

    /**
     * @brief Get charging status
     */
    ChargingStatus getStatus() const { return status; }

    /**
     * @brief Check if charging is active
     */
    bool isCharging() const;

    /**
     * @brief Check if safe to charge
     */
    bool isSafeToCharge() const;

private:
    BMSManager& bms;
    NLG5Manager& nlg5;
    ChargingStatus status;
    SemaphoreHandle_t statusMutex;

    // Safety monitoring
    unsigned long lastSafetyCheckTime;
    unsigned long chargeStartTime;
    float energyAccumulator;  // For integrating power over time

    // Charging parameters (calculated from BMS)
    float maxChargeVoltage;
    float maxChargeCurrent;
    float maxChargeTemp;

    // State machine
    void updateStateMachine();
    void transitionToState(ChargingState newState);

    // Safety checks
    bool performSafetyChecks();
    bool checkBMSSafety();
    bool checkChargerSafety();
    bool checkTemperatureSafety();
    bool checkVoltageSafety();
    bool checkTimeoutSafety();

    // Charging logic
    void calculateChargingParameters();
    void updateChargingSetpoints();
    void updateStatistics();

    // Helper functions
    float estimateTimeRemaining() const;
};
