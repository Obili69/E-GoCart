#pragma once
#include <Arduino.h>
#include <esp_sleep.h>
#include "config.h"

// Forward declarations
class BMSManager;
class ContactorManager;

//=============================================================================
// STATE MANAGER
// Handles: Vehicle state machine, sleep/wake, system initialization
//=============================================================================

class StateManager {
public:
    StateManager();
    
    /**
     * @brief Initialize state manager
     * @param bmsMgr Pointer to BMSManager for power control
     * @param contactorMgr Pointer to ContactorManager for contactor control
     */
    void begin(BMSManager* bmsMgr = nullptr, ContactorManager* contactorMgr = nullptr);
    
    /**
     * @brief Update state machine (call in loop)
     */
    void update();
    
    // -------------------------------------------------------------------------
    // STATE QUERIES
    // -------------------------------------------------------------------------
    VehicleState getCurrentState() const { return currentState; }
    bool isReady() const { return currentState == VehicleState::READY || 
                                  currentState == VehicleState::DRIVE; }
    bool isDriving() const { return currentState == VehicleState::DRIVE; }
    bool isCharging() const { return currentState == VehicleState::CHARGING; }
    
    // -------------------------------------------------------------------------
    // STATE TRANSITIONS (Called by external events)
    // -------------------------------------------------------------------------
    void handleWakeup(bool fromCharger);          // Handle system wake-up (button or charger)
    void handleChargerPinChange(bool connected);   // Charger pin HIGH/LOW
    void handleStopRequest();                      // Stop request from DRIVE (neutral + button)
    void handleEmergencyShutdown();                // Emergency shutdown (interlock open)
    
    // -------------------------------------------------------------------------
    // SYSTEM STATUS
    // -------------------------------------------------------------------------
    bool isBatteryArmed() const { return batteryArmed; }
    bool isSystemReady() const { return systemReady; }

    // -------------------------------------------------------------------------
    // COOLING CONTROL
    // -------------------------------------------------------------------------
    /**
     * @brief Update cooling pump based on motor/inverter temperatures
     * @param motorTemp Motor temperature in °C
     * @param inverterTemp Inverter temperature in °C
     */
    void updateCoolingPump(float motorTemp, float inverterTemp);

private:
    // Current state
    VehicleState currentState;
    VehicleState previousState;

    // System flags
    bool batteryArmed;
    bool systemReady;
    bool emergencyShutdown;
    uint8_t requestedMode;

    // Cooling control
    bool pumpRunning;                  // Current pump state

    // Manager references
    BMSManager* bmsManager;
    ContactorManager* contactorManager;

    // Timing
    unsigned long stateEntryTime;      // When we entered current state
    unsigned long stopButtonPressTime; // For sleep timeout
    
    // State handlers
    void handleSleepState();
    void handleInitState();
    void handleReadyState();
    void handleDriveState();
    void handleChargingState();
    
    // State transitions
    void transitionTo(VehicleState newState);
    
    // System control
    void initializeSystems();
    void shutdownSystems();
    void enterDeepSleep();
    
    // Battery management (NEW - uses contactor manager)
    void armBatteryForCharging();
    void armBatteryForDischarging();
    void disarmBattery();

    // Helpers
    bool shouldDisableChargerWakeup();
};