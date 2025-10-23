#pragma once
#include <Arduino.h>
#include <esp_sleep.h>
#include "config.h"

//=============================================================================
// STATE MANAGER
// Handles: Vehicle state machine, sleep/wake, system initialization
//=============================================================================

class StateManager {
public:
    StateManager();
    
    /**
     * @brief Initialize state manager
     */
    void begin();
    
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
    void handleStartButton();     // Wake from sleep / Enter READY
    void handleStopButton();      // Enter sleep after timeout
    void handleILOpen();          // Interlock opened → Emergency shutdown
    void handleChargerConnected(); // Enter CHARGING mode
    void handleChargerDisconnected();
    
    // -------------------------------------------------------------------------
    // SYSTEM STATUS
    // -------------------------------------------------------------------------
    bool isBatteryArmed() const { return batteryArmed; }
    bool isPrechargeComplete() const { return prechargeComplete; }
    bool isSystemReady() const { return systemReady; }
    
private:
    // Current state
    VehicleState currentState;
    VehicleState previousState;
    
    // System flags
    bool batteryArmed;
    bool prechargeComplete;
    bool systemReady;
    bool emergencyShutdown;
    
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
    
    // Battery management
    void startPrecharge();
    void checkPrecharge();
    void armBattery();
    void disarmBattery();
};