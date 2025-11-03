#pragma once
#include <Arduino.h>
#include "config.h"

// Forward declarations
class BMSManager;
class InputManager;
class ContactorManager;

//=============================================================================
// VEHICLE CONTROL
// Handles: Torque calculation, direction control, safety logic
//=============================================================================

class VehicleControl {
public:
    VehicleControl();
    
    /**
     * @brief Initialize vehicle control
     * @param bmsMgr Pointer to BMSManager for display control (optional)
     * @param inputMgr Pointer to InputManager for safety monitoring (optional)
     * @param contactorMgr Pointer to ContactorManager for emergency shutdown (optional)
     */
    void begin(BMSManager* bmsMgr = nullptr, InputManager* inputMgr = nullptr, ContactorManager* contactorMgr = nullptr);
    
    /**
     * @brief Update control logic (call in loop)
     */
    void update();
    
    // -------------------------------------------------------------------------
    // TORQUE OUTPUT
    // -------------------------------------------------------------------------
    /**
     * @brief Get calculated torque demand in Nm
     * @return Torque in Nm (-420 to +850)
     */
    int16_t getTorqueDemand() const { return torqueDemand; }
    
    // -------------------------------------------------------------------------
    // DIRECTION CONTROL
    // -------------------------------------------------------------------------
    void handleDirectionToggle();  // Call when direction button pressed (short press toggles D/R)
    void setNeutral();             // Set gear to Neutral (long hold)
    GearState getCurrentGear() const { return currentGear; }
    
    // -------------------------------------------------------------------------
    // INPUT SETTERS (Called by InputManager)
    // -------------------------------------------------------------------------
    void setThrottle(float percent) { throttleInput = percent; }
    void setRegen(float percent) { regenInput = percent; }
    void setBrakePressed(bool pressed) { brakePressed = pressed; }
    void setMotorSpeed(float rpm) { motorSpeedRPM = rpm; }
    void setSystemReady(bool ready) { systemReady = ready; }
    
    // -------------------------------------------------------------------------
    // SAFETY
    // -------------------------------------------------------------------------
    void emergencyStop();  // Force zero torque + neutral
    
private:
    // Current state
    GearState currentGear;
    bool systemReady;

    // Inputs
    float throttleInput;    // 0-100%
    float regenInput;       // 0-100%
    bool brakePressed;
    float motorSpeedRPM;

    // Output
    int16_t torqueDemand;   // Nm

    // Manager references
    BMSManager* bmsManager;
    InputManager* inputManager;
    ContactorManager* contactorManager;

    // Timing
    unsigned long lastUpdateTime;

    // Helper to update BMS display based on gear
    void updateBMSDisplay();
    
    // Torque calculation
    int16_t calculateTorque();
    int16_t applyTorqueCurve(int16_t baseTorque);
    int16_t applyRegenCurve(int16_t baseRegen);
    
    // Safety checks
    bool isSafeToApplyTorque();
    void applyDeadzone(int16_t& torque);
    void checkDischargeAllowance();  // Monitor discharge allowance during driving
    void checkChargeAllowance();     // Monitor charge allowance for regen

    // Safety state
    bool regenEnabled;               // Regen can be disabled if charge not allowed
};