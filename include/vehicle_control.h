#pragma once
#include <Arduino.h>
#include "config.h"

//=============================================================================
// VEHICLE CONTROL
// Handles: Torque calculation, direction control, safety logic
//=============================================================================

class VehicleControl {
public:
    VehicleControl();
    
    /**
     * @brief Initialize vehicle control
     */
    void begin();
    
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
    void handleDirectionToggle();  // Call when direction button pressed
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
    
    // Timing
    unsigned long lastUpdateTime;
    
    // Torque calculation
    int16_t calculateTorque();
    int16_t applyTorqueCurve(int16_t baseTorque);
    int16_t applyRegenCurve(int16_t baseRegen);
    
    // Safety checks
    bool isSafeToApplyTorque();
    void applyDeadzone(int16_t& torque);
};