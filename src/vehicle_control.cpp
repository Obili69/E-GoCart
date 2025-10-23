#include "vehicle_control.h"

VehicleControl::VehicleControl()
    : currentGear(GearState::NEUTRAL)
    , systemReady(false)
    , throttleInput(0.0f)
    , regenInput(0.0f)
    , brakePressed(false)
    , motorSpeedRPM(0.0f)
    , torqueDemand(0)
    , lastUpdateTime(0)
{
}

void VehicleControl::begin() {
    DEBUG_PRINTLN("VehicleControl: Initialized");
}

void VehicleControl::update() {
    torqueDemand = calculateTorque();
}

//=============================================================================
// DIRECTION CONTROL
//=============================================================================

void VehicleControl::handleDirectionToggle() {
    if (!systemReady) {
        DEBUG_PRINTLN("Direction: System not ready");
        return;
    }
    
    // Cycle through gears: N -> D -> R -> D -> ...
    switch (currentGear) {
        case GearState::NEUTRAL:
            currentGear = GearState::DRIVE;
            DEBUG_PRINTLN("Gear: DRIVE");
            break;
            
        case GearState::DRIVE:
            currentGear = GearState::REVERSE;
            DEBUG_PRINTLN("Gear: REVERSE");
            break;
            
        case GearState::REVERSE:
            currentGear = GearState::DRIVE;
            DEBUG_PRINTLN("Gear: DRIVE");
            break;
    }
}

//=============================================================================
// TORQUE CALCULATION
//=============================================================================

int16_t VehicleControl::calculateTorque() {
    // Safety checks
    if (!isSafeToApplyTorque()) {
        return 0;
    }
    
    int16_t torque = 0;
    
    // Regen has priority over throttle (only when throttle is zero)
    if (throttleInput < 1.0f && regenInput > 1.0f) {
        // Regen mode: map 0-100% to 0 to -MAX_REGEN_NM
        float regenNm = (regenInput / 100.0f) * Motor::MAX_REGEN_NM;
        torque = -(int16_t)regenNm;  // Negative for regen
        
        // Apply regen curve based on motor speed
        torque = applyRegenCurve(torque);
        
    } else if (throttleInput > 1.0f) {
        // Throttle mode: map 0-100% to 0 to MAX_TORQUE
        if (currentGear == GearState::DRIVE) {
            float throttleNm = (throttleInput / 100.0f) * Motor::MAX_TORQUE_NM;
            torque = (int16_t)throttleNm;
            
        } else if (currentGear == GearState::REVERSE) {
            // Limit torque in reverse
            float throttleNm = (throttleInput / 100.0f) * Motor::MAX_REVERSE_NM;
            torque = -(int16_t)throttleNm;  // Negative for reverse
        }
        
        // Apply torque curve based on motor speed
        torque = applyTorqueCurve(torque);
    }
    
    // Apply deadzone
    applyDeadzone(torque);
    
    return torque;
}

int16_t VehicleControl::applyTorqueCurve(int16_t baseTorque) {
    // Find speed zone (0-1000, 1000-2000, 2000-4000, 4000-6000)
    float absSpeed = abs(motorSpeedRPM);
    int zone = 0;
    
    if (absSpeed < 1000) zone = 0;
    else if (absSpeed < 2000) zone = 1;
    else if (absSpeed < 4000) zone = 2;
    else zone = 3;
    
    // Apply curve multiplier
    float multiplier = Motor::TORQUE_CURVE[zone] / 100.0f;
    return (int16_t)(baseTorque * multiplier);
}

int16_t VehicleControl::applyRegenCurve(int16_t baseRegen) {
    // Find speed zone
    float absSpeed = abs(motorSpeedRPM);
    int zone = 0;
    
    if (absSpeed < 1000) zone = 0;
    else if (absSpeed < 2000) zone = 1;
    else if (absSpeed < 4000) zone = 2;
    else zone = 3;
    
    // Apply curve multiplier (regen increases with speed)
    float multiplier = Motor::REGEN_CURVE[zone] / 100.0f;
    return (int16_t)(baseRegen * multiplier);
}

//=============================================================================
// SAFETY CHECKS
//=============================================================================

bool VehicleControl::isSafeToApplyTorque() {
    // Check brake
    if (brakePressed) {
        return false;
    }
    
    // Check gear
    if (currentGear == GearState::NEUTRAL) {
        return false;
    }
    
    // Check system ready
    if (!systemReady) {
        return false;
    }
    
    return true;
}

void VehicleControl::applyDeadzone(int16_t& torque) {
    // Calculate deadzone in Nm
    int16_t deadzone = (int16_t)(Motor::MAX_TORQUE_NM * Motor::DEADZONE_PERCENT / 100.0f);
    
    // Apply deadzone
    if (abs(torque) < deadzone) {
        torque = 0;
    }
}

void VehicleControl::emergencyStop() {
    DEBUG_PRINTLN("EMERGENCY STOP");
    torqueDemand = 0;
    currentGear = GearState::NEUTRAL;
}