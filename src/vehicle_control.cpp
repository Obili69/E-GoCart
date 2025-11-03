#include "vehicle_control.h"
#include "bms_manager.h"
#include "input_manager.h"
#include "contactor_manager.h"

VehicleControl::VehicleControl()
    : currentGear(GearState::NEUTRAL)
    , systemReady(false)
    , throttleInput(0.0f)
    , regenInput(0.0f)
    , brakePressed(false)
    , motorSpeedRPM(0.0f)
    , torqueDemand(0)
    , bmsManager(nullptr)
    , inputManager(nullptr)
    , contactorManager(nullptr)
    , lastUpdateTime(0)
    , regenEnabled(true)
{
}

void VehicleControl::begin(BMSManager* bmsMgr, InputManager* inputMgr, ContactorManager* contactorMgr) {
    bmsManager = bmsMgr;
    inputManager = inputMgr;
    contactorManager = contactorMgr;
    DEBUG_PRINTLN("VehicleControl: Initialized");
    if (bmsManager != nullptr) {
        DEBUG_PRINTLN("  BMS display control enabled");
    }
    if (inputManager != nullptr) {
        DEBUG_PRINTLN("  Safety allowance monitoring enabled");
    }
    if (contactorManager != nullptr) {
        DEBUG_PRINTLN("  Emergency contactor shutdown enabled");
    }
}

void VehicleControl::update() {
    // Check safety allowances first
    checkDischargeAllowance();
    checkChargeAllowance();

    // Calculate torque demand
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

    // Short press: Toggle between D and R only (doesn't affect Neutral)
    if (currentGear == GearState::NEUTRAL) {
        currentGear = GearState::DRIVE;
        DEBUG_PRINTLN("Gear: DRIVE");
    } else if (currentGear == GearState::DRIVE) {
        currentGear = GearState::REVERSE;
        DEBUG_PRINTLN("Gear: REVERSE");
    } else if (currentGear == GearState::REVERSE) {
        currentGear = GearState::DRIVE;
        DEBUG_PRINTLN("Gear: DRIVE");
    }

    // Update BMS display based on new gear state
    updateBMSDisplay();
}

void VehicleControl::setNeutral() {
    currentGear = GearState::NEUTRAL;
    DEBUG_PRINTLN("Gear: NEUTRAL");

    // Update BMS display based on new gear state
    updateBMSDisplay();
}

void VehicleControl::updateBMSDisplay() {
    if (bmsManager == nullptr) {
        return;  // BMS manager not available
    }

    // Display ON in NEUTRAL, OFF in DRIVE/REVERSE
    if (currentGear == GearState::NEUTRAL) {
        bmsManager->setDisplayEnabled(true);   // Display ON
    } else {
        bmsManager->setDisplayEnabled(false);  // Display OFF
    }
}

//=============================================================================
// TORQUE CALCULATION
//=============================================================================

int16_t VehicleControl::calculateTorque() {
    int16_t torque = 0;

    // Regen has priority over throttle (only when throttle is zero)
    // Regen: Only in DRIVE gear, can work with brake pressed, requires charge allowed
    if (throttleInput < 1.0f && regenInput > 1.0f && regenEnabled && currentGear == GearState::DRIVE) {
        // Regen mode: map 0-100% to 0 to -MAX_REGEN_NM
        float regenNm = (regenInput / 100.0f) * Motor::MAX_REGEN_NM;
        torque = -(int16_t)regenNm;  // Negative for regen

        // Apply regen curve based on motor speed
        torque = applyRegenCurve(torque);

    } else if (throttleInput > 1.0f) {
        // Throttle mode: requires safety checks (no brake, system ready, not neutral)
        if (!isSafeToApplyTorque()) {
            return 0;
        }

        // Apply throttle based on gear
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

    // Emergency shutdown contactors FIRST (safety critical!)
    if (contactorManager != nullptr) {
        contactorManager->emergencyShutdown();
    }

    torqueDemand = 0;
    currentGear = GearState::NEUTRAL;
}

//=============================================================================
// SAFETY ALLOWANCE MONITORING
//=============================================================================

void VehicleControl::checkDischargeAllowance() {
    if (inputManager == nullptr) {
        return;
    }

    // Check if discharge is not allowed during driving
    if (!inputManager->isDischargeAllowed()) {
        // SAFETY: Force vehicle to Neutral - no discharge allowed!
        if (currentGear != GearState::NEUTRAL) {
            DEBUG_PRINTLN("SAFETY: Discharge not allowed - forcing NEUTRAL!");
            setNeutral();
        }

        // Also disable torque immediately
        torqueDemand = 0;
    }
}

void VehicleControl::checkChargeAllowance() {
    if (inputManager == nullptr) {
        return;
    }

    // Check if charge is not allowed - disables regen but allows driving
    if (!inputManager->isChargeAllowed()) {
        // SAFETY: Disable regen - no charge allowed!
        if (regenEnabled) {
            DEBUG_PRINTLN("SAFETY: Charge not allowed - disabling REGEN!");
            regenEnabled = false;
        }
    } else {
        // Re-enable regen if charge becomes allowed again
        if (!regenEnabled) {
            DEBUG_PRINTLN("SAFETY: Charge allowed - enabling REGEN!");
            regenEnabled = true;
        }
    }
}