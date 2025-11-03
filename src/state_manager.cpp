#include "state_manager.h"
#include "bms_manager.h"
#include "contactor_manager.h"
#include "data_structures.h"
#include "driver/rtc_io.h"

StateManager::StateManager()
    : currentState(VehicleState::SLEEP)
    , previousState(VehicleState::SLEEP)
    , batteryArmed(false)
    , systemReady(false)
    , emergencyShutdown(false)
    , bmsManager(nullptr)
    , contactorManager(nullptr)
    , pumpRunning(false)
    , stateEntryTime(0)
    , stopButtonPressTime(0)
{
}
void StateManager::begin(BMSManager* bmsMgr, ContactorManager* contactorMgr) {
    DEBUG_PRINTLN("StateManager: Initializing...");

    bmsManager = bmsMgr;
    contactorManager = contactorMgr;

    // Configure all output pins
    pinMode(Pins::BMS_PWR, OUTPUT);          // BMS main power
    pinMode(Pins::DMC_ENABLE, OUTPUT);
    pinMode(Pins::WATER_PUMP, OUTPUT);
    pinMode(Pins::NEXTION_POWER, OUTPUT);

    // Ensure all outputs are OFF initially
    digitalWrite(Pins::BMS_PWR, LOW);
    digitalWrite(Pins::DMC_ENABLE, LOW);
    digitalWrite(Pins::NLG_ENABLE, LOW);
    digitalWrite(Pins::WATER_PUMP, LOW);
    digitalWrite(Pins::NEXTION_POWER, LOW);

    // Power on BMS immediately so it's ready for config
    DEBUG_PRINTLN("  Powering on BMS...");
    digitalWrite(Pins::BMS_PWR, HIGH);

    // Wait 2 seconds for BMS to boot
    DEBUG_PRINTLN("  Waiting 2s for BMS boot...");
    delay(2000);

    DEBUG_PRINTLN("  BMS powered and ready for configuration");
    
    // Configure wakeup sources for deep sleep
    // Use ext1 wakeup with ANY_HIGH mode (wakes when button/charger goes HIGH)
    // Wake on START button press OR charger connection
    uint64_t wakeup_pin_mask = (1ULL << GPIO_NUM_7) | (1ULL << GPIO_NUM_9);  // GPIO 7 = START, GPIO 9 = CHARGER
    esp_sleep_enable_ext1_wakeup(wakeup_pin_mask, ESP_EXT1_WAKEUP_ANY_HIGH);

    // Configure RTC GPIO pull-ups/pull-downs for stable wake-up detection
    // Since we're using ANY_HIGH mode, tie pins to GND with pull-downs during sleep
    // This ensures stable LOW state and proper HIGH detection for wake-up
    rtc_gpio_pulldown_en(GPIO_NUM_7);   // START_STOP_BUTTON
    rtc_gpio_pullup_dis(GPIO_NUM_7);
    rtc_gpio_pulldown_en(GPIO_NUM_9);   // CHARGER_WAKEUP
    rtc_gpio_pullup_dis(GPIO_NUM_9);

    DEBUG_PRINTLN("StateManager: Initialized");
}

void StateManager::update() {
    
    switch (currentState) {
        case VehicleState::SLEEP:
            handleSleepState();
            break;
            
        case VehicleState::INIT:
            handleInitState();
            break;
            
        case VehicleState::READY:
            handleReadyState();
            break;
            
        case VehicleState::DRIVE:
            handleDriveState();
            break;
            
        case VehicleState::CHARGING:
            handleChargingState();
            break;
    }
}

//=============================================================================
// STATE HANDLERS
//=============================================================================

void StateManager::handleSleepState() {
    // This state is reached when system should be off
    // Deep sleep is triggered by transitionTo(SLEEP) entry action
    // If we're here and still awake, we're in debug mode or waiting for deep sleep
    // Just stay in this state - do nothing
}

void StateManager::handleInitState() {
    unsigned long elapsed = millis() - stateEntryTime;
    
    // Wait for systems to stabilize
    if (elapsed < 500) {
        return;
    }
    
    // Check if BMS is communicating (will be set by CANManager)
    // For now, assume ready after timeout
    DEBUG_PRINTLN("Initialization complete");
    systemReady = true;
    transitionTo(VehicleState::READY);
}

void StateManager::handleReadyState() {
    // READY is a brief transition state - arm battery based on requested mode
    if (requestedMode == 1) {
        DEBUG_PRINTLN("READY → DRIVE (requestedMode=1)");
        armBatteryForDischarging();
        transitionTo(VehicleState::DRIVE);
    } else if (requestedMode == 2) {
        DEBUG_PRINTLN("READY → CHARGING (requestedMode=2)");
        armBatteryForCharging();
        transitionTo(VehicleState::CHARGING);
    } else {
        // No mode requested - this shouldn't happen, but go to sleep as fallback
        DEBUG_PRINTLN("WARNING: READY with no requestedMode - going to SLEEP");
        transitionTo(VehicleState::SLEEP);
    }
}

void StateManager::handleDriveState() {
    // Monitor critical conditions
    if (emergencyShutdown) {
        StateManager::handleEmergencyShutdown();
        //DEBUG_PRINTLN("Emergency shutdown!");
        //disarmBattery();
        //transitionTo(VehicleState::SLEEP);
        //emergencyShutdown = false;
    }

    // Battery already armed in READY state
    // Just monitor for emergency conditions here
}

void StateManager::handleChargingState() {
    // Get current BMS data and config
    BMSData bmsData = sharedBMSData.get();
    RuntimeConfigData config = sharedRuntimeConfig.get();

    // Calculate target pack voltage (in 0.1V units)
    float targetPackVoltage = config.maxChargeVoltagePerCell * Battery::NUM_CELLS;  // e.g., 4.17V × 104 = 433.68V
    uint16_t targetPackVoltageScaled = (uint16_t)(targetPackVoltage * 10.0f);  // Convert to 0.1V units = 4336

    // Determine stop condition based on voltage target
    bool shouldStop = false;

    if (config.maxChargeVoltagePerCell >= 4.17f) {
        // Full charge mode (≥4.17V/cell) - wait for SOC = 100% to allow balancing
        if (bmsData.soc >= 100) {
            DEBUG_PRINTF("Full charge complete: SOC = %d%%\n", bmsData.soc);
            shouldStop = true;
        }
    } else {
        // Limited voltage mode (<4.17V/cell) - stop at voltage target
        if (bmsData.voltage >= targetPackVoltageScaled) {
            DEBUG_PRINTF("Voltage target reached: %d >= %d (0.1V units)\n",
                        bmsData.voltage, targetPackVoltageScaled);
            DEBUG_PRINTF("Pack voltage: %.1fV, Target: %.1fV\n",
                        bmsData.voltage / 10.0f, targetPackVoltage);
            shouldStop = true;
        }
    }

    if (shouldStop) {
        DEBUG_PRINTLN("Stopping charge - target reached");
        digitalWrite(Pins::NLG_ENABLE, LOW);
        disarmBattery();
        requestedMode = 0;
        transitionTo(VehicleState::SLEEP);
        return;
    }

    // Check for charger disconnect (pin goes LOW)
    if (digitalRead(Pins::CHARGER_WAKEUP) == LOW) {
        DEBUG_PRINTLN("Charger disconnected - stopping charge");
        digitalWrite(Pins::NLG_ENABLE, LOW);
        disarmBattery();
        requestedMode = 0;
        transitionTo(VehicleState::SLEEP);
        return;
    }

    // Check for emergency conditions
    if (emergencyShutdown) {
        DEBUG_PRINTLN("Emergency shutdown during charging!");
        digitalWrite(Pins::NLG_ENABLE, LOW);
        disarmBattery();
        requestedMode = 0;
        transitionTo(VehicleState::SLEEP);
        emergencyShutdown = false;
        return;
    }
}

//=============================================================================
// STATE TRANSITIONS
//=============================================================================

void StateManager::transitionTo(VehicleState newState) {
    if (newState == currentState) {
        return;
    }
    
    DEBUG_PRINTF("State transition: %d -> %d\n", (int)currentState, (int)newState);
    
    previousState = currentState;
    currentState = newState;
    stateEntryTime = millis();
    
    // Execute entry actions
    switch (newState) {
        case VehicleState::SLEEP:
            shutdownSystems();
            break;

        case VehicleState::INIT:
            initializeSystems();
            digitalWrite(Pins::NEXTION_POWER, HIGH);  // Power on Nextion display
            break;

        case VehicleState::READY:
            systemReady = true;
            // Battery armed in handleReadyState()
            break;

        case VehicleState::DRIVE:
            digitalWrite(Pins::DMC_ENABLE, HIGH);
            DEBUG_PRINTLN("DMC enabled");
            // Battery armed for discharge in handleReadyState()
            break;

        case VehicleState::CHARGING:
            digitalWrite(Pins::NLG_ENABLE, HIGH);
            // Battery armed for charge in handleReadyState()
            break;

        default:
            break;
    }
}

//=============================================================================
// EXTERNAL EVENT HANDLERS
//=============================================================================

void StateManager::handleWakeup(bool fromCharger) {
    DEBUG_PRINTLN("Handling wakeup...");

    if (fromCharger) {
        DEBUG_PRINTLN("Wake reason: CHARGER");
        requestedMode = 2;  // Charging mode
    } else {
        DEBUG_PRINTLN("Wake reason: BUTTON");
        requestedMode = 1;  // Drive mode
    }

    // Transition from SLEEP to INIT
    if (currentState == VehicleState::SLEEP) {
        transitionTo(VehicleState::INIT);
    }
}

void StateManager::handleChargerPinChange(bool connected) {
    if (connected) {
        DEBUG_PRINTLN("Charger pin HIGH - charger connected");

        if (currentState == VehicleState::READY) {
            // In READY state - transition to charging
            requestedMode = 2;
            transitionTo(VehicleState::CHARGING);
        } else if (currentState == VehicleState::DRIVE) {
            // Charger plugged during drive - emergency shutdown and restart
            DEBUG_PRINTLN("Charger connected during DRIVE - restarting to CHARGING");
            requestedMode = 2;  // Set mode for after restart
            transitionTo(VehicleState::SLEEP);  // Will wake immediately from charger pin
        }
        // INIT/CHARGING/SLEEP states: ignore (will be handled by flow)

    } else {
        DEBUG_PRINTLN("Charger pin LOW - charger disconnected");

        if (currentState == VehicleState::CHARGING) {
            // Charger unplugged during charging - shutdown
            digitalWrite(Pins::NLG_ENABLE, LOW);
            requestedMode = 0;
            transitionTo(VehicleState::SLEEP);
        }
        // Other states: ignore
    }
}

void StateManager::handleStopRequest() {
    DEBUG_PRINTLN("Stop button pressed");

    if (currentState == VehicleState::DRIVE) {
        // Only allow stop from DRIVE if in NEUTRAL
        // (gear check should be done in main.cpp before calling this)
        requestedMode = 0;
        transitionTo(VehicleState::SLEEP);
    }
}

void StateManager::handleEmergencyShutdown() {
    DEBUG_PRINTLN("EMERGENCY SHUTDOWN!");
    emergencyShutdown = true;

    // Emergency shutdown contactors FIRST (safety critical!)
    if (contactorManager != nullptr) {
        contactorManager->emergencyShutdown();
    }

    // Immediate power cut - no CAN delays!
    digitalWrite(Pins::DMC_ENABLE, LOW);
    DEBUG_PRINTLN("DMC disbled");
    digitalWrite(Pins::WATER_PUMP, LOW);
    digitalWrite(Pins::BMS_PWR, LOW);
    digitalWrite(Pins::NLG_ENABLE, LOW);

    batteryArmed = false;
    requestedMode = 0;

    transitionTo(VehicleState::SLEEP);
}

//=============================================================================
// SYSTEM CONTROL
//=============================================================================

void StateManager::initializeSystems() {
    DEBUG_PRINTLN("Initializing systems...");

    // BMS already powered and configured during StateManager.begin()
    // Nextion powered on in transitionTo(INIT)
    // Just wait for systems to stabilize

    delay(100);

    DEBUG_PRINTLN("Systems initialized");
}

void StateManager::shutdownSystems() {
    DEBUG_PRINTLN("Shutting down systems...");

    // Disarm battery first (if armed)
    if (batteryArmed) {
        disarmBattery();
    }

    // Disable all power outputs
    digitalWrite(Pins::DMC_ENABLE, LOW);
    DEBUG_PRINTLN("DMC disbled");
    digitalWrite(Pins::NLG_ENABLE, LOW);
    digitalWrite(Pins::WATER_PUMP, LOW);
    digitalWrite(Pins::NEXTION_POWER, LOW);
    // BMS_PWR stays on until disarmBattery() is called

    batteryArmed = false;
    systemReady = false;
    enterDeepSleep();
}

void StateManager::enterDeepSleep() {
    DEBUG_PRINTLN("Entering deep sleep...");
    delay(100);  // Allow serial to flush

    RuntimeConfigData config = sharedRuntimeConfig.get();

    // Check if we should disable charger wakeup (reached charge target)
    bool disableChargerWakeup = shouldDisableChargerWakeup();

    if (disableChargerWakeup) {
        DEBUG_PRINTLN("Charge target reached - disabling charger wakeup pin");
        DEBUG_PRINTLN("Press START button to re-enable charging");

        // Only enable START button wakeup (GPIO 7)
        uint64_t wakeup_pin_mask = (1ULL << GPIO_NUM_7);
        esp_sleep_enable_ext1_wakeup(wakeup_pin_mask, ESP_EXT1_WAKEUP_ANY_HIGH);

        // Configure pull-downs
        rtc_gpio_pulldown_en(GPIO_NUM_7);  // START button
        rtc_gpio_pullup_dis(GPIO_NUM_7);

    } else {
        DEBUG_PRINTLN("Enabling both START and CHARGER wakeup pins");

        // Enable both START button (GPIO 7) and CHARGER (GPIO 9)
        uint64_t wakeup_pin_mask = (1ULL << GPIO_NUM_7) | (1ULL << GPIO_NUM_9);
        esp_sleep_enable_ext1_wakeup(wakeup_pin_mask, ESP_EXT1_WAKEUP_ANY_HIGH);

        // Configure pull-downs
        rtc_gpio_pulldown_en(GPIO_NUM_7);  // START button
        rtc_gpio_pullup_dis(GPIO_NUM_7);
        rtc_gpio_pulldown_en(GPIO_NUM_9);  // CHARGER
        rtc_gpio_pullup_dis(GPIO_NUM_9);
    }

    if (!config.debugMode) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_deep_sleep_start();
    } else {
        DEBUG_PRINTLN("Debug mode - staying awake");
    }
}

//=============================================================================
// BATTERY MANAGEMENT (NEW - Uses Contactor Manager)
//=============================================================================

void StateManager::armBatteryForCharging() {
    DEBUG_PRINTLN("Arming battery for CHARGING...");

    if (contactorManager == nullptr) {
        DEBUG_PRINTLN("ERROR: Contactor Manager not available!");
        return;
    }

    // Start charge contactor sequence:
    // 1) Arm BMS via CAN
    // 2) Close charge precharge relay (pin 17)
    // 3) Wait precharge time
    // 4) Close main charge contactor (pin 13)
    // 5) Open precharge relay
    if (contactorManager->startChargeSequence()) {
        batteryArmed = true;
        DEBUG_PRINTLN("Battery armed for charging");
        // NOTE: Cooling pump is now temperature-controlled via updateCoolingPump()
    } else {
        DEBUG_PRINTLN("ERROR: Failed to arm battery for charging!");
    }
}

void StateManager::armBatteryForDischarging() {
    DEBUG_PRINTLN("Arming battery for DISCHARGING...");

    if (contactorManager == nullptr) {
        DEBUG_PRINTLN("ERROR: Contactor Manager not available!");
        return;
    }

    // Start discharge contactor sequence:
    // 1) Arm BMS via CAN
    // 2) Close discharge precharge relay (pin 14)
    // 3) Wait precharge time
    // 4) Close main discharge contactor (pin 18)
    // 5) Open precharge relay
    if (contactorManager->startDischargeSequence()) {
        batteryArmed = true;
        DEBUG_PRINTLN("Battery armed for discharging");
        // NOTE: Cooling pump is now temperature-controlled via updateCoolingPump()
    } else {
        DEBUG_PRINTLN("ERROR: Failed to arm battery for discharging!");
    }
}

void StateManager::disarmBattery() {
    DEBUG_PRINTLN("Disarming battery...");

    // Disable DMC first
    digitalWrite(Pins::DMC_ENABLE, LOW);
    DEBUG_PRINTLN("DMC disbled");

    // Stop cooling pump (force off when disarmed)
    digitalWrite(Pins::WATER_PUMP, LOW);
    pumpRunning = false;

    // Use contactor manager to safely open contactors
    if (contactorManager != nullptr) {
        DEBUG_PRINTLN("  Opening contactors via ContactorManager...");
        contactorManager->openAllContactors(false);  // Wait for current to reach zero
    }

    // Power off BMS
    DEBUG_PRINTLN("  Powering off BMS...");
    digitalWrite(Pins::BMS_PWR, LOW);

    batteryArmed = false;

    DEBUG_PRINTLN("Battery disarmed");
}

//=============================================================================
// COOLING MANAGEMENT (Temperature-based with Hysteresis)
//=============================================================================

void StateManager::updateCoolingPump(float motorTemp, float inverterTemp) {
    // Only control pump when battery is armed
    if (!batteryArmed) {
        if (pumpRunning) {
            digitalWrite(Pins::WATER_PUMP, LOW);
            pumpRunning = false;
            DEBUG_PRINTLN("Cooling pump: OFF (battery disarmed)");
        }
        return;
    }

    // Get the highest temperature
    float maxTemp = max(motorTemp, inverterTemp);

    // Hysteresis control:
    // - Turn ON at 60°C
    // - Turn OFF at 45°C (60 - 15)
    if (!pumpRunning && maxTemp >= Cooling::PUMP_ON_TEMP_C) {
        // Temperature reached ON threshold
        digitalWrite(Pins::WATER_PUMP, HIGH);
        pumpRunning = true;
        DEBUG_PRINTF("Cooling pump: ON (Motor: %.1f°C, Inverter: %.1f°C)\n", motorTemp, inverterTemp);
    }
    else if (pumpRunning && maxTemp <= Cooling::PUMP_OFF_TEMP_C) {
        // Temperature dropped below OFF threshold
        digitalWrite(Pins::WATER_PUMP, LOW);
        pumpRunning = false;
        DEBUG_PRINTF("Cooling pump: OFF (Motor: %.1f°C, Inverter: %.1f°C)\n", motorTemp, inverterTemp);
    }
    // Else: Stay in current state (hysteresis prevents rapid cycling)
}

//=============================================================================
// HELPERS
//=============================================================================

bool StateManager::shouldDisableChargerWakeup() {
    // Disable charger wakeup if we just finished charging and reached the target
    if (previousState != VehicleState::CHARGING) {
        return false;  // Not coming from charging state
    }

    BMSData bmsData = sharedBMSData.get();
    RuntimeConfigData config = sharedRuntimeConfig.get();

    // Calculate target pack voltage
    float targetPackVoltage = config.maxChargeVoltagePerCell * Battery::NUM_CELLS;
    uint16_t targetPackVoltageScaled = (uint16_t)(targetPackVoltage * 10.0f);

    // Check if we reached the charge target
    if (config.maxChargeVoltagePerCell >= 4.17f) {
        // Full charge - check SOC
        if (bmsData.soc >= 100) {
            return true;
        }
    } else {
        // Limited voltage - check voltage
        if (bmsData.voltage >= targetPackVoltageScaled) {
            return true;
        }
    }

    return false;
}