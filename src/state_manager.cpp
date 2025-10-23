#include "state_manager.h"
#include "data_structures.h"

StateManager::StateManager()
    : currentState(VehicleState::SLEEP)
    , previousState(VehicleState::SLEEP)
    , batteryArmed(false)
    , prechargeComplete(false)
    , systemReady(false)
    , emergencyShutdown(false)
    , stateEntryTime(0)
    , stopButtonPressTime(0)
{
}

void StateManager::begin() {
    DEBUG_PRINTLN("StateManager: Initializing...");
    
    // Configure all output pins
    pinMode(Pins::MAIN_CONTACTOR, OUTPUT);
    pinMode(Pins::PRECHARGE_RELAY, OUTPUT);
    pinMode(Pins::DMC_ENABLE, OUTPUT);
    pinMode(Pins::NLG_ENABLE, OUTPUT);
    pinMode(Pins::BMS_ENABLE, OUTPUT);
    pinMode(Pins::WATER_PUMP, OUTPUT);
    pinMode(Pins::NEXTION_POWER, OUTPUT);
    
    // Ensure all outputs are OFF
    shutdownSystems();
    
    // Configure wakeup sources for deep sleep
    esp_sleep_enable_ext0_wakeup((gpio_num_t)Pins::START_BUTTON, LOW);  // Wake on button press
    
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
    // This state is only reached briefly before entering deep sleep
    // Actual sleep is triggered by handleStopButton()
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
    // System is ready, waiting for:
    // - Throttle input → DRIVE
    // - Charger connection → CHARGING
    // Already handled by external events
}

void StateManager::handleDriveState() {
    // Monitor critical conditions
    if (emergencyShutdown) {
        DEBUG_PRINTLN("Emergency shutdown!");
        disarmBattery();
        transitionTo(VehicleState::READY);
        emergencyShutdown = false;
    }
    
    // Check precharge status
    if (batteryArmed && !prechargeComplete) {
        checkPrecharge();
    }
}

void StateManager::handleChargingState() {
    // Monitor charger status
    // Handled by external NLG events
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
            break;
            
        case VehicleState::READY:
            systemReady = true;
            break;
            
        case VehicleState::DRIVE:
            startPrecharge();
            break;
            
        case VehicleState::CHARGING:
            digitalWrite(Pins::NLG_ENABLE, HIGH);
            digitalWrite(Pins::BMS_ENABLE, HIGH);
            break;
    }
}

//=============================================================================
// EXTERNAL EVENT HANDLERS
//=============================================================================

void StateManager::handleStartButton() {
    DEBUG_PRINTLN("Start button pressed");
    
    if (currentState == VehicleState::SLEEP) {
        transitionTo(VehicleState::INIT);
    }
}

void StateManager::handleStopButton() {
    DEBUG_PRINTLN("Stop button pressed");
    
    stopButtonPressTime = millis();
    
    // Start timeout timer
    // If held for SLEEP_TIMEOUT, enter sleep
}

void StateManager::handleILOpen() {
    DEBUG_PRINTLN("Interlock opened!");
    emergencyShutdown = true;
    disarmBattery();
}

void StateManager::handleChargerConnected() {
    DEBUG_PRINTLN("Charger connected");
    
    if (currentState == VehicleState::READY) {
        transitionTo(VehicleState::CHARGING);
    }
}

void StateManager::handleChargerDisconnected() {
    DEBUG_PRINTLN("Charger disconnected");
    
    if (currentState == VehicleState::CHARGING) {
        digitalWrite(Pins::NLG_ENABLE, LOW);
        transitionTo(VehicleState::READY);
    }
}

//=============================================================================
// SYSTEM CONTROL
//=============================================================================

void StateManager::initializeSystems() {
    DEBUG_PRINTLN("Initializing systems...");
    
    // Power on BMS
    digitalWrite(Pins::BMS_ENABLE, HIGH);
    delay(100);
    
    // Power on Nextion display
    digitalWrite(Pins::NEXTION_POWER, HIGH);
    delay(100);
    
    DEBUG_PRINTLN("Systems initialized");
}

void StateManager::shutdownSystems() {
    DEBUG_PRINTLN("Shutting down systems...");
    
    // Disable all power outputs
    digitalWrite(Pins::MAIN_CONTACTOR, LOW);
    digitalWrite(Pins::PRECHARGE_RELAY, LOW);
    digitalWrite(Pins::DMC_ENABLE, LOW);
    digitalWrite(Pins::NLG_ENABLE, LOW);
    digitalWrite(Pins::BMS_ENABLE, LOW);
    digitalWrite(Pins::WATER_PUMP, LOW);
    digitalWrite(Pins::NEXTION_POWER, LOW);
    
    batteryArmed = false;
    prechargeComplete = false;
    systemReady = false;
}

void StateManager::enterDeepSleep() {
    DEBUG_PRINTLN("Entering deep sleep...");
    delay(100);  // Allow serial to flush
    
    esp_deep_sleep_start();
}

//=============================================================================
// BATTERY MANAGEMENT
//=============================================================================

void StateManager::startPrecharge() {
    DEBUG_PRINTLN("Starting precharge sequence...");
    
    // Close precharge relay
    digitalWrite(Pins::PRECHARGE_RELAY, HIGH);
    
    batteryArmed = false;
    prechargeComplete = false;
    
    DEBUG_PRINTLN("Precharge relay closed");
}

void StateManager::checkPrecharge() {
    static unsigned long prechargeStartTime = 0;
    if (prechargeStartTime == 0) {
        prechargeStartTime = millis();
        DEBUG_PRINTLN("Precharge: Timer started");
    }

    // Get latest voltage data from CAN
    BMSData bmsData = sharedBMSData.get();
    DMCData dmcData = sharedDMCData.get();

    float bmsVoltage = bmsData.voltage;
    float dmcVoltage = dmcData.dcVoltage;
    float voltageDiff = abs(bmsVoltage - dmcVoltage);

    DEBUG_PRINTF("Precharge: BMS=%.1fV, DMC=%.1fV, Diff=%.1fV\n",
                 bmsVoltage, dmcVoltage, voltageDiff);

    // Check if voltages are within tolerance
    if (voltageDiff <= Battery::PRECHARGE_TOLERANCE) {
        DEBUG_PRINTLN("Precharge: Voltage match! Completing precharge...");
        armBattery();
        prechargeStartTime = 0;
        return;
    }

    // Timeout check (safety fallback)
    if (millis() - prechargeStartTime > Timing::PRECHARGE_TIMEOUT) {
        DEBUG_PRINTLN("ERROR: Precharge timeout!");
        DEBUG_PRINTF("Final voltages: BMS=%.1fV, DMC=%.1fV, Diff=%.1fV\n",
                     bmsVoltage, dmcVoltage, voltageDiff);

        // Abort precharge
        digitalWrite(Pins::PRECHARGE_RELAY, LOW);
        prechargeComplete = false;
        batteryArmed = false;
        prechargeStartTime = 0;

        // Transition back to READY state
        transitionTo(VehicleState::READY);
    }
}

void StateManager::armBattery() {
    DEBUG_PRINTLN("Arming battery...");
    
    // Close main contactor
    digitalWrite(Pins::MAIN_CONTACTOR, HIGH);
    delay(50);
    
    // Open precharge relay
    digitalWrite(Pins::PRECHARGE_RELAY, LOW);
    
    // Enable DMC
    digitalWrite(Pins::DMC_ENABLE, HIGH);
    
    // Enable cooling pump
    digitalWrite(Pins::WATER_PUMP, HIGH);
    
    batteryArmed = true;
    prechargeComplete = true;
    
    DEBUG_PRINTLN("Battery armed");
}

void StateManager::disarmBattery() {
    DEBUG_PRINTLN("Disarming battery...");
    
    // Disable DMC first
    digitalWrite(Pins::DMC_ENABLE, LOW);
    delay(50);
    
    // Open main contactor
    digitalWrite(Pins::MAIN_CONTACTOR, LOW);
    
    // Open precharge relay
    digitalWrite(Pins::PRECHARGE_RELAY, LOW);
    
    // Stop cooling pump
    digitalWrite(Pins::WATER_PUMP, LOW);
    
    batteryArmed = false;
    prechargeComplete = false;
    
    DEBUG_PRINTLN("Battery disarmed");
}