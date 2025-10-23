#include "input_manager.h"

InputManager::InputManager()
    : mcp()
    , ads(I2C::ADS1115_ADDR)
    , startPressed(false)
    , stopPressed(false)
    , resetPressed(false)
    , ilClosed(true)
    , chargerWakeup(false)
    , brakePressed(false)
    , directionToggleCurrent(false)
    , directionToggleLast(false)
    , throttlePercent(0.0f)
    , regenPercent(0.0f)
    , lastDebounceTime(0)
    , lastDirectionToggleTime(0)
{
}

bool InputManager::begin() {
    DEBUG_PRINTLN("InputManager: Initializing...");
    
    // Initialize I2C
    Wire.begin(Pins::SDA, Pins::SCL);
    Wire.setClock(400000);  // 400kHz
    
    // Initialize MCP23017
    if (!mcp.begin_I2C(I2C::MCP23017_ADDR)) {
        DEBUG_PRINTLN("ERROR: MCP23017 not found!");
        return false;
    }
    
    // Configure MCP23017 pins (Port B = inputs with pullups)
    mcp.pinMode(MCP::BRAKE_SWITCH, INPUT_PULLUP);
    mcp.pinMode(MCP::DIRECTION_TOGGLE, INPUT_PULLUP);
    
    // Enable interrupts on Port B
    mcp.setupInterrupts(true, false, LOW);  // Mirrored, open-drain, active-low
    mcp.setupInterruptPin(MCP::BRAKE_SWITCH, CHANGE);
    mcp.setupInterruptPin(MCP::DIRECTION_TOGGLE, FALLING);  // Trigger on button press
    
    DEBUG_PRINTLN("MCP23017 initialized");
    
    // Initialize ADS1115
    if (!ads.begin()) {
        DEBUG_PRINTLN("ERROR: ADS1115 not found!");
        return false;
    }
    ads.setGain(1);  // ±4.096V range
    ads.setDataRate(7);  // 860 SPS
    
    DEBUG_PRINTLN("ADS1115 initialized");
    
    // Configure GPIO inputs
    pinMode(Pins::START_BUTTON, INPUT_PULLUP);
    pinMode(Pins::STOP_BUTTON, INPUT_PULLUP);
    pinMode(Pins::RESET_BUTTON, INPUT_PULLUP);
    pinMode(Pins::IL_SWITCH, INPUT_PULLUP);
    pinMode(Pins::CHARGER_WAKEUP, INPUT_PULLUP);
    
    // Configure interrupt pins
    pinMode(Pins::MCP_INT_B, INPUT_PULLUP);
    
    DEBUG_PRINTLN("InputManager: Initialized successfully");
    return true;
}

void InputManager::update() {
    updateButtons();
    updateMCP();
    updateAnalog();
}

void InputManager::updateButtons() {
    unsigned long currentTime = millis();
    
    // Simple debounce: only update if enough time passed
    if (currentTime - lastDebounceTime < Timing::BUTTON_DEBOUNCE) {
        return;
    }
    lastDebounceTime = currentTime;
    
    // Read all buttons (active LOW with pullup)
    startPressed = (digitalRead(Pins::START_BUTTON) == LOW);
    stopPressed = (digitalRead(Pins::STOP_BUTTON) == LOW);
    resetPressed = (digitalRead(Pins::RESET_BUTTON) == LOW);
    ilClosed = (digitalRead(Pins::IL_SWITCH) == LOW);  // IL closed = LOW
    chargerWakeup = (digitalRead(Pins::CHARGER_WAKEUP) == HIGH);  // Active HIGH
}

void InputManager::updateMCP() {
    // Check if interrupt triggered
    if (digitalRead(Pins::MCP_INT_B) == HIGH) {
        return;  // No interrupt pending
    }
    
    // Read Port B
    uint8_t portB = mcp.readGPIO(1);  // Read GPIOB register
    
    // Extract button states (active LOW)
    brakePressed = !(portB & (1 << MCP::BRAKE_SWITCH));
    directionToggleCurrent = !(portB & (1 << MCP::DIRECTION_TOGGLE));
    
    // Clear interrupt
    mcp.readGPIO(1);
}

bool InputManager::isDirectionTogglePressed() {
    unsigned long currentTime = millis();
    
    // Edge detection: only trigger on rising edge (button release)
    bool risingEdge = directionToggleCurrent && !directionToggleLast;
    directionToggleLast = directionToggleCurrent;
    
    // Debounce: require 200ms between toggles
    if (risingEdge && (currentTime - lastDirectionToggleTime > 200)) {
        lastDirectionToggleTime = currentTime;
        DEBUG_PRINTLN("Direction toggle detected");
        return true;
    }
    
    return false;
}

void InputManager::updateAnalog() {
    // Read throttle pot
    int16_t throttleADC = ads.readADC(ADC::THROTTLE_POT);
    throttlePercent = mapADCToPercent(throttleADC, 
                                     ADC_Cal::THROTTLE_MIN, 
                                     ADC_Cal::THROTTLE_MAX);
    
    // Read regen pot
    int16_t regenADC = ads.readADC(ADC::REGEN_POT);
    regenPercent = mapADCToPercent(regenADC, 
                                   ADC_Cal::REGEN_MIN, 
                                   ADC_Cal::REGEN_MAX);
}

float InputManager::mapADCToPercent(int16_t adcValue, int16_t minVal, int16_t maxVal) {
    // Map and constrain to 0-100%
    float percent = (float)(adcValue - minVal) * 100.0f / (float)(maxVal - minVal);
    return constrain(percent, 0.0f, 100.0f);
}