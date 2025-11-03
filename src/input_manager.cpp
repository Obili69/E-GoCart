#include "input_manager.h"

InputManager::InputManager()
    : mcp()
    , ads(I2C::ADS1115_ADDR)
    , startStopPressed(false)
    , ilClosed(true)
    , chargerWakeup(false)
    , brakePressed(false)
    , directionToggleCurrent(false)
    , directionToggleLast(false)
    , chargeAllowed(true)          // Default to allowed (safe default - LOW = allowed)
    , dischargeAllowed(true)       // Default to allowed (safe default - LOW = allowed)
    , chargeAllowedLast(true)
    , dischargeAllowedLast(true)
    , throttlePercent(0.0f)
    , regenPercent(0.0f)
    , throttleRawADC(0)
    , regenRawADC(0)
    , throttleMinADC(ADC_Cal::THROTTLE_MIN)
    , throttleMaxADC(ADC_Cal::THROTTLE_MAX)
    , regenMinADC(ADC_Cal::REGEN_MIN)
    , regenMaxADC(ADC_Cal::REGEN_MAX)
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
    
    // Configure MCP23017 Port A (Safety Critical Inputs)
    // Charge/Discharge Allow signals are active LOW (LOW = allowed, HIGH = not allowed)
    // Use INPUT_PULLUP for safety - if signal disconnects, defaults to HIGH (not allowed = safe)
    mcp.pinMode(MCP::CHARGE_ALLOW, INPUT_PULLUP);
    mcp.pinMode(MCP::DISCHARGE_ALLOW, INPUT_PULLUP);

    // Configure MCP23017 Port B (User Inputs)
    // Buttons are active HIGH (connect to 3.3V when pressed)
    // Use INPUT (no pullup) - external pulldowns handle default LOW state
    mcp.pinMode(MCP::BRAKE_SWITCH, INPUT);
    mcp.pinMode(MCP::DIRECTION_TOGGLE, INPUT);

    // Enable interrupts on both ports
    mcp.setupInterrupts(true, false, LOW);  // Mirrored, open-drain, active-low

    // Port A interrupts (safety critical - trigger on any change)
    mcp.setupInterruptPin(MCP::CHARGE_ALLOW, CHANGE);
    mcp.setupInterruptPin(MCP::DISCHARGE_ALLOW, CHANGE);

    // Port B interrupts (user inputs)
    mcp.setupInterruptPin(MCP::BRAKE_SWITCH, CHANGE);
    mcp.setupInterruptPin(MCP::DIRECTION_TOGGLE, RISING);  // Trigger on button press (active HIGH)
    
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
    // START_STOP_BUTTON: INPUT_PULLDOWN because the button connects to 3V3 when pressed
    // (GPIO 7 has external pulldown or is open drain, so we use PULLDOWN)
    pinMode(Pins::START_STOP_BUTTON, INPUT_PULLDOWN);
    pinMode(Pins::IL_SWITCH, INPUT_PULLUP);
    pinMode(Pins::CHARGER_WAKEUP, INPUT_PULLUP);
    
    // Configure interrupt pins
    pinMode(Pins::MCP_INT_A, INPUT_PULLUP);  // Port A interrupt (safety critical)
    pinMode(Pins::MCP_INT_B, INPUT_PULLUP);  // Port B interrupt (user inputs)
    
    DEBUG_PRINTLN("InputManager: Initialized successfully");
    return true;
}

void InputManager::update() {
    updateButtons();
    updateMCPPortA();  // Safety critical - check first
    updateMCPPortB();  // User inputs
    updateAnalog();
}

void InputManager::updateButtons() {
    unsigned long currentTime = millis();
    
    // Simple debounce: only update if enough time passed
    if (currentTime - lastDebounceTime < Timing::BUTTON_DEBOUNCE) {
        return;
    }
    lastDebounceTime = currentTime;
    
    // Read all buttons
    // START_STOP_BUTTON: Active HIGH (button pulls pin HIGH when pressed)
    // This is inverted from typical because GPIO 7 appears to be pulled LOW by hardware
    startStopPressed = (digitalRead(Pins::START_STOP_BUTTON) == HIGH);
    ilClosed = (digitalRead(Pins::IL_SWITCH) == HIGH);  // IL closed = HIGH (HIGH = closed, LOW = open)
    chargerWakeup = (digitalRead(Pins::CHARGER_WAKEUP) == HIGH);  // Active HIGH
}

void InputManager::updateMCPPortA() {
    // SAFETY CRITICAL - Check Port A (charge/discharge allow)
    // Check if interrupt triggered on Port A
    //if (digitalRead(Pins::MCP_INT_A) == HIGH) {
    //    return;  // No interrupt pending
    //}

    // Read Port A
    uint8_t portA = mcp.readGPIO(0);  // Read GPIOA register

    // Extract safety signals (active LOW - LOW = allowed, HIGH = not allowed)
    // Note: Logic is INVERTED - LOW on pin means allowed (true), HIGH means not allowed (false)
    bool chargeAllowPin = (portA & (1 << MCP::CHARGE_ALLOW)) == 0;  // LOW = allowed (true)
    bool dischargeAllowPin = (portA & (1 << MCP::DISCHARGE_ALLOW)) == 0;  // LOW = allowed (true)

    // Detect changes for logging
    if (chargeAllowPin != chargeAllowed) {
        DEBUG_PRINTF("SAFETY: Charge Allow changed: %s -> %s\n",
                     chargeAllowed ? "ALLOWED" : "NOT ALLOWED",
                     chargeAllowPin ? "ALLOWED" : "NOT ALLOWED");
    }

    if (dischargeAllowPin != dischargeAllowed) {
        DEBUG_PRINTF("SAFETY: Discharge Allow changed: %s -> %s\n",
                     dischargeAllowed ? "ALLOWED" : "NOT ALLOWED",
                     dischargeAllowPin ? "ALLOWED" : "NOT ALLOWED");
    }

    // Update states
    chargeAllowed = chargeAllowPin;
    dischargeAllowed = dischargeAllowPin;

    // Clear interrupt by reading INTCAP register
    mcp.readGPIO(0);
}

void InputManager::updateMCPPortB() {
    // Check if interrupt triggered on Port B
    if (digitalRead(Pins::MCP_INT_B) == HIGH) {
        return;  // No interrupt pending
    }

    // Read Port B
    uint8_t portB = mcp.readGPIO(1);  // Read GPIOB register

    // Extract button states (active HIGH - buttons connect to 3.3V when pressed)
    brakePressed = (portB & (1 << MCP::BRAKE_SWITCH)) != 0;
    directionToggleCurrent = (portB & (1 << MCP::DIRECTION_TOGGLE)) != 0;

    // Clear interrupt
    mcp.readGPIO(1);
}

void InputManager::handleSafetyInterrupt() {
    // This can be called from an ISR for immediate response to safety signals
    // For now, just set a flag - actual handling in updateMCPPortA()
    // Future: Could trigger emergency stop directly from ISR if needed
    updateMCPPortA();
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
    // Read throttle pot and store raw value
    throttleRawADC = ads.readADC(ADC::THROTTLE_POT);
    throttlePercent = mapADCToPercent(throttleRawADC, throttleMinADC, throttleMaxADC);

    // Read regen pot and store raw value
    regenRawADC = ads.readADC(ADC::REGEN_POT);
    regenPercent = mapADCToPercent(regenRawADC, regenMinADC, regenMaxADC);
}

float InputManager::mapADCToPercent(int16_t adcValue, int16_t minVal, int16_t maxVal) {
    // Map and constrain to 0-100%
    float percent = (float)(adcValue - minVal) * 100.0f / (float)(maxVal - minVal);
    return constrain(percent, 0.0f, 100.0f);
}

// Calibration methods
int16_t InputManager::getThrottleRawADC() const {
    return throttleRawADC;
}

int16_t InputManager::getRegenRawADC() const {
    return regenRawADC;
}

void InputManager::setCalibration(int16_t throttleMin, int16_t throttleMax,
                                   int16_t regenMin, int16_t regenMax) {
    throttleMinADC = throttleMin;
    throttleMaxADC = throttleMax;
    regenMinADC = regenMin;
    regenMaxADC = regenMax;
    DEBUG_PRINTLN("InputManager: Calibration values updated");
    DEBUG_PRINTF("  Throttle: %d - %d\n", throttleMin, throttleMax);
    DEBUG_PRINTF("  Regen: %d - %d\n", regenMin, regenMax);
}