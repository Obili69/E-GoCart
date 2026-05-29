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
    , i2cOk(false)
{
}

bool InputManager::recoverI2CBus() {
    // Clock out any stuck I2C transaction by bit-banging SCL while SDA is LOW.
    // Called before Wire.begin() so we work directly on GPIO.
    DEBUG_PRINTLN("InputManager: Attempting I2C bus recovery...");

    pinMode(Pins::SDA, INPUT_PULLUP);
    pinMode(Pins::SCL, OUTPUT);

    bool sdaStuck = (digitalRead(Pins::SDA) == LOW);
    if (sdaStuck) {
        DEBUG_PRINTLN("  I2C SDA stuck LOW — clocking out...");
        for (int i = 0; i < 9; i++) {
            digitalWrite(Pins::SCL, HIGH);
            delayMicroseconds(10);
            digitalWrite(Pins::SCL, LOW);
            delayMicroseconds(10);
        }
        // Generate STOP condition: SDA LOW→HIGH while SCL HIGH
        pinMode(Pins::SDA, OUTPUT);
        digitalWrite(Pins::SDA, LOW);
        delayMicroseconds(10);
        digitalWrite(Pins::SCL, HIGH);
        delayMicroseconds(10);
        digitalWrite(Pins::SDA, HIGH);
        delayMicroseconds(10);
    }

    // Release both lines to I2C driver
    pinMode(Pins::SDA, INPUT_PULLUP);
    pinMode(Pins::SCL, INPUT_PULLUP);
    delay(10);

    bool recovered = (digitalRead(Pins::SDA) == HIGH);
    DEBUG_PRINTF("  I2C recovery: SDA now %s\n", recovered ? "HIGH (OK)" : "still LOW (FAILED)");
    return recovered;
}

bool InputManager::begin() {
    DEBUG_PRINTLN("InputManager: Initializing...");

    // Recover I2C bus before initializing (clears any stuck transaction from previous crash)
    recoverI2CBus();

    // Initialize I2C with a short timeout so reads never block indefinitely
    Wire.begin(Pins::SDA, Pins::SCL);
    Wire.setClock(400000);  // 400kHz
    Wire.setTimeout(10);    // 10ms max per I2C operation — prevents watchdog cascade on bus lockup

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
    
    i2cOk = true;
    DEBUG_PRINTLN("InputManager: Initialized successfully");
    return true;
}

void InputManager::update() {
    updateButtons();
    if (!i2cOk) return;  // Skip I2C reads if bus failed — prevents watchdog cascade
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
    // Always poll Port B directly - do NOT gate on interrupt.
    // Interrupts are mirrored (INTA=INTB): a Port A read clears the shared
    // interrupt flag, so a simultaneous brake press on Port B would be missed
    // entirely if we returned early here. The brake is safety-critical.
    uint8_t portB = mcp.readGPIO(1);

    brakePressed = (portB & (1 << MCP::BRAKE_SWITCH)) != 0;
    directionToggleCurrent = (portB & (1 << MCP::DIRECTION_TOGGLE)) != 0;
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
    int16_t newThrottle = ads.readADC(ADC::THROTTLE_POT);

    // Stale-read detection: if the value is identical to the previous read AND
    // at a high level, count consecutive matches. After 5 in a row (~100ms) treat
    // it as a stuck/failed ADC and zero the throttle. A real pot always dithers.
    static uint8_t throttleSameCount = 0;
    if (newThrottle == throttleRawADC && newThrottle > (throttleMaxADC / 2)) {
        if (++throttleSameCount >= 5) {
            throttlePercent = 0.0f;
            return;
        }
    } else {
        throttleSameCount = 0;
    }

    throttleRawADC = newThrottle;
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