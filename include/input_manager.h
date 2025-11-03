#pragma once
#include <Arduino.h>
#include <Adafruit_MCP23X17.h>
#include <ADS1X15.h>
#include "config.h"

//=============================================================================
// INPUT MANAGER
// Handles: Buttons, MCP23017, ADS1115 (Throttle/Regen Pots)
//=============================================================================

class InputManager {
public:
    InputManager();
    
    /**
     * @brief Initialize all input devices
     * @return true if initialization successful
     */
    bool begin();
    
    /**
     * @brief Update all inputs (call in loop)
     */
    void update();
    
    // -------------------------------------------------------------------------
    // BUTTON STATES (Debounced)
    // -------------------------------------------------------------------------
    bool isStartStopPressed() const { return startStopPressed; }
    bool isILClosed() const { return ilClosed; }  // Interlock closed = true

    // -------------------------------------------------------------------------
    // MCP23017 INPUTS
    // -------------------------------------------------------------------------
    bool isBrakePressed() const { return brakePressed; }
    bool isDirectionTogglePressed();        // Returns true on RISING edge only
    bool isDirectionToggleHeld() const { return directionToggleCurrent; }  // Raw button state for hold detection

    // -------------------------------------------------------------------------
    // SAFETY CRITICAL - MCP23017 Port A (Interrupt Driven)
    // -------------------------------------------------------------------------
    bool isChargeAllowed() const { return chargeAllowed; }      // true = charge allowed, false = no charge
    bool isDischargeAllowed() const { return dischargeAllowed; } // true = discharge allowed, false = no discharge

    // -------------------------------------------------------------------------
    // ANALOG INPUTS (0-100%)
    // -------------------------------------------------------------------------
    float getThrottlePercent() const { return throttlePercent; }
    float getRegenPercent() const { return regenPercent; }

    // -------------------------------------------------------------------------
    // CALIBRATION
    // -------------------------------------------------------------------------
    int16_t getThrottleRawADC() const;   // Get current raw ADC value
    int16_t getRegenRawADC() const;      // Get current raw ADC value
    void setCalibration(int16_t throttleMin, int16_t throttleMax,
                       int16_t regenMin, int16_t regenMax);  // Set calibration values
    
    // -------------------------------------------------------------------------
    // CHARGER WAKEUP
    // -------------------------------------------------------------------------
    bool isChargerConnected() const { return chargerWakeup; }
    
private:
    // Hardware interfaces
    Adafruit_MCP23X17 mcp;
    ADS1115 ads;

    // Button states (direct GPIO)
    bool startStopPressed;
    bool ilClosed;
    bool chargerWakeup;

    // MCP23017 Port B states
    bool brakePressed;
    bool directionToggleCurrent;
    bool directionToggleLast;

    // MCP23017 Port A states (safety critical)
    bool chargeAllowed;           // true = charge allowed (LOW signal)
    bool dischargeAllowed;        // true = discharge allowed (LOW signal)
    bool chargeAllowedLast;       // For edge detection
    bool dischargeAllowedLast;    // For edge detection

    // Analog values
    float throttlePercent;
    float regenPercent;
    int16_t throttleRawADC;
    int16_t regenRawADC;

    // Calibration values (loaded from config)
    int16_t throttleMinADC;
    int16_t throttleMaxADC;
    int16_t regenMinADC;
    int16_t regenMaxADC;

    // Debounce timing
    unsigned long lastDebounceTime;
    unsigned long lastDirectionToggleTime;

    // Helper methods
    void updateButtons();
    void updateMCPPortB();         // Update Port B (brake, direction)
    void updateMCPPortA();         // Update Port A (charge/discharge allow - SAFETY CRITICAL)
    void updateAnalog();
    float mapADCToPercent(int16_t adcValue, int16_t minVal, int16_t maxVal);
    void handleSafetyInterrupt();  // Called by ISR for Port A changes
};