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
    bool isStartPressed() const { return startPressed; }
    bool isStopPressed() const { return stopPressed; }
    bool isResetPressed() const { return resetPressed; }
    bool isILClosed() const { return ilClosed; }  // Interlock closed = true
    
    // -------------------------------------------------------------------------
    // MCP23017 INPUTS
    // -------------------------------------------------------------------------
    bool isBrakePressed() const { return brakePressed; }
    bool isDirectionTogglePressed();  // Returns true on RISING edge only
    
    // -------------------------------------------------------------------------
    // ANALOG INPUTS (0-100%)
    // -------------------------------------------------------------------------
    float getThrottlePercent() const { return throttlePercent; }
    float getRegenPercent() const { return regenPercent; }
    
    // -------------------------------------------------------------------------
    // CHARGER WAKEUP
    // -------------------------------------------------------------------------
    bool isChargerConnected() const { return chargerWakeup; }
    
private:
    // Hardware interfaces
    Adafruit_MCP23X17 mcp;
    ADS1115 ads;
    
    // Button states (direct GPIO)
    bool startPressed;
    bool stopPressed;
    bool resetPressed;
    bool ilClosed;
    bool chargerWakeup;
    
    // MCP23017 states
    bool brakePressed;
    bool directionToggleCurrent;
    bool directionToggleLast;
    
    // Analog values
    float throttlePercent;
    float regenPercent;
    
    // Debounce timing
    unsigned long lastDebounceTime;
    unsigned long lastDirectionToggleTime;
    
    // Helper methods
    void updateButtons();
    void updateMCP();
    void updateAnalog();
    float mapADCToPercent(int16_t adcValue, int16_t minVal, int16_t maxVal);
};