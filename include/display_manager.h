#pragma once
#include <Arduino.h>
#include "config.h"

//=============================================================================
// DISPLAY MANAGER
// Handles: Nextion screen updates
//=============================================================================

class DisplayManager {
public:
    DisplayManager();
    
    /**
     * @brief Initialize display (Serial2)
     */
    bool begin();
    
    /**
     * @brief Update display (call periodically)
     */
    void update();
    
    // -------------------------------------------------------------------------
    // DATA SETTERS
    // -------------------------------------------------------------------------
    void setSpeed(int speedKmh) { speed = speedKmh; }
    void setPower(int powerKw) { power = powerKw; }
    void setSOC(uint8_t socPercent) { soc = socPercent; }
    void setVoltage(uint16_t voltageV) { voltage = voltageV; }
    void setGear(GearState gear) { currentGear = gear; }
    
    void setTempMotor(float temp) { tempMotor = temp; }
    void setTempInverter(float temp) { tempInverter = temp; }
    void setTempBattery(float temp) { tempBattery = temp; }
    void setMinCellVoltage(float volts) { minCellVoltage = volts; }
    
    // -------------------------------------------------------------------------
    // WARNINGS & ERRORS
    // -------------------------------------------------------------------------
    void showWarning(const char* message);
    void clearWarning();
    
private:
    // Display data
    int speed;
    int power;
    uint8_t soc;
    uint16_t voltage;
    GearState currentGear;
    
    float tempMotor;
    float tempInverter;
    float tempBattery;
    float minCellVoltage;
    
    // Timing
    unsigned long lastUpdate;
    
    // -------------------------------------------------------------------------
    // PRIVATE UPDATE METHODS (FEHLTEN!)
    // -------------------------------------------------------------------------
    void updateSpeed();
    void updatePower();
    void updateSOC();
    void updateVoltage();
    void updateGear();
    void updateTemperatures();
    void updateMinCellVoltage();
    
    // -------------------------------------------------------------------------
    // Helper methods
    // -------------------------------------------------------------------------
    void sendCommand(const String& cmd);
    void updateField(const String& labelId, const String& valueId, 
                     const String& value, bool critical);
    
    // Check critical values
    bool isCriticalTemp(float temp, float threshold);
    bool isCriticalVoltage(float voltage, float threshold);
    bool isCriticalSOC(uint8_t soc, uint8_t threshold);
};