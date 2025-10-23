#include "display_manager.h"

DisplayManager::DisplayManager()
    : speed(0)
    , power(0)
    , soc(0)
    , voltage(0)
    , currentGear(GearState::NEUTRAL)
    , tempMotor(0.0f)
    , tempInverter(0.0f)
    , tempBattery(0.0f)
    , minCellVoltage(0.0f)
    , lastUpdate(0)
{
}

bool DisplayManager::begin() {
    DEBUG_PRINTLN("DisplayManager: Initializing...");
    DEBUG_PRINTF("  TX Pin: %d, RX Pin: %d\n", Pins::NEXTION_TX, Pins::NEXTION_RX);

    // Initialize Serial2 for Nextion (19=TX, 20=RX)
    Serial2.begin(9600, SERIAL_8N1, Pins::NEXTION_RX, Pins::NEXTION_TX);

    // Wait for display to boot
    DEBUG_PRINTLN("  Waiting for display boot...");
    delay(500);

    // Reset display and go to page 0
    DEBUG_PRINTLN("  Sending reset command...");
    sendCommand("reset");  // Fixed typo: was "rest"
    delay(500);

    DEBUG_PRINTLN("  Setting page 0...");
    sendCommand("page 0");
    delay(100);

    DEBUG_PRINTLN("DisplayManager: Initialized");
    return true;
}

void DisplayManager::update() {
    unsigned long currentTime = millis();

    // Update at fixed rate (20 Hz)
    if (currentTime - lastUpdate < Timing::DISPLAY_UPDATE) {
        return;
    }
    lastUpdate = currentTime;

    // Debug: Log first few updates
    static uint8_t debugCount = 0;
    if (debugCount < 3) {
        DEBUG_PRINTF("DisplayManager: Update #%d - Speed=%d, Power=%d, SOC=%d%%\n",
                     debugCount, speed, power, soc);
        debugCount++;
    }

    // Update all display fields
    updateSpeed();
    updatePower();
    updateSOC();
    updateVoltage();
    updateGear();
    updateTemperatures();
    updateMinCellVoltage();
}

//=============================================================================
// DISPLAY UPDATE METHODS
//=============================================================================

void DisplayManager::updateSpeed() {
    // Update speed value
    sendCommand("speed.txt=\"" + String(speed) + "\"");
}

void DisplayManager::updatePower() {
    // Update power label and value
    sendCommand("t7.txt=\"PWR\"");
    sendCommand("pwr.txt=\"" + String(power) + "kW\"");
}

void DisplayManager::updateSOC() {
    // Check if SOC is critical
    bool critical = isCriticalSOC(soc, Battery::MIN_SOC);
    
    // Update SOC field with warning colors if needed
    updateField("t4", "soc", String(soc) + "%", critical);
}

void DisplayManager::updateVoltage() {
    // Check if voltage is critical
    bool critical = isCriticalVoltage(voltage, Battery::MIN_VOLTAGE);
    
    // Update voltage field
    updateField("t0", "voltage", String(voltage) + "V", critical);
}

void DisplayManager::updateGear() {
    // Convert gear enum to character
    char gearChar;
    switch (currentGear) {
        case GearState::DRIVE:
            gearChar = 'D';
            break;
        case GearState::REVERSE:
            gearChar = 'R';
            break;
        case GearState::NEUTRAL:
        default:
            gearChar = 'N';
            break;
    }
    
    // Update gear display
    sendCommand("gear.txt=\"" + String(gearChar) + "\"");
}

void DisplayManager::updateTemperatures() {
    // Motor temperature
    bool motorCritical = isCriticalTemp(tempMotor, Safety::MAX_MOTOR_TEMP);
    // Using \xB0 for degree symbol (ASCII 176/0xB0), separate strings to avoid hex escape issue
    updateField("t2", "tempMot", String((int)tempMotor) + String("\xB0") + "C", motorCritical);

    // Inverter temperature (Contactor temp on display)
    bool inverterCritical = isCriticalTemp(tempInverter, Safety::MAX_INVERTER_TEMP);
    updateField("t1", "tempCont", String((int)tempInverter) + String("\xB0") + "C", inverterCritical);

    // Battery temperature
    bool batteryCritical = isCriticalTemp(tempBattery, Safety::MAX_BATTERY_TEMP);
    updateField("t5", "tempBat", String((int)tempBattery) + String("\xB0") + "C", batteryCritical);
}

void DisplayManager::updateMinCellVoltage() {
    // Check if min cell voltage is critical
    bool critical = isCriticalVoltage(minCellVoltage, Safety::CRITICAL_CELL_VOLTAGE);
    
    // Update min cell voltage field
    updateField("t3", "ubat", String(minCellVoltage, 2) + "V", critical);
}

//=============================================================================
// WARNING SYSTEM
//=============================================================================

void DisplayManager::showWarning(const char* message) {
    // Display warning message (implementation depends on your Nextion layout)
    // For now, just log it
    DEBUG_PRINTF("Display Warning: %s\n", message);
    
    // You can add a warning popup or message area if your Nextion has one
    // Example: sendCommand("page 1");  // Go to warning page
}

void DisplayManager::clearWarning() {
    // Clear any displayed warnings
    // sendCommand("page 0");  // Return to main page
}

//=============================================================================
// HELPER METHODS
//=============================================================================

void DisplayManager::sendCommand(const String& cmd) {
    Serial2.print(cmd);
    Serial2.write(0xFF);
    Serial2.write(0xFF);
    Serial2.write(0xFF);
}

void DisplayManager::updateField(const String& labelId, const String& valueId, 
                                 const String& value, bool critical) {
    // Update the value text
    sendCommand(valueId + ".txt=\"" + value + "\"");
    
    // Set colors based on critical status
    if (critical) {
        // Yellow background for critical values
        sendCommand(labelId + ".bco=65504");  // Label yellow
        sendCommand(valueId + ".bco=65504");  // Value yellow
    } else {
        // White background for normal values
        sendCommand(labelId + ".bco=65535");  // Label white
        sendCommand(valueId + ".bco=65535");  // Value white
    }
}

//=============================================================================
// CRITICAL VALUE CHECKS
//=============================================================================

bool DisplayManager::isCriticalTemp(float temp, float threshold) {
    return (temp >= threshold);
}

bool DisplayManager::isCriticalVoltage(float voltage, float threshold) {
    return (voltage <= threshold);
}

bool DisplayManager::isCriticalSOC(uint8_t soc, uint8_t threshold) {
    return (soc <= threshold);
}