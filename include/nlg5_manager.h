#pragma once
#include <Arduino.h>
#include "config.h"
#include "data_structures.h"

//=============================================================================
// NLG5 CHARGER MANAGER - Manual Control (No Control Pilot)
// CAN Protocol: From DBC file, 500kbps, Standard Format
//=============================================================================

//=============================================================================
// EXTENDED NLG5 DATA STRUCTURE
//=============================================================================

struct NLG5DataExtended {
    // Actuals (from NLG5_ACT_I message - ID 1553)
    float mainsVoltageActual;                       // AC mains voltage (V)
    float mainsCurrentActual;                       // AC mains current (A)
    float batteryVoltageActual;                     // DC output voltage (V)
    float batteryCurrentActual;                     // DC output current (A)

    // Actuals II (from NLG5_ACT_II message - ID 1554)
    float auxBatteryVoltage;                        // 12V aux battery (V)
    float mainsCurrent_CP;                          // Mains current limit from CP (A)
    float mainsCurrent_PI;                          // Mains current limit from PI (A)
    float boosterCurrent;                           // Booster output current (A)
    int16_t ahCounter;                              // External Ah counter (Ah)

    // Temperatures (from NLG5_TEMP message - ID 1555)
    float tempPowerStage;                           // Power stage temp (°C)
    float tempExt1;                                 // External temp 1 (°C)
    float tempExt2;                                 // External temp 2 (°C)
    float tempExt3;                                 // External temp 3 (°C)

    // Status flags (from NLG5_ST message - ID 1552)
    uint32_t statusBits;                            // Full 32-bit status
    bool cpDetected;                                // Control pilot detected
    bool bypassDetected;                            // Bypass detected
    bool errorActive;                               // Error flag
    bool europeanMains;                             // 230V 50Hz detected
    bool usMains1;                                  // 120V 60Hz detected
    bool usMains2;                                  // 240V 60Hz detected
    bool fanActive;                                 // Cooling fan running
    bool warningActive;                             // Warning (power limited)
    bool aux12VCharging;                            // 12V aux charging active
    bool hardwareEnabled;                           // Hardware enable signal

    // Power limiting flags
    bool limitedBy_CP;                              // Limited by control pilot
    bool limitedBy_MainsCurrent;                    // Limited by mains current
    bool limitedBy_MaxCurrent;                      // Limited by max capability
    bool limitedBy_BatteryCurrent;                  // Limited by battery current
    bool limitedBy_BatteryVoltage;                  // Limited by battery voltage
    bool limitedBy_MaxPower;                        // Limited by max power
    bool limitedBy_Temperature;                     // Limited by temperature
    bool limitedBy_PowerIndicator;                  // Limited by PI signal

    // Temperature limiting flags
    bool limitedBy_TempBattery;                     // Battery temp
    bool limitedBy_TempCapacitor;                   // Capacitor temp
    bool limitedBy_TempDiode;                       // Diode temp
    bool limitedBy_TempTransformer;                 // Transformer temp
    bool limitedBy_TempPower;                       // Power stage temp

    // Error flags (from NLG5_ERR message - ID 1556)
    uint64_t errorBits;                             // Full 64-bit error word
    bool error_MainsFuse;                           // Mains fuse defective
    bool error_OutputFuse;                          // Output fuse defective
    bool error_MainsOvervoltage1;                   // Mains overvoltage 1
    bool error_MainsOvervoltage2;                   // Mains overvoltage 2
    bool error_BatteryOvervoltage;                  // Battery overvoltage
    bool error_BatteryPolarity;                     // Wrong battery polarity
    bool error_ShortCircuit;                        // Power stage short circuit
    bool error_CANTimeout;                          // CAN control timeout (>300ms)
    bool error_CANOff;                              // CAN off (TX buffer >255)
    bool error_TempSensor;                          // Temperature sensor error
    bool error_CRCChecksum;                         // Checksum failure

    // Warning flags (from NLG5_ERR message)
    bool warning_LowMainsVoltage;                   // Mains voltage too low
    bool warning_LowBatteryVoltage;                 // Battery voltage too low
    bool warning_HighTemperature;                   // Internal overtemperature
    bool warning_ControlOutOfRange;                 // Control value out of range

    // Control echo (what charger is receiving)
    float commandedVoltage;                         // Voltage command being executed
    float commandedCurrent;                         // Current command being executed
    float commandedMaxMainsCurrent;                 // Mains current limit being used

    // Diagnostic
    unsigned long lastUpdateTime;                   // Timestamp of last message
    bool dataValid;                                 // True if data is current
};

//=============================================================================
// NLG5 CHARGING STATE MACHINE
//=============================================================================

enum class NLG5State : uint8_t {
    IDLE,           // Not charging, disabled
    STARTING,       // Enabling charger, waiting for ready
    BULK,           // Constant current bulk charging
    ABSORPTION,     // Constant voltage absorption
    COMPLETED,      // Charging complete
    ERROR,          // Error state
    DISABLED_STATE  // Manually disabled (renamed to avoid ESP32 HAL macro conflict)
};

//=============================================================================
// NLG5 MANAGER CLASS
//=============================================================================

class NLG5Manager {
public:
    NLG5Manager();

    /**
     * @brief Initialize NLG5 manager
     */
    void begin();

    /**
     * @brief Process incoming NLG5 CAN message
     * @param id CAN message ID
     * @param buf Data buffer (8 bytes)
     * @param len Data length
     */
    void processMessage(uint32_t id, const uint8_t* buf, uint8_t len);

    /**
     * @brief Update task - send control messages, manage state machine
     * Call this periodically (e.g., every 100ms)
     */
    void update();

    /**
     * @brief Start charging with given limits
     * @param targetVoltage Target pack voltage (V)
     * @param maxCurrent Maximum charge current (A)
     * @param maxMainsCurrent Maximum mains current (A)
     */
    void startCharging(float targetVoltage, float maxCurrent, float maxMainsCurrent = Charger::MAINS_CURRENT_MAX);

    /**
     * @brief Stop charging
     */
    void stopCharging();

    /**
     * @brief Clear charger errors (cycle error clear bit 0-1-0)
     */
    void clearErrors();

    /**
     * @brief Get extended NLG5 data
     */
    NLG5DataExtended getData() const { return data; }

    /**
     * @brief Get current charging state
     */
    NLG5State getState() const { return state; }

    /**
     * @brief Check if charger is alive (receiving CAN messages)
     */
    bool isAlive() const;

    /**
     * @brief Check if charger has errors
     */
    bool hasError() const;

    /**
     * @brief Check if charging is active
     */
    bool isCharging() const;

    /**
     * @brief Get actual charging power
     */
    float getChargingPower() const;

private:
    NLG5DataExtended data;
    NLG5State state;
    SemaphoreHandle_t dataMutex;

    // Control parameters
    float targetVoltage;
    float targetCurrent;
    float maxMainsCurrent;
    bool enableCharger;
    bool errorClearCycle;  // For cycling error clear bit

    // State machine timing
    unsigned long stateStartTime;
    unsigned long lastControlSentTime;

    // Message processing
    void processStatus(const uint8_t* buf);         // NLG5_ST (0x610)
    void processActualsI(const uint8_t* buf);       // NLG5_ACT_I (0x611)
    void processActualsII(const uint8_t* buf);      // NLG5_ACT_II (0x612)
    void processTemperatures(const uint8_t* buf);   // NLG5_TEMP (0x613)
    void processErrors(const uint8_t* buf);         // NLG5_ERR (0x614)

    // Control message sending
    void sendControl();                             // Send NLG5_CTL message

    // State machine
    void updateStateMachine();
    void transitionToState(NLG5State newState);

    // Helper functions
    void updateSharedData();
    void checkTimeout();

    // Parse helpers
    uint16_t parseU16(const uint8_t* buf, uint8_t offset) const;
    int16_t parseI16(const uint8_t* buf, uint8_t offset) const;
    bool parseBit(uint32_t value, uint8_t bit) const;
    bool parseBit64(uint64_t value, uint8_t bit) const;
};
