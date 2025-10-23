#pragma once
#include <Arduino.h>
#include "config.h"
#include "data_structures.h"

//=============================================================================
// 192S BMS MANAGER - Configured for 104S LiPo Pack
// CAN Protocol: RX=0xF4, TX=0xF5, 500kbps, Extended Format
//=============================================================================

//=============================================================================
// EXTENDED BMS DATA STRUCTURE
//=============================================================================

struct BMSDataExtended {
    // Basic pack data
    uint16_t packVoltage;                           // Pack voltage (0.1V resolution)
    int16_t packCurrent;                            // Pack current (0.01A resolution, signed)
    uint16_t packPower;                             // Pack power (W)

    // Cell voltages (104 cells, 1mV resolution)
    uint16_t cellVoltage[Battery::NUM_CELLS];      // mV
    float minCellVoltage;                           // V
    float maxCellVoltage;                           // V
    uint8_t minCellIndex;                           // Cell index with min voltage
    uint8_t maxCellIndex;                           // Cell index with max voltage
    float cellVoltageDelta;                         // Max - Min cell voltage

    // Temperature (16 modules)
    int16_t moduleTemp[Battery::NUM_TEMP_SENSORS]; // 0.1°C resolution, signed
    float hostTemp;                                 // BMS controller temp (°C)
    float maxTemp;                                  // Highest module temp
    float minTemp;                                  // Lowest module temp
    bool tempNegative;                              // True if any temp below 0°C

    // Capacity and SOC
    uint16_t packCapacityAh;                        // Total pack capacity (Ah)
    uint16_t usedCapacityAh;                        // Used capacity (Ah)
    uint16_t remainingCapacityAh;                   // Remaining capacity (Ah)
    uint16_t chargeCapacityAh;                      // Charge capacity (Ah)
    uint8_t capacityPercent;                        // SOC percentage

    // Protection and Limits
    uint16_t chargeProtectionVoltage;               // Overcharge protection (0.1V)
    uint16_t dischargeProtectionVoltage;            // Overdischarge protection (0.1V)
    uint16_t chargeRecoveryVoltage;                 // Charge recovery voltage
    uint16_t dischargeRecoveryVoltage;              // Discharge recovery voltage
    uint16_t protectionCurrent;                     // Overcurrent protection (A)
    uint16_t protectionTemp;                        // Over-temp protection (°C)

    // Status flags (16-bit from message group 6)
    uint16_t statusFlags;
    bool overTemp;                                  // Over temperature
    bool overCharge;                                // Overcharge detected
    bool cellError;                                 // Cell string error
    bool overCurrent;                               // Overcurrent detected
    bool overDischarge;                             // Over-discharge detected
    bool balancing;                                 // Balancing active
    bool charging;                                  // Current polarity (true=charge)
    bool channelEnabled;                            // Main channel state

    // Balancing
    uint16_t balanceStartVoltage;                   // Balance start voltage (mV)
    uint16_t balanceRefVoltage;                     // Balance reference voltage (mV)

    // MOS Status
    bool chargeMOSEnabled;                          // Charge MOS state
    bool dischargeMOSEnabled;                       // Discharge MOS state

    // Diagnostic
    uint16_t numCellsConfigured;                    // Number of cells configured
    uint8_t protectionHistory;                      // Historical protection log
    unsigned long lastUpdateTime;                   // Timestamp of last update
    bool dataValid;                                 // True if data is current
};

//=============================================================================
// BMS MANAGER CLASS
//=============================================================================

class BMSManager {
public:
    BMSManager();

    /**
     * @brief Initialize BMS manager
     */
    void begin();

    /**
     * @brief Process incoming BMS CAN message
     * @param id CAN message ID
     * @param buf Data buffer (8 bytes)
     * @param len Data length
     */
    void processMessage(uint32_t id, const uint8_t* buf, uint8_t len);

    /**
     * @brief Send command to BMS
     * @param command Command header (0x01-0x1C)
     * @param value Command value (16-bit)
     */
    void sendCommand(uint8_t command, uint16_t value);

    /**
     * @brief Update task - check timeouts, update shared data
     * Call this periodically (e.g., every 100ms)
     */
    void update();

    /**
     * @brief Get extended BMS data
     */
    BMSDataExtended getData() const { return data; }

    /**
     * @brief Check if BMS is alive
     */
    bool isAlive() const;

    /**
     * @brief Check if BMS is in error state
     */
    bool hasError() const;

    /**
     * @brief Check if charging is allowed
     */
    bool canCharge() const;

    /**
     * @brief Check if discharging is allowed
     */
    bool canDischarge() const;

    /**
     * @brief Get charging limits for charger control
     */
    void getChargingLimits(float& maxVoltage, float& maxCurrent, float& maxTemp) const;

    /**
     * @brief Enable/disable BMS channel
     */
    void setChannelEnabled(bool enable);

    /**
     * @brief Enable/disable balancing
     */
    void setBalancingEnabled(bool enable);

    /**
     * @brief Request BMS to start CAN communication
     */
    void requestConnection();

private:
    BMSDataExtended data;
    SemaphoreHandle_t dataMutex;

    // Message tracking (which message groups we've received)
    uint8_t receivedGroups[11];  // Track groups 1-6, 71-73, 80-82 (summary data)

    // Message processing helpers
    void processGroup1(const uint8_t* buf);  // Discharge protection
    void processGroup2(const uint8_t* buf);  // Charge protection & num cells
    void processGroup3(const uint8_t* buf);  // Voltage, current, power
    void processGroup4(const uint8_t* buf);  // Capacity and SOC
    void processGroup5(const uint8_t* buf);  // Recovery voltages
    void processGroup6(const uint8_t* buf);  // Status and temperature
    void processCellVoltages(uint8_t group, const uint8_t* buf);  // Groups 7-41 (104 cells)
    void processModuleTemps(uint8_t group, const uint8_t* buf);   // Groups 71-79 (16 modules)
    void processGroup80(const uint8_t* buf);  // Additional diagnostics
    void processGroup81(const uint8_t* buf);  // Min/max voltages, MOS status
    void processGroup82(const uint8_t* buf);  // Accumulated capacity

    // Helper functions
    void updateMinMaxCells();
    void updateMinMaxTemps();
    void updateSharedData();
    void checkTimeout();

    // Parse helpers
    uint16_t parseU16(const uint8_t* buf, uint8_t offset) const;
    int16_t parseI16(const uint8_t* buf, uint8_t offset) const;
    bool parseBit(uint16_t value, uint8_t bit) const;
};
