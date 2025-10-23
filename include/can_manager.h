#pragma once
#include <Arduino.h>
#include <SPI.h>
#include <mcp2515_can.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "config.h"
#include "data_structures.h"

//=============================================================================
// CAN MANAGER - FreeRTOS Version
// Handles: BMS, DMC, NLG communication with interrupt-driven RX
//=============================================================================

// Note: BMSData, DMCData, NLGData are now defined in data_structures.h

//=============================================================================
// CAN MANAGER CLASS
//=============================================================================

class CANManager {
public:
    CANManager();

    /**
     * @brief Initialize CAN bus and interrupt
     * @return true if initialization successful
     */
    bool begin();

    /**
     * @brief Process incoming messages from queue (call from CAN RX task)
     */
    void processRxQueue();

    /**
     * @brief Send periodic CAN messages (call from CAN TX task)
     */
    void sendPeriodicMessages();

    // -------------------------------------------------------------------------
    // DMC CONTROL (Thread-safe)
    // -------------------------------------------------------------------------
    /**
     * @brief Send DMC control message
     * @param torqueNm Torque demand in Nm
     * @param enable Enable motor controller
     */
    void sendDMCControl(int16_t torqueNm, bool enable);

    /**
     * @brief Send DMC limits message
     */
    void sendDMCLimits();

    // -------------------------------------------------------------------------
    // NLG CONTROL (Charger) - Thread-safe
    // -------------------------------------------------------------------------
    /**
     * @brief Send NLG control message
     * @param stateDemand Requested charger state
     */
    void sendNLGControl(uint8_t stateDemand);

    // -------------------------------------------------------------------------
    // DATA ACCESS (Thread-safe via shared data structures)
    // -------------------------------------------------------------------------
    // Data is now accessed via sharedBMSData, sharedDMCData, sharedNLGData
    // in data_structures.h

    // -------------------------------------------------------------------------
    // STATUS QUERIES
    // -------------------------------------------------------------------------
    bool isBMSAlive() const;
    bool isDMCReady() const;
    bool isCharging() const;

    /**
     * @brief CAN interrupt handler (static, called from ISR)
     */
    static void IRAM_ATTR canISR();

    /**
     * @brief Set the instance for ISR callback
     */
    static void setInstance(CANManager* instance) { _instance = instance; }

private:
    // Hardware
    mcp2515_can CAN;
    SPIClass* spi;

    // Local copies of data (updated from shared data)
    BMSData bmsData;
    DMCData dmcData;
    NLGData nlgData;

    // Timing
    unsigned long lastBMSTime;
    TickType_t lastSlowCycleTime;

    // Thread synchronization
    SemaphoreHandle_t spiMutex;  // Protect SPI bus access

    // Static instance for ISR
    static CANManager* _instance;

    // Message processing
    void processCANMessage(const CANMessage& msg);
    void processBMSMessage(uint32_t id, const uint8_t* buf, uint8_t len);
    void processDMCMessage(uint32_t id, const uint8_t* buf, uint8_t len);
    void processNLGMessage(uint32_t id, const uint8_t* buf, uint8_t len);

    // Message buffers
    uint8_t dmcControlBuffer[8];
    uint8_t dmcLimitsBuffer[8];
    uint8_t nlgControlBuffer[8];

    // Helper functions
    void resetBuffers();
    void readCANMessage();  // Called from ISR to push message to queue
    bool sendCANMessage(uint32_t id, uint8_t len, const uint8_t* data);
};