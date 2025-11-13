#pragma once
#include <Arduino.h>
#include <SPI.h>
#include <mcp2515_can.h>
#include <driver/twai.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "config.h"
#include "data_structures.h"

// Forward declarations
class BMSManager;
class NLG5Manager;

//=============================================================================
// CAN MANAGER - Dual CAN Bus FreeRTOS Version
// CAN1 (500kbps, MCP2515): DMC Motor Controller, NLG Charger
// CAN2 (250kbps, ESP32 TWAI): BMS Battery Management System
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
     * @brief Set BMS manager for routing BMS messages
     * @param bmsMgr Pointer to BMSManager
     */
    void setBMSManager(BMSManager* bmsMgr);

    /**
     * @brief Set NLG5 manager for routing NLG5 messages
     * @param nlg5Mgr Pointer to NLG5Manager
     */
    void setNLG5Manager(NLG5Manager* nlg5Mgr);

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
     * @param enableCharger Enable charger power stage (bit 0)
     * @param clearErrors Cycle error clear bit (bit 1)
     */
    void sendNLGControl(uint8_t stateDemand, bool enableCharger, bool clearErrors = false);

    // -------------------------------------------------------------------------
    // DATA ACCESS (Thread-safe via shared data structures)
    // -------------------------------------------------------------------------
    // Data is now accessed via sharedBMSData, sharedDMCData, sharedNLGData
    // in data_structures.h

    // -------------------------------------------------------------------------
    // BMS CONTROL (Thread-safe)
    // -------------------------------------------------------------------------
    /**
     * @brief Send message on CAN2 (BMS bus)
     * @param id CAN message ID
     * @param len Data length (0-8)
     * @param data Data buffer
     * @return true if sent successfully
     */
    bool sendCAN2Message(uint32_t id, uint8_t len, const uint8_t* data);

    // -------------------------------------------------------------------------
    // STATUS QUERIES
    // -------------------------------------------------------------------------
    bool isBMSAlive() const;
    bool isDMCReady() const;
    bool isCharging() const;

    /**
     * @brief Populate system error status with DMC errors
     * @param status SystemErrorStatus structure to populate
     */
    void populateDMCErrorStatus(SystemErrorStatus& status);

    /**
     * @brief CAN interrupt handler (static, called from ISR)
     */
    static void IRAM_ATTR canISR();

    /**
     * @brief Set the instance for ISR callback
     */
    static void setInstance(CANManager* instance) { _instance = instance; }

    // Task handle for notification from ISR (public so main.cpp can set it)
    TaskHandle_t canRxTaskHandle;

private:
    // Hardware - Dual CAN buses
    mcp2515_can CAN1;  // DMC + NLG (500kbps, MCP2515 via SPI)
    // CAN2 uses ESP32-S3 internal TWAI controller (no object needed)
    SPIClass* spi;
    bool twaiInitialized;

    // Manager pointers for routing messages
    BMSManager* bmsManager;
    NLG5Manager* nlg5Manager;

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

    // Error clear tracking
    bool errorClearActive;              // Is error clear bit currently set?
    unsigned long errorClearSetTime;    // When was error clear bit set to 1?

    // Helper functions
    void resetBuffers();
    void readCAN1Message();  // Called from task to read CAN1 messages
    void pollCAN2();         // Polled BMS messages (no interrupt)
    bool sendCAN1Message(uint32_t id, uint8_t len, const uint8_t* data);
};