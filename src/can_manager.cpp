#include "can_manager.h"

// Static instance for ISR
CANManager* CANManager::_instance = nullptr;

CANManager::CANManager()
    : CAN(Pins::CAN_CS)
    , spi(nullptr)
    , lastBMSTime(0)
    , lastSlowCycleTime(0)
    , spiMutex(nullptr)
{
    resetBuffers();

    // Initialize data structures
    memset(&bmsData, 0, sizeof(BMSData));
    memset(&dmcData, 0, sizeof(DMCData));
    memset(&nlgData, 0, sizeof(NLGData));
}

bool CANManager::begin() {
    DEBUG_PRINTLN("CANManager: Initializing...");

    // Create SPI mutex
    spiMutex = xSemaphoreCreateMutex();
    if (spiMutex == NULL) {
        DEBUG_PRINTLN("ERROR: Failed to create SPI mutex!");
        return false;
    }

    // Initialize SPI
    spi = new SPIClass(HSPI);
    spi->begin(Pins::SPI_SCK, Pins::SPI_MISO, Pins::SPI_MOSI, Pins::CAN_CS);
    CAN.setSPI(spi);

    // Initialize MCP2515 CAN controller
    uint8_t retries = 0;
    const uint8_t MAX_RETRIES = 5;

    while (CAN_OK != CAN.begin(CAN_500KBPS)) {
        DEBUG_PRINTLN("ERROR: CAN init failed, retrying...");
        delay(100);

        if (++retries >= MAX_RETRIES) {
            DEBUG_PRINTLN("ERROR: CAN init failed after max retries!");
            return false;
        }
    }

    DEBUG_PRINTLN("CAN initialized successfully (500 kbps)");

    // Set instance for ISR
    setInstance(this);

    // Attach interrupt
    pinMode(Pins::CAN_INT, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(Pins::CAN_INT), canISR, FALLING);

    DEBUG_PRINTLN("CAN interrupt attached");

    return true;
}

void IRAM_ATTR CANManager::canISR() {
    if (_instance != nullptr) {
        _instance->readCANMessage();
    }
}

void CANManager::readCANMessage() {
    // This runs in ISR context - must be fast!
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Check if message available
    if (CAN_MSGAVAIL == CAN.checkReceive()) {
        CANMessage msg;
        msg.timestamp = millis();

        if (CAN_OK == CAN.readMsgBuf(&msg.len, msg.data)) {
            msg.id = CAN.getCanId();

            // Push to queue (don't block in ISR)
            xQueueSendFromISR(canRxQueue, &msg, &xHigherPriorityTaskWoken);
        }
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void CANManager::processRxQueue() {
    CANMessage msg;

    // Process all messages in queue
    while (xQueueReceive(canRxQueue, &msg, 0) == pdTRUE) {
        processCANMessage(msg);
    }
}

void CANManager::processCANMessage(const CANMessage& msg) {
#if DEBUG_CAN_MESSAGES
    DEBUG_PRINTF("CAN RX: 0x%03X [%d] ", msg.id, msg.len);
    for (int i = 0; i < msg.len; i++) {
        DEBUG_PRINTF("%02X ", msg.data[i]);
    }
    DEBUG_PRINTLN();
#endif

    // Route message to appropriate handler
    if (msg.id == CAN_ID::BMS_TX) {
        processBMSMessage(msg.id, msg.data, msg.len);
    }
    else if (msg.id == CAN_ID::DMC_STATUS ||
             msg.id == CAN_ID::DMC_POWER ||
             msg.id == CAN_ID::DMC_TEMPERATURE) {
        processDMCMessage(msg.id, msg.data, msg.len);
    }
    else if (msg.id == CAN_ID::NLG5_ST ||
             msg.id == CAN_ID::NLG5_ACT_I ||
             msg.id == CAN_ID::NLG5_ACT_II ||
             msg.id == CAN_ID::NLG5_TEMP ||
             msg.id == CAN_ID::NLG5_ERR) {
        processNLGMessage(msg.id, msg.data, msg.len);
    }
}

void CANManager::sendPeriodicMessages() {
    TickType_t currentTime = xTaskGetTickCount();

    // Get runtime config for timing
    RuntimeConfigData config = sharedRuntimeConfig.get();

    // Send slow cycle messages
    if ((currentTime - lastSlowCycleTime) >= pdMS_TO_TICKS(config.canSlowCycle)) {
        lastSlowCycleTime = currentTime;
        sendDMCLimits();
    }
}

//=============================================================================
// BMS MESSAGE PROCESSING
//=============================================================================

void CANManager::processBMSMessage(uint32_t id, const uint8_t* buf, uint8_t len) {
    if (id != CAN_ID::BMS_TX || len < 8) {
        return;
    }

    // Parse BMS status message (0x010)
    bmsData.soc = buf[0] / 2;
    bmsData.voltage = ((uint16_t)buf[1] << 8 | buf[2]) / 10;
    bmsData.current = (int16_t)((uint16_t)buf[3] << 8 | buf[4]);
    bmsData.maxDischarge = (uint16_t)buf[5] << 8 | buf[6];
    bmsData.maxCharge = buf[7] * 2;

    lastBMSTime = millis();

    // Update shared data
    sharedBMSData.set(bmsData);

    // Set BMS alive event
    xEventGroupSetBits(systemEvents, EVENT_BMS_ALIVE);

#if DEBUG_CAN_MESSAGES
    DEBUG_PRINTF("BMS: SOC=%d%%, V=%dV, I=%dA\n", bmsData.soc, bmsData.voltage, bmsData.current);
#endif
}

//=============================================================================
// DMC MESSAGE PROCESSING
//=============================================================================

void CANManager::processDMCMessage(uint32_t id, const uint8_t* buf, uint8_t len) {
    switch (id) {
        case CAN_ID::DMC_STATUS:
            if (len >= 8) {
                dmcData.ready = (buf[0] & 0x80) != 0;
                dmcData.running = (buf[0] & 0x40) != 0;
                dmcData.torqueActual = (int16_t)((uint16_t)buf[4] << 8 | buf[5]) * 0.01f;
                dmcData.speedActual = (int16_t)((uint16_t)buf[6] << 8 | buf[7]);

                sharedDMCData.set(dmcData);

                if (dmcData.ready) {
                    xEventGroupSetBits(systemEvents, EVENT_DMC_READY);
                } else {
                    xEventGroupClearBits(systemEvents, EVENT_DMC_READY);
                }
            }
            break;

        case CAN_ID::DMC_POWER:
            if (len >= 8) {
                dmcData.dcVoltage = ((uint16_t)buf[0] << 8 | buf[1]) * 0.1f;
                dmcData.dcCurrent = ((uint16_t)buf[2] << 8 | buf[3]) * 0.1f;
                sharedDMCData.set(dmcData);
            }
            break;

        case CAN_ID::DMC_TEMPERATURE:
            if (len >= 5) {
                dmcData.tempInverter = ((uint16_t)buf[0] << 8 | buf[1]) * 0.5f;
                dmcData.tempMotor = ((uint16_t)buf[2] << 8 | buf[3]) * 0.5f;
                sharedDMCData.set(dmcData);
            }
            break;
    }
}

//=============================================================================
// NLG MESSAGE PROCESSING
//=============================================================================

void CANManager::processNLGMessage(uint32_t id, const uint8_t* buf, uint8_t len) {
    switch (id) {
        case CAN_ID::NLG5_ST:
            if (len >= 4) {
                nlgData.state = (buf[0] >> 5) & 0x07;
                nlgData.dcVoltage = (((uint16_t)(buf[0] & 0x1F) << 8) | buf[1]);
                nlgData.dcCurrent = (((uint16_t)(buf[2] & 0x07) << 8) | buf[3]);
                sharedNLGData.set(nlgData);
            }
            break;

        case CAN_ID::NLG5_ACT_I:
        case CAN_ID::NLG5_ACT_II:
        case CAN_ID::NLG5_TEMP:
        case CAN_ID::NLG5_ERR:
            // Additional NLG5 messages - handle if needed
            // For now, just acknowledge receipt
            break;
    }
}

//=============================================================================
// DMC CONTROL
//=============================================================================

void CANManager::sendDMCControl(int16_t torqueNm, bool enable) {
    memset(dmcControlBuffer, 0, 8);

    dmcControlBuffer[0] = (enable << 7) | (0 << 6) | (1 << 5) | (1 << 1) | (1 << 0);
    dmcControlBuffer[1] = 0;

    int16_t speedLimit = Motor::MAX_RPM;
    dmcControlBuffer[2] = (speedLimit >> 8) & 0xFF;
    dmcControlBuffer[3] = speedLimit & 0xFF;

    int16_t torqueScaled = torqueNm * 100;
    dmcControlBuffer[4] = (torqueScaled >> 8) & 0xFF;
    dmcControlBuffer[5] = torqueScaled & 0xFF;

    dmcControlBuffer[6] = 0;
    dmcControlBuffer[7] = 0;

    sendCANMessage(CAN_ID::DMC_CONTROL, 8, dmcControlBuffer);

#if DEBUG_CAN_MESSAGES
    DEBUG_PRINTF("DMC TX: Enable=%d, Torque=%dNm\n", enable, torqueNm);
#endif
}

void CANManager::sendDMCLimits() {
    BMSData bms = sharedBMSData.get();

    memset(dmcLimitsBuffer, 0, 8);

    uint16_t voltLimitMotor = Battery::MIN_VOLTAGE * 10;
    dmcLimitsBuffer[0] = (voltLimitMotor >> 8) & 0xFF;
    dmcLimitsBuffer[1] = voltLimitMotor & 0xFF;

    uint16_t voltLimitGen = (Battery::MAX_VOLTAGE + 4) * 10;
    dmcLimitsBuffer[2] = (voltLimitGen >> 8) & 0xFF;
    dmcLimitsBuffer[3] = voltLimitGen & 0xFF;

    uint16_t currLimitMotor = bms.maxDischarge * 10;
    dmcLimitsBuffer[4] = (currLimitMotor >> 8) & 0xFF;
    dmcLimitsBuffer[5] = currLimitMotor & 0xFF;

    uint16_t currLimitGen = bms.maxCharge * 10;
    dmcLimitsBuffer[6] = (currLimitGen >> 8) & 0xFF;
    dmcLimitsBuffer[7] = currLimitGen & 0xFF;

    sendCANMessage(CAN_ID::DMC_LIMITS, 8, dmcLimitsBuffer);
}

//=============================================================================
// NLG CONTROL
//=============================================================================

void CANManager::sendNLGControl(uint8_t stateDemand) {
    BMSData bms = sharedBMSData.get();

    memset(nlgControlBuffer, 0, 8);

    uint16_t voltageScaled = Battery::MAX_VOLTAGE * 10;
    nlgControlBuffer[0] = (0 << 7) | (0 << 6) | (0 << 5) | ((voltageScaled >> 8) & 0x1F);
    nlgControlBuffer[1] = voltageScaled & 0xFF;

    uint16_t currentScaled = (bms.maxCharge + 102.4f) * 10;
    nlgControlBuffer[2] = (stateDemand << 5) | ((currentScaled >> 8) & 0x07);
    nlgControlBuffer[3] = currentScaled & 0xFF;

    uint16_t acCurrentScaled = (32 + 102.4f) * 10;
    nlgControlBuffer[4] = (0 << 4) | ((acCurrentScaled >> 8) & 0x07);
    nlgControlBuffer[5] = acCurrentScaled & 0xFF;

    nlgControlBuffer[6] = 0;
    nlgControlBuffer[7] = 0;

    sendCANMessage(CAN_ID::NLG5_CTL, 8, nlgControlBuffer);

#if DEBUG_CAN_MESSAGES
    DEBUG_PRINTF("NLG TX: State=%d\n", stateDemand);
#endif
}

//=============================================================================
// STATUS QUERIES
//=============================================================================

bool CANManager::isBMSAlive() const {
    return (millis() - lastBMSTime) < 2000;
}

bool CANManager::isDMCReady() const {
    DMCData dmc = sharedDMCData.get();
    return dmc.ready;
}

bool CANManager::isCharging() const {
    NLGData nlg = sharedNLGData.get();
    // NLG state values: 0=idle, 1-7=various charging states
    // Consider charging if state > 0
    return (nlg.state > 0 && nlg.state < 7);
}

//=============================================================================
// UTILITY
//=============================================================================

void CANManager::resetBuffers() {
    memset(dmcControlBuffer, 0, 8);
    memset(dmcLimitsBuffer, 0, 8);
    memset(nlgControlBuffer, 0, 8);
}

bool CANManager::sendCANMessage(uint32_t id, uint8_t len, const uint8_t* data) {
    // Take SPI mutex
    if (xSemaphoreTake(spiMutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        DEBUG_PRINTLN("CAN TX: Failed to acquire SPI mutex");
        return false;
    }

    // Send message
    uint8_t result = CAN.sendMsgBuf(id, 0, len, (uint8_t*)data);

    // Release mutex
    xSemaphoreGive(spiMutex);

    return (result == CAN_OK);
}
