#include "can_manager.h"
#include "bms_manager.h"

// Static instance for ISR
CANManager* CANManager::_instance = nullptr;

CANManager::CANManager()
    : CAN1(Pins::CAN1_CS)  // DMC + NLG on CAN1 (MCP2515)
    , spi(nullptr)
    , twaiInitialized(false)
    , bmsManager(nullptr)
    , lastBMSTime(0)
    , lastSlowCycleTime(0)
    , spiMutex(nullptr)
    , canRxTaskHandle(nullptr)
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

    // Initialize SPI bus (shared by both CAN controllers)
    DEBUG_PRINTLN("Initializing SPI bus...");
    DEBUG_PRINTF("  SCK: %d, MISO: %d, MOSI: %d\n", Pins::SPI_SCK, Pins::SPI_MISO, Pins::SPI_MOSI);
    spi = new SPIClass(HSPI);
    spi->begin(Pins::SPI_SCK, Pins::SPI_MISO, Pins::SPI_MOSI, Pins::CAN1_CS);

    // Reduce SPI speed for better reliability (default is 10MHz, reduce to 2MHz)
    spi->setFrequency(2000000);  // 2 MHz
    DEBUG_PRINTLN("  SPI frequency: 2 MHz");

    // Configure CAN1 CS pin
    pinMode(Pins::CAN1_CS, OUTPUT);
    digitalWrite(Pins::CAN1_CS, HIGH);  // Deselect
    DEBUG_PRINTF("  CAN1 CS: %d\n", Pins::CAN1_CS);

    // Set SPI for CAN1 controller
    CAN1.setSPI(spi);

    // Initialize CAN1 (DMC + NLG) - 500kbps with 16MHz crystal
    uint8_t retries = 0;
    const uint8_t MAX_RETRIES = 5;

    DEBUG_PRINTLN("Initializing CAN1 (DMC/NLG) @ 500 kbps...");
    DEBUG_PRINTF("  CS Pin: %d, INT Pin: %d, Crystal: 16MHz\n", Pins::CAN1_CS, Pins::CAN1_INT);
    while (CAN_OK != CAN1.begin(CAN_500KBPS, MCP_16MHz)) {
        DEBUG_PRINTF("ERROR: CAN1 init failed (attempt %d/%d), retrying...\n", retries + 1, MAX_RETRIES);
        delay(100);

        if (++retries >= MAX_RETRIES) {
            DEBUG_PRINTLN("ERROR: CAN1 init failed after max retries!");
            return false;
        }
    }
    DEBUG_PRINTLN("CAN1 initialized successfully (500 kbps)");

    // Initialize CAN2 (BMS) - 250kbps using ESP32-S3 internal TWAI controller
#if ENABLE_CAN2
    DEBUG_PRINTLN("Initializing CAN2 (BMS) @ 250 kbps using TWAI...");
    DEBUG_PRINTF("  TX Pin: %d, RX Pin: %d\n", Pins::CAN2_TX, Pins::CAN2_RX);

    // Configure TWAI timing for 250kbps (assuming 80MHz APB clock)
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();

    // Configure TWAI filter to accept all messages
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    // Configure TWAI general settings
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)Pins::CAN2_TX,
        (gpio_num_t)Pins::CAN2_RX,
        TWAI_MODE_NORMAL
    );
    g_config.tx_queue_len = 20;
    g_config.rx_queue_len = 20;

    // Install TWAI driver
    esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
    if (err == ESP_OK) {
        DEBUG_PRINTLN("  TWAI driver installed");

        // Start TWAI driver
        err = twai_start();
        if (err == ESP_OK) {
            twaiInitialized = true;
            DEBUG_PRINTLN("CAN2 (TWAI) initialized successfully (250 kbps)");
        } else {
            DEBUG_PRINTF("ERROR: TWAI start failed: %s\n", esp_err_to_name(err));
            return false;
        }
    } else {
        DEBUG_PRINTF("ERROR: TWAI driver install failed: %s\n", esp_err_to_name(err));
        DEBUG_PRINTLN("  Check: Are TX/RX pins correct?");
        DEBUG_PRINTLN("  Check: Are pins already in use?");
        return false;
    }
#else
    DEBUG_PRINTLN("CAN2 disabled (ENABLE_CAN2 = 0)");
#endif

    // Set instance for ISR
    setInstance(this);

    // Attach interrupt for CAN1 only (CAN2 is polled)
    pinMode(Pins::CAN1_INT, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(Pins::CAN1_INT), canISR, FALLING);

    DEBUG_PRINTLN("CAN1 interrupt attached");

    return true;
}

void CANManager::setBMSManager(BMSManager* bmsMgr) {
    bmsManager = bmsMgr;
    DEBUG_PRINTLN("CANManager: BMS Manager linked");
}

void IRAM_ATTR CANManager::canISR() {
    // Minimal ISR - just notify the task to wake up and read the message
    if (_instance != nullptr && _instance->canRxTaskHandle != nullptr) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(_instance->canRxTaskHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void CANManager::readCAN1Message() {
    // This runs in task context (not ISR) - called when ISR notifies us
    // Take SPI mutex before accessing CAN controller
    if (xSemaphoreTake(spiMutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        return;  // Couldn't get mutex, skip this read
    }

    // Read all available messages
    while (CAN_MSGAVAIL == CAN1.checkReceive()) {
        CANMessage msg;
        msg.timestamp = millis();

        if (CAN_OK == CAN1.readMsgBuf(&msg.len, msg.data)) {
            msg.id = CAN1.getCanId();

            // Push to queue (can block briefly since we're in task context)
            xQueueSend(canRxQueue, &msg, pdMS_TO_TICKS(5));
        }
    }

    xSemaphoreGive(spiMutex);
}

void CANManager::pollCAN2() {
    // Poll CAN2 (BMS) for new messages - called from CAN RX task
    // Uses ESP32-S3 internal TWAI controller (no SPI needed)

#if ENABLE_CAN2
    if (!twaiInitialized) {
        return;  // TWAI not initialized
    }

    // Try to receive message from TWAI (non-blocking)
    twai_message_t twai_msg;
    esp_err_t err = twai_receive(&twai_msg, 0);  // 0 = non-blocking

    if (err == ESP_OK) {
        // Convert TWAI message to CAN message format
        CANMessage msg;
        msg.timestamp = millis();
        msg.id = twai_msg.identifier;
        msg.len = twai_msg.data_length_code;
        memcpy(msg.data, twai_msg.data, msg.len);

        // Push to queue (can block briefly since not in ISR)
        xQueueSend(canRxQueue, &msg, pdMS_TO_TICKS(10));
    }
    // ESP_ERR_TIMEOUT means no message available - this is normal
#endif
}

void CANManager::processRxQueue() {
    CANMessage msg;

    // Check if we've been notified by CAN1 ISR (wait up to 1ms)
    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1)) > 0) {
        // ISR fired - read all pending CAN1 messages
        readCAN1Message();
    }

    // Poll CAN2 (BMS) for new messages since it has no interrupt
    pollCAN2();

    // Process all messages in queue (from both CAN1 interrupt and CAN2 polling)
    while (xQueueReceive(canRxQueue, &msg, 0) == pdTRUE) {
        processCANMessage(msg);
    }
}

void CANManager::processCANMessage(const CANMessage& msg) {
#if DEBUG_CAN_MESSAGES
    DEBUG_PRINTF("CAN RX: 0x%08X [%d] ", msg.id, msg.len);
    for (int i = 0; i < msg.len; i++) {
        DEBUG_PRINTF("%02X ", msg.data[i]);
    }
    DEBUG_PRINTLN();
#endif

    // Route message to appropriate handler
    // BMS uses extended CAN IDs - mask to 29-bit for comparison
    uint32_t masked_id = msg.id & 0x1FFFFFFF;

    if (masked_id == CAN_ID::BMS_TX) {
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
    // Mask to 29-bit extended ID for comparison
    uint32_t masked_id = id & 0x1FFFFFFF;

    if (masked_id != CAN_ID::BMS_TX) {
        return;  // Not a BMS message
    }

    if (len < 1) {
        return;  // Need at least 1 byte for group ID
    }

    // Route to BMSManager for proper protocol handling
    if (bmsManager != nullptr) {
        bmsManager->processMessage(id, buf, len);
        lastBMSTime = millis();
        xEventGroupSetBits(systemEvents, EVENT_BMS_ALIVE);
    } else {
        // Fallback: simple parsing if BMSManager not available
        // Parse BMS status message (Group 3 only)
        if (buf[0] == 0x03) {  // Group 3: voltage, current, power
            bmsData.voltage = ((uint16_t)buf[1] << 8 | buf[2]);  // 0.1V
            bmsData.current = (int16_t)((uint16_t)buf[3] << 8 | buf[4]);  // 0.01A

            
            sharedBMSData.set(bmsData);
            

#if DEBUG_CAN_MESSAGES
            DEBUG_PRINTF("BMS: V=%d.%dV, I=%d.%02dA\n",
                bmsData.voltage / 10, bmsData.voltage % 10,
                bmsData.current / 100, abs(bmsData.current) % 100);
#endif
        }
    }
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

    sendCAN1Message(CAN_ID::DMC_CONTROL, 8, dmcControlBuffer);

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

    sendCAN1Message(CAN_ID::DMC_LIMITS, 8, dmcLimitsBuffer);
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

    sendCAN1Message(CAN_ID::NLG5_CTL, 8, nlgControlBuffer);

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

bool CANManager::sendCAN1Message(uint32_t id, uint8_t len, const uint8_t* data) {
    // Send on CAN1 (DMC + NLG @ 500kbps)
    // Take SPI mutex
    if (xSemaphoreTake(spiMutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        DEBUG_PRINTLN("CAN1 TX: Failed to acquire SPI mutex");
        return false;
    }

    // Send message
    uint8_t result = CAN1.sendMsgBuf(id, 0, len, (uint8_t*)data);

    // Release mutex
    xSemaphoreGive(spiMutex);

    return (result == CAN_OK);
}

bool CANManager::sendCAN2Message(uint32_t id, uint8_t len, const uint8_t* data) {
    // Send on CAN2 (BMS @ 250kbps) using TWAI
    if (!twaiInitialized) {
        return false;
    }

    // Prepare TWAI message
    twai_message_t twai_msg;
    twai_msg.identifier = id;
    twai_msg.data_length_code = len;
    twai_msg.extd = 1;  // Extended ID (29-bit) for BMS
    twai_msg.rtr = 0;   // Data frame, not remote
    twai_msg.ss = 0;    // Not single shot
    twai_msg.self = 0;  // Not self-reception request
    twai_msg.dlc_non_comp = 0;
    memcpy(twai_msg.data, data, len);

    // Send message (10ms timeout)
    esp_err_t err = twai_transmit(&twai_msg, pdMS_TO_TICKS(10));

#if DEBUG_CAN_MESSAGES
    if (err == ESP_OK) {
        DEBUG_PRINTF("CAN2 TX: 0x%03X [%d] (extended)\n", id, len);
    } else {
        DEBUG_PRINTF("CAN2 TX FAILED: 0x%03X\n", id);
    }
#endif

    return (err == ESP_OK);
}
