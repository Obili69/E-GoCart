#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>
#include "config.h"

//=============================================================================
// THREAD-SAFE DATA STRUCTURES FOR FREERTOS
//=============================================================================

//-----------------------------------------------------------------------------
// CAN DATA STRUCTURES
//-----------------------------------------------------------------------------

// BMS Data Structure
struct BMSData {
    uint8_t soc;              // State of charge (%)
    uint16_t voltage;         // Pack voltage (V)
    int16_t current;          // Pack current (A)
    uint16_t maxDischarge;    // Max discharge current (A)
    uint16_t maxCharge;       // Max charge current (A)
    float minCellVoltage;     // Minimum cell voltage (V)
    float maxCellVoltage;     // Maximum cell voltage (V)
    float temperature;        // Battery temperature (°C)
};

// DMC Data Structure
struct DMCData {
    bool ready;               // DMC ready flag
    bool running;             // DMC running flag
    float speedActual;        // Motor speed (RPM)
    float torqueActual;       // Actual torque (Nm)
    float dcVoltage;          // DC bus voltage (V)
    float dcCurrent;          // DC bus current (A)
    float tempInverter;       // Inverter temperature (°C)
    float tempMotor;          // Motor temperature (°C)

    // Error tracking (from DMC_ERR message 0x25A)
    uint64_t errorBits;       // Full 64-bit error word
    bool errorActive;         // Any error present
    unsigned long lastErrorTime;  // Timestamp of last error
};

// NLG Data Structure
struct NLGData {
    uint8_t state;            // Charger state
    uint16_t dcVoltage;       // Charging voltage (V)
    uint16_t dcCurrent;       // Charging current (A)
    bool connectorLocked;     // Connector lock status
    float temperature;        // Charger temperature (°C)
};

//-----------------------------------------------------------------------------
// CAN MESSAGE STRUCTURE (for queue)
//-----------------------------------------------------------------------------
struct CANMessage {
    uint32_t id;
    uint8_t len;
    uint8_t data[8];
    unsigned long timestamp;
};

//-----------------------------------------------------------------------------
// INPUT EVENT STRUCTURE (for queue)
//-----------------------------------------------------------------------------
enum class InputEventType : uint8_t {
    START_BUTTON,
    STOP_BUTTON,
    RESET_BUTTON,
    DIRECTION_TOGGLE,
    IL_OPEN,
    IL_CLOSE,
    CHARGER_CONNECT,
    CHARGER_DISCONNECT,
    BRAKE_PRESS,
    BRAKE_RELEASE
};

struct InputEvent {
    InputEventType type;
    uint32_t value;  // Optional value (e.g., button hold time)
    unsigned long timestamp;
};

//-----------------------------------------------------------------------------
// VEHICLE TELEMETRY (for webserver)
//-----------------------------------------------------------------------------
struct VehicleTelemetry {
    // Vehicle state
    VehicleState state;
    GearState gear;
    bool systemReady;

    // Motor/Vehicle
    float speedKmh;
    int16_t torqueDemand;
    float torqueActual;
    float motorRPM;

    // Battery
    uint8_t soc;
    uint16_t voltage;
    int16_t current;
    float powerKw;
    float minCellVoltage;
    float maxCellVoltage;

    // Temperatures
    float tempMotor;
    float tempInverter;
    float tempBattery;

    // Inputs
    float throttlePercent;
    float regenPercent;
    int16_t throttleRawADC;
    int16_t regenRawADC;
    bool brakePressed;
    bool ilClosed;

    // Safety Allowances (NEW)
    bool chargeAllowed;         // Charge allowance from BMS (via MCP A0)
    bool dischargeAllowed;      // Discharge allowance from BMS (via MCP A1)
    bool regenEnabled;          // Regen enabled state (disabled if charge not allowed)

    // Contactor States (NEW)
    uint8_t contactorState;     // ContactorState enum value
    bool chargeArmed;           // Charge path armed
    bool dischargeArmed;        // Discharge path armed
    bool contactorError;        // Contactor error flag

    // CAN status
    bool bmsAlive;
    bool dmcReady;
    bool chargerConnected;

    // Timestamp
    unsigned long timestamp;
};

//-----------------------------------------------------------------------------
// ERROR REPORTING (for web error dashboard)
//-----------------------------------------------------------------------------

// Error severity levels
enum class ErrorSeverity : uint8_t {
    INFO,       // Informational (balancing active, etc.)
    WARNING,    // Operation limited but safe
    CRITICAL    // Requires immediate attention/shutdown
};

// Individual error entry with metadata
struct ErrorEntry {
    const char* code;           // Short code (e.g., "BMS_OVERTEMP")
    const char* message;        // Human-readable message
    ErrorSeverity severity;     // Severity level
    bool active;                // Currently active
    unsigned long timestamp;    // When error occurred/cleared

    ErrorEntry() : code(""), message(""), severity(ErrorSeverity::INFO), active(false), timestamp(0) {}
};

// System-wide error status (all subsystems)
struct SystemErrorStatus {
    // ========== BMS ERRORS ==========
    ErrorEntry bmsOverTemp;              // Battery over-temperature
    ErrorEntry bmsOverCharge;            // Over-charging detected
    ErrorEntry bmsCellError;             // Cell/string error
    ErrorEntry bmsOverCurrent;           // Discharge overcurrent
    ErrorEntry bmsOverDischarge;         // Over-discharge detected
    ErrorEntry bmsTempNegative;          // Temperature below zero
    ErrorEntry bmsTimeout;               // BMS communication lost
    ErrorEntry bmsChargeMOSDisabled;     // Charge MOS disabled
    ErrorEntry bmsDischargeMOSDisabled;  // Discharge MOS disabled

    // ========== CHARGER (NLG5) ERRORS ==========
    ErrorEntry chargerMainsFuse;         // Mains fuse defective
    ErrorEntry chargerOutputFuse;        // Output fuse defective
    ErrorEntry chargerShortCircuit;      // Power stage short circuit
    ErrorEntry chargerMainsOV;           // Mains overvoltage
    ErrorEntry chargerBatteryOV;         // Battery overvoltage
    ErrorEntry chargerPolarity;          // Wrong battery polarity
    ErrorEntry chargerCANTimeout;        // CAN control timeout
    ErrorEntry chargerCANOff;            // CAN off (TX buffer full)
    ErrorEntry chargerTempSensor;        // Temperature sensor error
    ErrorEntry chargerCRCError;          // Checksum/CRC error
    ErrorEntry chargerTimeout;           // VCU charger timeout

    // ========== CHARGER (NLG5) WARNINGS ==========
    ErrorEntry chargerLowMainsV;         // Mains voltage too low
    ErrorEntry chargerLowBattV;          // Battery voltage too low
    ErrorEntry chargerHighTemp;          // Internal over-temperature
    ErrorEntry chargerControlOOR;        // Control out of range

    // ========== INVERTER (DMC) ERRORS ==========
    ErrorEntry dmcCANTimeout;            // CAN control timeout
    ErrorEntry dmcInverterOvertemp;      // Inverter over-temperature
    ErrorEntry dmcMotorOvertemp;         // Motor over-temperature
    ErrorEntry dmcSpeedSensor;           // Speed sensor error
    ErrorEntry dmcUndervoltage;          // DC under-voltage
    ErrorEntry dmcOvervoltage;           // DC over-voltage
    ErrorEntry dmcDCCurrentError;        // DC current measurement error
    ErrorEntry dmcInitError;             // Initialization error
    ErrorEntry dmcShortCircuit;          // Power stage short circuit
    ErrorEntry dmcACOvercurrent;         // AC overcurrent
    ErrorEntry dmcTimeout;               // VCU DMC timeout
    ErrorEntry dmcSpeedSensorSupply;     // Speed sensor supply error
    ErrorEntry dmcLimitsInvalid;         // Limits message invalid
    ErrorEntry dmcControlInvalid;        // Control message invalid
    ErrorEntry dmcVoltageMeas;           // Voltage measurement error
    ErrorEntry dmcEEPROMError;           // Motor EEPROM error
    ErrorEntry dmcStorageError;          // Data storage error

    // ========== INVERTER (DMC) WARNINGS ==========
    ErrorEntry dmcGeneralWarning;        // General warning flag
    ErrorEntry dmcHVUndervoltage;        // HV under-voltage warning
    ErrorEntry dmcTempSensorWarning;     // Temperature sensor warning

    // ========== VCU SYSTEM ERRORS ==========
    ErrorEntry vcuContactorError;        // Contactor operation error
    ErrorEntry vcuEmergencyStop;         // Emergency stop activated
    ErrorEntry vcuPrechargeTimeout;      // Precharge timeout
    ErrorEntry vcuMotorOvertemp;         // Motor temp critical
    ErrorEntry vcuInverterOvertemp;      // Inverter temp critical
    ErrorEntry vcuBatteryOvertemp;       // Battery temp critical
    ErrorEntry vcuLowVoltage;            // Pack voltage critically low
    ErrorEntry vcuLowSOC;                // SOC critically low
    ErrorEntry vcuCellCritical;          // Cell voltage critical
    ErrorEntry vcuInterlock;             // Interlock open

    // ========== SUMMARY COUNTERS ==========
    uint16_t criticalCount;
    uint16_t warningCount;
    uint16_t infoCount;
    bool hasAnyError;
};

//-----------------------------------------------------------------------------
// RUNTIME CONFIG (web-configurable, persisted to LittleFS)
//-----------------------------------------------------------------------------
struct RuntimeConfigData {
    // CAN Timing
    uint32_t canFastCycle;      // 10-50ms
    uint32_t canSlowCycle;      // 100-1000ms

    // Motor Limits
    int16_t maxTorqueNm;
    int16_t maxRegenNm;
    int16_t maxReverseNm;

    // Safety Limits
    float maxMotorTemp;
    float maxInverterTemp;
    float maxBatteryTemp;
    float criticalCellVoltage;

    // WiFi Mode
    bool enableAP;              // Access Point mode
    bool enableSTA;             // Station mode
    char staSSID[32];
    char staPassword[64];

    // Debug
    bool debugMode;             // Prevents sleep, keeps WiFi on
    bool enableOTA;             // Allow OTA updates

    // Charging Configuration
    float storageVoltagePerCell;      // Default: 3.80V, Range: 3.70-3.90V
    float maxChargeVoltagePerCell;    // Default: 4.17V, Range: 4.10-4.20V
    float bulkChargeVoltagePerCell;   // Default: 4.10V, Range: 4.00-4.15V
    float maxChargeCurrent;           // Default: 10.0A, Range: 1.0-12.5A (NLG5 limit)
    float storageChargeCurrent;       // Default: 5.0A, Range: 1.0-10.0A
    float mainsCurrentLimit;          // Default: 13.0A, Range: 6.0-16.0A
    uint16_t chargeTimeoutMinutes;    // Default: 240 (4 hours), Range: 60-600
    float maxChargeTemp;              // Default: 45.0°C, Range: 35.0-50.0°C
    float minChargeTemp;              // Default: 0.0°C, Range: -5.0-10.0°C
    uint8_t chargingPreset;           // 0=Custom, 1=Storage, 2=Fast, 3=Gentle, 4=Eco
    bool autoStopAtStorage;           // Stop automatically when reaching storage voltage
    bool balancingEnabled;            // Enable cell balancing during charge

    // Pedal Calibration
    int16_t throttleMinADC;           // Throttle pot minimum ADC value
    int16_t throttleMaxADC;           // Throttle pot maximum ADC value
    int16_t regenMinADC;              // Regen pot minimum ADC value
    int16_t regenMaxADC;              // Regen pot maximum ADC value

    // Checksum for validation
    uint32_t checksum;
};

//-----------------------------------------------------------------------------
// TASK STATISTICS (for monitoring)
//-----------------------------------------------------------------------------
struct TaskStats {
    char name[16];
    uint8_t priority;
    uint32_t stackHighWater;    // Minimum free stack (bytes)
    uint32_t cpuPercent;        // CPU usage percentage
    eTaskState state;           // Running, Ready, Blocked, etc.
    uint32_t runtime;           // Total runtime (ticks)
    bool watchdogOK;            // Watchdog status
};

//-----------------------------------------------------------------------------
// THREAD-SAFE DATA WRAPPER
//-----------------------------------------------------------------------------
template<typename T>
class ThreadSafeData {
public:
    ThreadSafeData() {
        mutex = xSemaphoreCreateMutex();
        if (mutex == NULL) {
            DEBUG_PRINTLN("ERROR: Failed to create mutex!");
        }
    }

    ~ThreadSafeData() {
        if (mutex != NULL) {
            vSemaphoreDelete(mutex);
        }
    }

    // Get a copy of the data (thread-safe)
    T get() {
        T copy;
        if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
            copy = data;
            xSemaphoreGive(mutex);
        }
        return copy;
    }

    // Set the data (thread-safe)
    void set(const T& newData) {
        if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
            data = newData;
            xSemaphoreGive(mutex);
        }
    }

    // Update specific fields using a lambda
    template<typename Func>
    void update(Func func) {
        if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
            func(data);
            xSemaphoreGive(mutex);
        }
    }

    // Try to get data with timeout (returns false if timeout)
    bool tryGet(T& copy, TickType_t timeout = 100) {
        if (xSemaphoreTake(mutex, timeout) == pdTRUE) {
            copy = data;
            xSemaphoreGive(mutex);
            return true;
        }
        return false;
    }

private:
    T data;
    SemaphoreHandle_t mutex;

    // Prevent copying
    ThreadSafeData(const ThreadSafeData&) = delete;
    ThreadSafeData& operator=(const ThreadSafeData&) = delete;
};

//-----------------------------------------------------------------------------
// GLOBAL SHARED DATA (extern declarations, defined in main.cpp)
//-----------------------------------------------------------------------------
extern QueueHandle_t canRxQueue;          // Queue for incoming CAN messages
extern QueueHandle_t inputEventQueue;     // Queue for input events
extern EventGroupHandle_t systemEvents;   // Event flags for system events

// Thread-safe shared data
extern ThreadSafeData<BMSData> sharedBMSData;
extern ThreadSafeData<DMCData> sharedDMCData;
extern ThreadSafeData<NLGData> sharedNLGData;
extern ThreadSafeData<VehicleTelemetry> sharedTelemetry;
extern ThreadSafeData<RuntimeConfigData> sharedRuntimeConfig;
extern ThreadSafeData<SystemErrorStatus> sharedErrorStatus;

//-----------------------------------------------------------------------------
// SYSTEM EVENT BITS (for EventGroupHandle_t)
//-----------------------------------------------------------------------------
#define EVENT_BMS_ALIVE         (1 << 0)
#define EVENT_DMC_READY         (1 << 1)
#define EVENT_BATTERY_ARMED     (1 << 2)
#define EVENT_PRECHARGE_DONE    (1 << 3)
#define EVENT_EMERGENCY_STOP    (1 << 4)
#define EVENT_CHARGER_CONNECTED (1 << 5)
#define EVENT_IL_CLOSED         (1 << 6)
#define EVENT_WIFI_CONNECTED    (1 << 7)
#define EVENT_OTA_MODE          (1 << 8)

//-----------------------------------------------------------------------------
// HELPER FUNCTIONS
//-----------------------------------------------------------------------------

// Calculate checksum for RuntimeConfigData
inline uint32_t calculateConfigChecksum(const RuntimeConfigData& config) {
    uint32_t sum = 0;
    const uint8_t* ptr = (const uint8_t*)&config;
    size_t size = sizeof(RuntimeConfigData) - sizeof(uint32_t); // Exclude checksum field
    for (size_t i = 0; i < size; i++) {
        sum += ptr[i];
    }
    return sum;
}

// Validate RuntimeConfigData
inline bool validateConfig(const RuntimeConfigData& config) {
    uint32_t expected = calculateConfigChecksum(config);
    return (config.checksum == expected);
}

// Validate charging configuration ranges
inline bool validateChargingConfig(const RuntimeConfigData& config) {
    // Voltage per cell checks
    if (config.storageVoltagePerCell < 3.70f || config.storageVoltagePerCell > 3.90f)
        return false;
    if (config.maxChargeVoltagePerCell < 4.10f || config.maxChargeVoltagePerCell > 4.20f)
        return false;
    if (config.bulkChargeVoltagePerCell < 4.00f || config.bulkChargeVoltagePerCell > 4.15f)
        return false;

    // Current checks
    if (config.maxChargeCurrent < 1.0f || config.maxChargeCurrent > 12.5f)
        return false;
    if (config.storageChargeCurrent < 1.0f || config.storageChargeCurrent > 10.0f)
        return false;
    if (config.mainsCurrentLimit < 6.0f || config.mainsCurrentLimit > 16.0f)
        return false;

    // Timeout check
    if (config.chargeTimeoutMinutes < 60 || config.chargeTimeoutMinutes > 600)
        return false;

    // Temperature checks
    if (config.maxChargeTemp < 35.0f || config.maxChargeTemp > 50.0f)
        return false;
    if (config.minChargeTemp < -5.0f || config.minChargeTemp > 10.0f)
        return false;

    return true;
}

// Helper functions to calculate pack voltages from per-cell settings
inline float getStoragePackVoltage(const RuntimeConfigData& config) {
    return config.storageVoltagePerCell * Battery::NUM_CELLS;  // × 104
}

inline float getMaxChargePackVoltage(const RuntimeConfigData& config) {
    return config.maxChargeVoltagePerCell * Battery::NUM_CELLS;  // × 104
}

inline float getBulkChargePackVoltage(const RuntimeConfigData& config) {
    return config.bulkChargeVoltagePerCell * Battery::NUM_CELLS;  // × 104
}

// Calculate estimated DC current from AC mains limit
inline float estimateDCCurrentFromMains(float mainsCurrentLimit, float targetVoltage) {
    constexpr float MAINS_VOLTAGE = 230.0f;   // Assume 230VAC
    constexpr float NLG5_EFFICIENCY = 0.93f;  // 93% efficient
    constexpr float NLG5_MAX_CURRENT = 12.5f; // Hardware limit

    float maxACPower = mainsCurrentLimit * MAINS_VOLTAGE;
    float maxDCPower = maxACPower * NLG5_EFFICIENCY;
    float maxDCCurrent = maxDCPower / targetVoltage;

    // Clamp to hardware limit
    if (maxDCCurrent > NLG5_MAX_CURRENT) {
        maxDCCurrent = NLG5_MAX_CURRENT;
    }

    return maxDCCurrent;
}

// Initialize default runtime config
inline RuntimeConfigData getDefaultRuntimeConfig() {
    RuntimeConfigData config;

    // CAN Timing
    config.canFastCycle = RuntimeConfig::CAN_FAST_DEFAULT;
    config.canSlowCycle = Timing::CAN_SLOW_CYCLE;

    // Motor Limits
    config.maxTorqueNm = RuntimeConfig::TORQUE_MAX_DEFAULT;
    config.maxRegenNm = RuntimeConfig::REGEN_MAX_DEFAULT;
    config.maxReverseNm = RuntimeConfig::REVERSE_MAX_DEFAULT;

    // Safety Limits
    config.maxMotorTemp = Safety::MAX_MOTOR_TEMP;
    config.maxInverterTemp = Safety::MAX_INVERTER_TEMP;
    config.maxBatteryTemp = Safety::MAX_BATTERY_TEMP;
    config.criticalCellVoltage = Safety::CRITICAL_CELL_VOLTAGE;

    // WiFi Mode
    config.enableAP = true;
    config.enableSTA = false;
    strncpy(config.staSSID, WiFi_Config::STA_SSID, 31);
    strncpy(config.staPassword, WiFi_Config::STA_PASSWORD, 63);

    // Debug
    config.debugMode = (DEBUG_MODE == 1);
    config.enableOTA = true;  // Enable OTA by default for development

    // Charging Configuration
    config.storageVoltagePerCell = 3.80f;       // 395.2V pack (104S) - optimal storage
    config.maxChargeVoltagePerCell = 4.17f;     // 433.68V pack - safe maximum
    config.bulkChargeVoltagePerCell = 4.10f;    // 426.4V pack - bulk charge
    config.maxChargeCurrent = 10.0f;            // 10A default (safe for most circuits)
    config.storageChargeCurrent = 5.0f;         // 5A for gentle storage charge
    config.mainsCurrentLimit = 13.0f;           // 13A (2.99kW @ 230V)
    config.chargeTimeoutMinutes = 240;          // 4 hours maximum
    config.maxChargeTemp = 45.0f;               // 45°C maximum safe temperature
    config.minChargeTemp = 0.0f;                // No charging below freezing
    config.chargingPreset = 0;                  // Custom (no preset)
    config.autoStopAtStorage = true;            // Auto-stop at storage voltage
    config.balancingEnabled = true;             // Enable cell balancing

    // Pedal Calibration
    config.throttleMinADC = ADC_Cal::THROTTLE_MIN;
    config.throttleMaxADC = ADC_Cal::THROTTLE_MAX;
    config.regenMinADC = ADC_Cal::REGEN_MIN;
    config.regenMaxADC = ADC_Cal::REGEN_MAX;

    // Calculate checksum
    config.checksum = calculateConfigChecksum(config);

    return config;
}
