#pragma once
#include <Arduino.h>

//=============================================================================
// VCU CONFIGURATION - Simplified Version
//=============================================================================

//-----------------------------------------------------------------------------
// ESP32 PINOUT
//-----------------------------------------------------------------------------
namespace Pins {
    // SPI (CAN Controllers)
    constexpr uint8_t SPI_SCK  = 4;
    constexpr uint8_t SPI_MISO = 5;
    constexpr uint8_t SPI_MOSI = 6;

    // CAN1 (DMC/Motor Controller - 500kbps) - MCP2515 via SPI
    constexpr uint8_t CAN1_CS   = 36;
    constexpr uint8_t CAN1_INT  = 37;

    // CAN2 (BMS - 250kbps) - ESP32-S3 internal TWAI controller
    constexpr uint8_t CAN2_TX   = 3;   // TWAI TX pin
    constexpr uint8_t CAN2_RX   = 46;  // TWAI RX pin

    // Legacy compatibility (CAN1 is primary)
    constexpr uint8_t CAN_CS   = CAN1_CS;
    constexpr uint8_t CAN_INT  = CAN1_INT;
    
    // I2C (MCP23017 + ADS1115)
    constexpr uint8_t SDA = 1;
    constexpr uint8_t SCL = 2;
    
    // Digital Inputs (Buttons/Switches)
    constexpr uint8_t START_STOP_BUTTON = 7;   // P1 - Combined Start/Stop Button (2s=start/stop, 15s=reset)
    constexpr uint8_t CHARGER_WAKEUP    = 9;   // P3 - NLG Wakeup
    constexpr uint8_t IL_SWITCH         = 47;  // P5 - Interlock

    // Status LEDs
    constexpr uint8_t STATUS_LED        = 21;  // Status LED (1Hz blink=startup, solid=running, 20Hz=error)
    constexpr uint8_t DIRECTION_LED     = 16;  // Direction LED (solid=Drive, 1Hz=Reverse, off=Neutral)

    // Power Outputs (Relays/Contactors)
    constexpr uint8_t WATER_PUMP             = 38;  // PWI0
    constexpr uint8_t BMS_PWR                = 11;  // LPWI0 - BMS main power
    constexpr uint8_t DMC_ENABLE             = 12;  // LWPI1

    // Charge Path Contactors
    constexpr uint8_t CHARGE_PRECHARGE       = 17;  // LWPI4 - Charge path precharge relay
    constexpr uint8_t MAIN_CHARGE_CONTACTOR  = 13;  // LWPI2 - Main charge contactor
    constexpr uint8_t NLG_ENABLE             = 39;  // LWPI2 - Charger enable (same as charge contactor)

    // Discharge Path Contactors
    constexpr uint8_t DISCHARGE_PRECHARGE    = 14;  // LWPI3 - Discharge path precharge relay
    constexpr uint8_t MAIN_DISCHARGE_CONTACTOR = 18; // LWPI5 - Main discharge contactor

    constexpr uint8_t NEXTION_POWER          = 8;   // Changed from 18 - moved to available pin
    
    // Serial (Nextion Display)
    constexpr uint8_t NEXTION_TX = 42;  // IO3
    constexpr uint8_t NEXTION_RX = 41;  // IO4
    
    // MCP23017 Interrupts
    constexpr uint8_t MCP_INT_A = 45;  // INTA - Low Expansion
    constexpr uint8_t MCP_INT_B = 48;  // INTB - High Expansion
}

//-----------------------------------------------------------------------------
// I2C DEVICE ADDRESSES
//-----------------------------------------------------------------------------
namespace I2C {
    constexpr uint8_t MCP23017_ADDR = 0x20;  // All address pins = 0
    constexpr uint8_t ADS1115_ADDR  = 0x48;  // Default address
}

//-----------------------------------------------------------------------------
// MCP23017 PIN MAPPING
//-----------------------------------------------------------------------------
namespace MCP {
    // Port A (Low Expansion) - Safety Critical Inputs with Interrupt
    constexpr uint8_t CHARGE_ALLOW     = 0;  // GPA0 - Charge allowance from BMS (SAFETY CRITICAL)
    constexpr uint8_t DISCHARGE_ALLOW  = 1;  // GPA1 - Discharge allowance from BMS (SAFETY CRITICAL)
    // GPA2-GPA7 unused

    // Port B (High Expansion) - Input Only
    constexpr uint8_t BRAKE_SWITCH      = 7;  // GPB7 (P0HE)
    constexpr uint8_t DIRECTION_TOGGLE  = 6;  // GPB6 (P1HE)
    // GPB5-GPB0 unused
}

//-----------------------------------------------------------------------------
// ADS1115 ANALOG CHANNELS
//-----------------------------------------------------------------------------
namespace ADC {
    constexpr uint8_t THROTTLE_POT = 0;  // AIN0
    constexpr uint8_t REGEN_POT    = 1;  // AIN1
    // AIN2, AIN3 unused
}

//-----------------------------------------------------------------------------
// CAN BUS CONFIGURATION
//-----------------------------------------------------------------------------
namespace CANBus {
    // CAN1 - DMC/Motor Controller & Charger (fast)
    constexpr uint16_t CAN1_SPEED_KBPS = 500;  // 500 kbps

    // CAN2 - BMS (slower, polled)
    constexpr uint16_t CAN2_SPEED_KBPS = 250;  // 250 kbps
}

//-----------------------------------------------------------------------------
// CAN MESSAGE IDS
//-----------------------------------------------------------------------------
namespace CAN_ID {
    // 192S BMS Messages (29-bit Extended Format, 250kbps on CAN2)
    constexpr uint32_t BMS_TX = 0xF5;  // BMS sends on 0xF5 (extended)
    constexpr uint32_t BMS_RX = 0xF4;  // We send commands on 0xF4 (extended)

    // DMC Messages (Motor Controller)
    constexpr uint16_t DMC_CONTROL      = 0x210;
    constexpr uint16_t DMC_LIMITS       = 0x211;
    constexpr uint16_t DMC_STATUS       = 0x258;
    constexpr uint16_t DMC_POWER        = 0x259;
    constexpr uint16_t DMC_TEMPERATURE  = 0x458;
    constexpr uint16_t DMC_ERR          = 0x25A;  // 602 - Error/Warning flags

    // NLG5 Messages (Charger - from DBC file)
    constexpr uint16_t NLG5_CTL         = 1560;  // 0x618 - Control TO charger
    constexpr uint16_t NLG5_ST          = 1552;  // 0x610 - Status FROM charger
    constexpr uint16_t NLG5_ACT_I       = 1553;  // 0x611 - Actuals I (V, A)
    constexpr uint16_t NLG5_ACT_II      = 1554;  // 0x612 - Actuals II
    constexpr uint16_t NLG5_TEMP        = 1555;  // 0x613 - Temperatures
    constexpr uint16_t NLG5_ERR         = 1556;  // 0x614 - Errors
    constexpr uint16_t NLG5_DIAG_RX     = 1816;  // 0x718 - Diagnostics RX
    constexpr uint16_t NLG5_DIAG_TX     = 1818;  // 0x71A - Diagnostics TX
}

//-----------------------------------------------------------------------------
// VEHICLE STATES
//-----------------------------------------------------------------------------
enum class VehicleState : uint8_t {
    SLEEP,      // Deep sleep mode
    INIT,       // Initializing systems
    READY,      // Ready to drive/charge
    DRIVE,      // Driving mode
    CHARGING    // Charging mode
};

enum class GearState : uint8_t {
    NEUTRAL,
    DRIVE,
    REVERSE
};

//-----------------------------------------------------------------------------
// TIMING CONSTANTS (milliseconds)
//-----------------------------------------------------------------------------
namespace Timing {
    constexpr uint32_t BUTTON_DEBOUNCE          = 50;
    constexpr uint32_t PRECHARGE_TIMEOUT        = 5000;   // 5 seconds
    constexpr uint32_t PRECHARGE_CHECK_INTERVAL = 100;    // Check every 100ms
    constexpr uint32_t PRECHARGE_DELAY_MS       = 3000;   // 3 second precharge time (from BMS config)
    constexpr uint32_t SLEEP_TIMEOUT            = 500;    // 500ms after STOP
    constexpr uint32_t CAN_FAST_CYCLE           = 10;     // DMC control
    constexpr uint32_t CAN_SLOW_CYCLE           = 100;    // Status updates
    constexpr uint32_t DISPLAY_UPDATE           = 50;     // 20 Hz refresh
    constexpr uint32_t CURRENT_ZERO_TIMEOUT     = 2000;   // 2 seconds to verify current = 0
    constexpr uint32_t CURRENT_ZERO_CHECK_INTERVAL = 100; // Check every 100ms
}

//-----------------------------------------------------------------------------
// BATTERY PARAMETERS (104S LiPo Configuration)
//-----------------------------------------------------------------------------
namespace Battery {
    // Cell Configuration
    constexpr uint8_t  NUM_CELLS            = 104;  // 104S LiPo pack (set to 4 for testing)
    constexpr uint8_t  NUM_CELLS_TESTING    = 4;    // Test configuration (4S)
    constexpr uint8_t  NUM_TEMP_SENSORS     = 16;   // 16 temperature modules
    constexpr uint16_t CAPACITY_AH          = 16;   // 16 Ah pack capacity

    // Cell Voltage Limits (LiPo Chemistry)
    constexpr float    CELL_MIN_V           = 3.0f;   // Absolute minimum per cell
    constexpr float    CELL_MAX_V           = 4.2f;   // Absolute maximum per cell
    constexpr float    CELL_NOM_V           = 3.7f;   // Nominal voltage per cell
    constexpr float    CELL_STORAGE_V       = 3.8f;   // Storage voltage per cell
    constexpr float    CELL_BALANCE_V       = 4.18f;  // Balance start voltage

    // Pack Voltage Limits (104S)
    constexpr uint16_t MIN_VOLTAGE          = 14;//312;    // 3.0V * 104S
    constexpr uint16_t MAX_VOLTAGE          = 437;    // 4.2V * 104S
    constexpr uint16_t NOM_VOLTAGE          = 385;    // 3.7V * 104S
    constexpr uint16_t PRECHARGE_TOLERANCE  = 20;     // ±20V for precharge complete

    // Current Thresholds for Safety Checks
    constexpr float CURRENT_ZERO_THRESHOLD  = 0.2f;  // ±0.5A considered "zero" for safety checks

    // Current Limits
    constexpr uint16_t MAX_DISCHARGE_CURRENT = 350;   // 350A max discharge
    constexpr uint16_t MAX_CHARGE_CURRENT    = 50;    // 50A max charge

    // Temperature Limits
    constexpr float    MAX_TEMP_C           = 55.0f;  // Max operating temperature
    constexpr float    MIN_TEMP_C           = 6.0f;   // Min operating temperature (cold protection)

    // Safety Limits
    constexpr uint8_t  MIN_SOC              = 10;     // Emergency cutoff at 10%
    constexpr float    MAX_CELL_DELTA       = 0.05f;  // Max voltage imbalance (V)
    constexpr uint32_t BMS_TIMEOUT_MS       = 1000;   // BMS message timeout

    // BMS Configuration Settings
    constexpr bool     BMS_AUTO_BALANCE     = true;   // Enable auto-balance during charging
    constexpr bool     BMS_AUTO_RESET_CAP   = true;   // Auto-reset capacity counter
    constexpr bool     BMS_ARM_ON_START     = true;   // Arm battery on startup
    constexpr bool     BMS_CHANNEL_DEFAULT  = false;   // Channel default state (on after power-on)
    constexpr uint16_t PRECHARGE_DELAY      = 3;
}

//-----------------------------------------------------------------------------
// MOTOR PARAMETERS
//-----------------------------------------------------------------------------
namespace Motor {
    constexpr int16_t MAX_TORQUE_NM      = 250;   // Forward max torque
    constexpr int16_t MAX_REGEN_NM       = 100;   // Regen max torque
    constexpr int16_t MAX_REVERSE_NM     = 60;   // Reverse max torque
    constexpr int16_t MAX_RPM            = 30000;
    constexpr float   DEADZONE_PERCENT   = 2.0f;  // 2% deadzone around zero

    // Transmission
    constexpr float GEAR_RATIO = 3.9f;
    constexpr float WHEEL_DIAMETER_M = 0.66f;  // 66cm diameter

    // Torque curves (% of max torque at given RPM)
    // Speed zones: 0-1000, 1000-2000, 2000-4000, 4000-6000
    constexpr float TORQUE_CURVE[] = {100.0f, 100.0f, 100.0f, 100.0f};

    // Regen curves (% of max regen at given RPM)
    constexpr float REGEN_CURVE[] = {20.0f, 40.0f, 60.0f, 80.0f};
}

//-----------------------------------------------------------------------------
// COOLING SYSTEM PARAMETERS
//-----------------------------------------------------------------------------
namespace Cooling {
    constexpr float PUMP_ON_TEMP_C       = 60.0f;  // Turn pump ON at 60°C
    constexpr float PUMP_HYSTERESIS_C    = 15.0f;  // Turn pump OFF at 45°C (60 - 15)
    constexpr float PUMP_OFF_TEMP_C      = PUMP_ON_TEMP_C - PUMP_HYSTERESIS_C;
}

//-----------------------------------------------------------------------------
// ADC CALIBRATION (ADS1115 16-bit values)
//-----------------------------------------------------------------------------
namespace ADC_Cal {
    constexpr int16_t THROTTLE_MIN = 0;      // 0V = 0%
    constexpr int16_t THROTTLE_MAX = 26400;  // ~4V = 100%
    constexpr int16_t REGEN_MIN    = 0;
    constexpr int16_t REGEN_MAX    = 26400;
}

//-----------------------------------------------------------------------------
// CHARGER PARAMETERS (NLG5 - Manual Control, No CP)
//-----------------------------------------------------------------------------
namespace Charger {
    // Charging Profile (LiPo - CC/CV)
    constexpr float CHARGE_VOLTAGE_MAX      = 4.17f;  // Max per cell (433.68V pack)
    constexpr float CHARGE_VOLTAGE_BULK     = 4.10f;  // Bulk charge (426.4V pack)
    constexpr float CHARGE_CURRENT_MAX      = 50.0f;  // Max charge current (A)
    constexpr float CHARGE_CURRENT_TAPER    = 2.0f;   // CV phase taper current (A)
    constexpr float CHARGE_POWER_MAX        = 3300.0f; // Max charger power (W)

    // Mains Settings (Manual Mode - No Control Pilot)
    constexpr float MAINS_CURRENT_MAX       = 16.0f;  // Max mains current (A)
    constexpr float MAINS_VOLTAGE_MIN       = 200.0f; // Min mains voltage (V)

    // Temperature Limits
    constexpr float CHARGE_TEMP_MIN         = 0.0f;   // Min charge temp (°C)
    constexpr float CHARGE_TEMP_MAX         = 45.0f;  // Max charge temp (°C)
    constexpr float CHARGER_TEMP_MAX        = 80.0f;  // Max charger temp (°C)

    // Safety
    constexpr uint32_t CHARGER_TIMEOUT_MS   = 2000;   // Charger message timeout
    constexpr uint32_t CHARGE_TIMEOUT_MS    = 14400000; // 4 hour max charge time

    // NLG5 Control Flags (for NLG5_CTL message)
    constexpr uint8_t CTL_FLAG_ENABLE       = 0x80;   // Bit 7: C_C_EN
    constexpr uint8_t CTL_FLAG_CLEAR_ERROR  = 0x40;   // Bit 6: C_C_EL
    constexpr uint8_t CTL_FLAG_CP_VENT      = 0x20;   // Bit 5: C_CP_V
    constexpr uint8_t CTL_FLAG_MAINS_REQ    = 0x10;   // Bit 4: C_MR
}

//-----------------------------------------------------------------------------
// SAFETY LIMITS
//-----------------------------------------------------------------------------
namespace Safety {
    constexpr float MAX_MOTOR_TEMP      = 140.0f;  // °C
    constexpr float MAX_INVERTER_TEMP   = 100.0f;  // °C
    constexpr float MAX_BATTERY_TEMP    = 55.0f;   // °C
    constexpr float CRITICAL_CELL_VOLTAGE = 3.0f;  // V
}

//-----------------------------------------------------------------------------
// FREERTOS CONFIGURATION
//-----------------------------------------------------------------------------
namespace FreeRTOS {
    // Task stack sizes (bytes) - increased to prevent stack overflow
    constexpr uint32_t STACK_CAN_RX        = 4096;
    constexpr uint32_t STACK_CAN_TX        = 4096;  // Increased from 3072
    constexpr uint32_t STACK_VEHICLE_CTRL  = 4096;  // Increased from 3072
    constexpr uint32_t STACK_STATE_MGR     = 4096;  // Increased from 3072
    constexpr uint32_t STACK_SAFETY        = 4096;  // Increased from 2048
    constexpr uint32_t STACK_INPUT         = 4096;  // Increased from 3072
    constexpr uint32_t STACK_DISPLAY       = 4096;  // Increased from 3072
    constexpr uint32_t STACK_WIFI          = 6144;  // Increased from 4096
    constexpr uint32_t STACK_WEBSERVER     = 8192;
    constexpr uint32_t STACK_MONITOR       = 3072;  // Increased from 2048

    // Task priorities (0-24, higher = more priority)
    constexpr uint8_t PRIORITY_CAN_RX      = 24;  // Highest - time critical
    constexpr uint8_t PRIORITY_CAN_TX      = 23;
    constexpr uint8_t PRIORITY_VEHICLE     = 22;
    constexpr uint8_t PRIORITY_STATE       = 21;
    constexpr uint8_t PRIORITY_SAFETY      = 20;
    constexpr uint8_t PRIORITY_INPUT       = 10;
    constexpr uint8_t PRIORITY_DISPLAY     = 5;
    constexpr uint8_t PRIORITY_WIFI        = 4;
    constexpr uint8_t PRIORITY_WEBSERVER   = 3;
    constexpr uint8_t PRIORITY_MONITOR     = 2;

    // Core assignment (ESP32 has 2 cores: 0 and 1)
    constexpr uint8_t CORE_PROTOCOL        = 0;  // CAN, Vehicle, State, Safety
    constexpr uint8_t CORE_APPLICATION     = 1;  // Input, Display, WiFi, Web

    // Queue sizes
    constexpr uint8_t QUEUE_CAN_RX         = 10;  // Incoming CAN messages
    constexpr uint8_t QUEUE_INPUT          = 5;   // Input events
    constexpr uint8_t QUEUE_EVENTS         = 10;  // State machine events
}

//-----------------------------------------------------------------------------
// WIFI CONFIGURATION
//-----------------------------------------------------------------------------
namespace WiFi_Config {
    // Access Point (Field Use)
    constexpr const char* AP_SSID          = "E-GoCart-VCU";
    constexpr const char* AP_PASSWORD      = "egocart123";  // Min 8 chars
    constexpr uint8_t     AP_CHANNEL       = 6;
    constexpr uint8_t     AP_MAX_CLIENTS   = 4;

    // Station Mode (Workshop) - stored in LittleFS, these are defaults
    constexpr const char* STA_SSID         = "OBR-INTERN";
    constexpr const char* STA_PASSWORD     = "";

    // Network Settings
    constexpr const char* AP_IP            = "192.168.4.1";
    constexpr const char* AP_GATEWAY       = "192.168.4.1";
    constexpr const char* AP_SUBNET        = "255.255.255.0";

    // Webserver
    constexpr uint16_t    HTTP_PORT        = 80;
    constexpr uint32_t    WEBSOCKET_UPDATE = 50;  // ms between updates

    // OTA Update
    constexpr const char* OTA_HOSTNAME     = "egocart-vcu";
    constexpr const char* OTA_PASSWORD     = "egocart123";
    constexpr uint16_t    OTA_PORT         = 3232;
}

//-----------------------------------------------------------------------------
// RUNTIME CONFIGURATION (Web-Configurable, stored in LittleFS)
//-----------------------------------------------------------------------------
namespace RuntimeConfig {
    // CAN Timing (configurable 10-50ms)
    constexpr uint32_t CAN_FAST_MIN        = 10;
    constexpr uint32_t CAN_FAST_MAX        = 50;
    constexpr uint32_t CAN_FAST_DEFAULT    = 10;   // Default to 10ms

    // Motor Limits (configurable via web)
    constexpr int16_t TORQUE_MAX_DEFAULT   = Motor::MAX_TORQUE_NM;
    constexpr int16_t REGEN_MAX_DEFAULT    = Motor::MAX_REGEN_NM;
    constexpr int16_t REVERSE_MAX_DEFAULT  = Motor::MAX_REVERSE_NM;
}

//-----------------------------------------------------------------------------
// DEBUG OPTIONS
//-----------------------------------------------------------------------------
#define DEBUG_SERIAL        1  // Enable Serial debug output
#define DEBUG_CAN_MESSAGES  0  // Log all CAN messages
#define DEBUG_STATE_MACHINE 1  // Log state transitions
#define DEBUG_FREERTOS      0  // Log FreeRTOS task info
#define DEBUG_WIFI          0  // Log WiFi events
#define DEBUG_WEBSERVER     0  // Log web requests
#define UPLOAD_BMS_CONFIG   0  // UPLOAD BMS param over CAN
// Debug Mode (prevents sleep, keeps WiFi always on)
#define DEBUG_MODE          0  // 1 = Debug mode, 0 = Production mode

// Hardware-less Test Mode (bypass hardware init, simulate telemetry)
// Set to 1 to test webserver without physical hardware connected
#define HARDWARE_TEST_MODE  0  // 1 = Simulated data, 0 = Real hardware

// CAN2 (BMS) - Set to 0 if CAN2 hardware not yet connected
#define ENABLE_CAN2         1  // 1 = CAN2 enabled, 0 = CAN2 disabled (optional)

#if DEBUG_SERIAL
    #define DEBUG_PRINT(x)   Serial.print(x)
    #define DEBUG_PRINTLN(x) Serial.println(x)
    #define DEBUG_PRINTF(...)  Serial.printf(__VA_ARGS__)
#else
    #define DEBUG_PRINT(x)
    #define DEBUG_PRINTLN(x)
    #define DEBUG_PRINTF(...)
#endif