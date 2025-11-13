#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/event_groups.h>
#include "driver/rtc_io.h"
#include "config.h"
#include "data_structures.h"
#include "input_manager.h"
#include "state_manager.h"
#include "vehicle_control.h"
#include "can_manager.h"
#include "bms_manager.h"
#include "nlg5_manager.h"
#include "contactor_manager.h"
#include "display_manager.h"
#include "wifi_manager.h"
#include "web_server.h"
#include "task_monitor.h"

//=============================================================================
// DEEP SLEEP CONFIGURATION
//=============================================================================
#define BUTTON_PIN_BITMASK(GPIO) (1ULL << GPIO)  // 2 ^ GPIO_NUMBER in hex
#define WAKEUP_GPIO_START        GPIO_NUM_7      // START/STOP button
#define WAKEUP_GPIO_CHARGER      GPIO_NUM_9      // Charger wakeup

// Define bitmask for multiple GPIOs
uint64_t wakeup_bitmask = BUTTON_PIN_BITMASK(WAKEUP_GPIO_START) | BUTTON_PIN_BITMASK(WAKEUP_GPIO_CHARGER);

//=============================================================================
// GLOBAL OBJECTS
//=============================================================================
InputManager inputManager;
StateManager stateManager;
VehicleControl vehicleControl;
CANManager canManager;
BMSManager bmsManager;
NLG5Manager nlg5Manager;
ContactorManager contactorManager;
DisplayManager displayManager;
WiFiManager wifiManager;
WebServer webServer;
TaskMonitor taskMonitor;

//=============================================================================
// FREERTOS HANDLES
//=============================================================================
TaskHandle_t taskHandleCANRx = NULL;
TaskHandle_t taskHandleCANTx = NULL;
TaskHandle_t taskHandleVehicle = NULL;
TaskHandle_t taskHandleState = NULL;
TaskHandle_t taskHandleSafety = NULL;
TaskHandle_t taskHandleInput = NULL;
TaskHandle_t taskHandleDisplay = NULL;
TaskHandle_t taskHandleWiFi = NULL;
TaskHandle_t taskHandleWeb = NULL;
TaskHandle_t taskHandleMonitor = NULL;
TaskHandle_t taskHandleLED = NULL;  // LED control task
TaskHandle_t taskHandleErrorAggregator = NULL;  // Error reporting task

#if HARDWARE_TEST_MODE
TaskHandle_t taskHandleSimData = NULL;  // Simulated data generator
#endif

//=============================================================================
// SHARED DATA INSTANCES (defined here, extern in data_structures.h)
//=============================================================================
QueueHandle_t canRxQueue;
QueueHandle_t inputEventQueue;
EventGroupHandle_t systemEvents;

ThreadSafeData<BMSData> sharedBMSData;
ThreadSafeData<DMCData> sharedDMCData;
ThreadSafeData<NLGData> sharedNLGData;
ThreadSafeData<VehicleTelemetry> sharedTelemetry;
ThreadSafeData<RuntimeConfigData> sharedRuntimeConfig;
ThreadSafeData<SystemErrorStatus> sharedErrorStatus;

//=============================================================================
// ERROR REPORTING FUNCTIONS
//=============================================================================

/**
 * @brief Populate VCU-specific errors into SystemErrorStatus
 * @param status SystemErrorStatus structure to populate
 *
 * VCU errors include:
 * - Contactor sequencing errors
 * - Subsystem communication timeouts (BMS, DMC, NLG)
 * - Emergency stop conditions
 * - Safety limit violations
 */
void populateVCUErrors(SystemErrorStatus& status) {
    unsigned long now = millis();

    // Contactor errors
    ContactorError contactorErr = contactorManager.getError();
    status.vcuContactorError.active = (contactorErr != ContactorError::NONE);
    if (status.vcuContactorError.active) {
        status.vcuContactorError.severity = ErrorSeverity::CRITICAL;
        if (status.vcuContactorError.timestamp == 0) status.vcuContactorError.timestamp = now;

        switch (contactorErr) {
            case ContactorError::PRECHARGE_TIMEOUT:
                status.vcuContactorError.code = "VCU_CONTACTOR_PRECHARGE";
                status.vcuContactorError.message = "Contactor precharge timeout";
                break;
            case ContactorError::CHARGE_ALLOW_VIOLATED:
                status.vcuContactorError.code = "VCU_CHARGE_ALLOW";
                status.vcuContactorError.message = "Charge allowance violated during sequence";
                break;
            case ContactorError::DISCHARGE_ALLOW_VIOLATED:
                status.vcuContactorError.code = "VCU_DISCHARGE_ALLOW";
                status.vcuContactorError.message = "Discharge allowance violated during sequence";
                break;
            case ContactorError::CURRENT_NOT_ZERO:
                status.vcuContactorError.code = "VCU_CURRENT_NOT_ZERO";
                status.vcuContactorError.message = "Battery current not zero during contactor operation";
                break;
            case ContactorError::BMS_NOT_ARMED:
                status.vcuContactorError.code = "VCU_BMS_NOT_ARMED";
                status.vcuContactorError.message = "BMS failed to arm during sequence";
                break;
            default:
                status.vcuContactorError.code = "VCU_CONTACTOR_UNKNOWN";
                status.vcuContactorError.message = "Unknown contactor error";
                break;
        }
    }

    // BMS communication timeout
    status.bmsTimeout.active = !canManager.isBMSAlive();
    if (status.bmsTimeout.active) {
        status.bmsTimeout.code = "VCU_BMS_TIMEOUT";
        status.bmsTimeout.message = "BMS CAN communication timeout";
        status.bmsTimeout.severity = ErrorSeverity::CRITICAL;
        if (status.bmsTimeout.timestamp == 0) status.bmsTimeout.timestamp = now;
    }

    // DMC communication timeout (check if DMC ready flag is false)
    DMCData dmc = sharedDMCData.get();
    status.dmcTimeout.active = !canManager.isDMCReady();
    if (status.dmcTimeout.active) {
        status.dmcTimeout.code = "VCU_DMC_TIMEOUT";
        status.dmcTimeout.message = "DMC CAN communication timeout";
        status.dmcTimeout.severity = ErrorSeverity::CRITICAL;
        if (status.dmcTimeout.timestamp == 0) status.dmcTimeout.timestamp = now;
    }

    // NLG timeout (check if NLG data is valid)
    status.chargerTimeout.active = !nlg5Manager.isAlive();
    if (status.chargerTimeout.active) {
        status.chargerTimeout.code = "VCU_NLG_TIMEOUT";
        status.chargerTimeout.message = "Charger CAN communication timeout";
        status.chargerTimeout.severity = ErrorSeverity::CRITICAL;
        if (status.chargerTimeout.timestamp == 0) status.chargerTimeout.timestamp = now;
    }

    // Emergency stop / Interlock open
    status.vcuInterlock.active = !inputManager.isILClosed();
    if (status.vcuInterlock.active) {
        status.vcuInterlock.code = "VCU_INTERLOCK";
        status.vcuInterlock.message = "Safety interlock open";
        status.vcuInterlock.severity = ErrorSeverity::CRITICAL;
        if (status.vcuInterlock.timestamp == 0) status.vcuInterlock.timestamp = now;
    }

    // Pack voltage safety limits
    BMSData bms = sharedBMSData.get();
    float packVoltage = bms.voltage;  // Already in volts
    status.vcuLowVoltage.active = (packVoltage < Battery::MIN_VOLTAGE);
    if (status.vcuLowVoltage.active) {
        status.vcuLowVoltage.code = "VCU_LOW_VOLTAGE";
        status.vcuLowVoltage.message = "Pack voltage below safety limit";
        status.vcuLowVoltage.severity = ErrorSeverity::CRITICAL;
        if (status.vcuLowVoltage.timestamp == 0) status.vcuLowVoltage.timestamp = now;
    }

    // Battery overtemperature safety limit
    status.vcuBatteryOvertemp.active = (bms.temperature > Battery::MAX_TEMP_C);
    if (status.vcuBatteryOvertemp.active) {
        status.vcuBatteryOvertemp.code = "VCU_BATTERY_OVERTEMP";
        status.vcuBatteryOvertemp.message = "Battery temperature exceeds safety limit";
        status.vcuBatteryOvertemp.severity = ErrorSeverity::CRITICAL;
        if (status.vcuBatteryOvertemp.timestamp == 0) status.vcuBatteryOvertemp.timestamp = now;
    }

    // Motor overtemperature
    status.vcuMotorOvertemp.active = (dmc.tempMotor > Safety::MAX_MOTOR_TEMP);
    if (status.vcuMotorOvertemp.active) {
        status.vcuMotorOvertemp.code = "VCU_MOTOR_OVERTEMP";
        status.vcuMotorOvertemp.message = "Motor temperature exceeds safety limit";
        status.vcuMotorOvertemp.severity = ErrorSeverity::CRITICAL;
        if (status.vcuMotorOvertemp.timestamp == 0) status.vcuMotorOvertemp.timestamp = now;
    }

    // Inverter overtemperature
    status.vcuInverterOvertemp.active = (dmc.tempInverter > Safety::MAX_INVERTER_TEMP);
    if (status.vcuInverterOvertemp.active) {
        status.vcuInverterOvertemp.code = "VCU_INVERTER_OVERTEMP";
        status.vcuInverterOvertemp.message = "Inverter temperature exceeds safety limit";
        status.vcuInverterOvertemp.severity = ErrorSeverity::CRITICAL;
        if (status.vcuInverterOvertemp.timestamp == 0) status.vcuInverterOvertemp.timestamp = now;
    }

    // Low SOC warning
    status.vcuLowSOC.active = (bms.soc < 10);  // Less than 10%
    if (status.vcuLowSOC.active) {
        status.vcuLowSOC.code = "VCU_LOW_SOC";
        status.vcuLowSOC.message = "Battery state of charge critically low";
        status.vcuLowSOC.severity = ErrorSeverity::WARNING;
        if (status.vcuLowSOC.timestamp == 0) status.vcuLowSOC.timestamp = now;
    }

    // Critical cell voltage
    status.vcuCellCritical.active = (bms.minCellVoltage < Safety::CRITICAL_CELL_VOLTAGE);
    if (status.vcuCellCritical.active) {
        status.vcuCellCritical.code = "VCU_CELL_CRITICAL";
        status.vcuCellCritical.message = "Cell voltage critically low";
        status.vcuCellCritical.severity = ErrorSeverity::CRITICAL;
        if (status.vcuCellCritical.timestamp == 0) status.vcuCellCritical.timestamp = now;
    }
}

//=============================================================================
// TASK FUNCTIONS
//=============================================================================

/**
 * CAN RX Task - Highest Priority
 * Processes incoming CAN messages from queue
 */
void taskCANRx(void* parameter) {
    DEBUG_PRINTLN("Task: CAN RX started");

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10);  // 10ms cycle

    TickType_t lastBMSUpdate = 0;
    const TickType_t bmsUpdateInterval = pdMS_TO_TICKS(100);  // Update BMS every 100ms

    while (1) {
        // Process all messages in queue
        canManager.processRxQueue();

        // Update BMS manager periodically (check timeouts, update shared data)
        TickType_t now = xTaskGetTickCount();
        if ((now - lastBMSUpdate) >= bmsUpdateInterval) {
            bmsManager.update();
            contactorManager.update();  // Update contactor state machine
            lastBMSUpdate = now;
        }

        // Feed watchdog
        FEED_WATCHDOG();

        // Wait for next cycle
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

/**
 * CAN TX Task - High Priority
 * Sends DMC control messages at configured rate
 */
void taskCANTx(void* parameter) {
    DEBUG_PRINTLN("Task: CAN TX started");

    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        // Get runtime config for cycle time
        RuntimeConfigData config = sharedRuntimeConfig.get();
        const TickType_t xFrequency = pdMS_TO_TICKS(config.canFastCycle);

        // Send DMC control if in DRIVE state
        if (stateManager.isDriving()) {
            int16_t torqueDemand = vehicleControl.getTorqueDemand();
            bool enableDMC = (torqueDemand != 0);
            canManager.sendDMCControl(torqueDemand, enableDMC);
        } else {
            // Send zero torque if not driving
            canManager.sendDMCControl(0, false);
        }

        // Send periodic messages (limits, etc.)
        canManager.sendPeriodicMessages();

        // Feed watchdog
        FEED_WATCHDOG();

        // Wait for next cycle
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

/**
 * Vehicle Control Task
 * Calculates torque demand based on inputs
 */
void taskVehicleControl(void* parameter) {
    DEBUG_PRINTLN("Task: Vehicle Control started");

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10);  // 10ms cycle

    while (1) {
        // Get inputs
        vehicleControl.setThrottle(inputManager.getThrottlePercent());
        vehicleControl.setRegen(inputManager.getRegenPercent());
        vehicleControl.setBrakePressed(inputManager.isBrakePressed());

        // Get motor speed from CAN
        DMCData dmcData = sharedDMCData.get();
        vehicleControl.setMotorSpeed(dmcData.speedActual);

        vehicleControl.setSystemReady(stateManager.isSystemReady());

        // Update vehicle control logic
        vehicleControl.update();

        // Feed watchdog
        FEED_WATCHDOG();

        // Wait for next cycle
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

/**
 * State Manager Task
 * Handles vehicle state machine
 */
void taskStateManager(void* parameter) {
    DEBUG_PRINTLN("Task: State Manager started");

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(50);  // 50ms cycle

    while (1) {
        // Update state machine
        stateManager.update();

        // Update cooling pump based on motor/inverter temperatures
        DMCData dmcData = sharedDMCData.get();
        stateManager.updateCoolingPump(dmcData.tempMotor, dmcData.tempInverter);

        // Update NLG5Manager with charge conditions
        bool chargeAllowed = inputManager.isChargeAllowed();
        bool batteryArmed = contactorManager.isChargeArmed();
        nlg5Manager.setChargeConditions(chargeAllowed, batteryArmed);

        // Update NLG5Manager (processes enable logic, error clearing, state machine)
        nlg5Manager.update();

        // Handle charging state - send NLG control with enable bit
        if (stateManager.isCharging()) {
            uint8_t chargerState = 1;  // 1 = charging enabled
            bool enableCharger = nlg5Manager.getEnableCharger();
            bool clearErrors = nlg5Manager.isErrorClearing();
            canManager.sendNLGControl(chargerState, enableCharger, clearErrors);
        }

        // Feed watchdog
        FEED_WATCHDOG();

        // Wait for next cycle
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

/**
 * Safety Monitor Task
 * Checks temperatures, voltages, and other safety conditions
 */
void taskSafetyMonitor(void* parameter) {
    DEBUG_PRINTLN("Task: Safety Monitor started");

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(100);  // 100ms cycle

    // Rate limiting for warnings (only print once per 5 seconds)
    static unsigned long lastWarningTime[5] = {0, 0, 0, 0, 0};
    const unsigned long WARNING_RATE_LIMIT = 5000;  // 5 seconds
    enum WarningType { MOTOR_TEMP = 0, INVERTER_TEMP = 1, VOLTAGE = 2, SOC = 3, CELL_CRITICAL = 4 };

    while (1) {
        BMSData bmsData = sharedBMSData.get();
        DMCData dmcData = sharedDMCData.get();
        RuntimeConfigData config = sharedRuntimeConfig.get();
        unsigned long currentTime = millis();

        // Check motor temperature
        if (dmcData.tempMotor > config.maxMotorTemp) {
            if (currentTime - lastWarningTime[MOTOR_TEMP] > WARNING_RATE_LIMIT) {
                DEBUG_PRINTLN("WARNING: Motor overtemperature!");
                displayManager.showWarning("MOTOR OVERHEAT");
                lastWarningTime[MOTOR_TEMP] = currentTime;
            }
        }

        // Check inverter temperature
        if (dmcData.tempInverter > config.maxInverterTemp) {
            if (currentTime - lastWarningTime[INVERTER_TEMP] > WARNING_RATE_LIMIT) {
                DEBUG_PRINTLN("WARNING: Inverter overtemperature!");
                displayManager.showWarning("INVERTER OVERHEAT");
                lastWarningTime[INVERTER_TEMP] = currentTime;
            }
        }

        // Check battery voltage
        if (bmsData.voltage < Battery::MIN_VOLTAGE) {
            if (currentTime - lastWarningTime[VOLTAGE] > WARNING_RATE_LIMIT) {
                DEBUG_PRINTLN("WARNING: Battery undervoltage!");
                displayManager.showWarning("LOW VOLTAGE");
                lastWarningTime[VOLTAGE] = currentTime;
            }
        }

        // Check SOC
        if (bmsData.soc < Battery::MIN_SOC) {
            if (currentTime - lastWarningTime[SOC] > WARNING_RATE_LIMIT) {
                DEBUG_PRINTLN("WARNING: Low battery!");
                displayManager.showWarning("LOW BATTERY");
                lastWarningTime[SOC] = currentTime;
            }
        }

        // Check cell voltage (critical - trigger emergency stop only when driving)
        if (bmsData.minCellVoltage < config.criticalCellVoltage && stateManager.isDriving()) {
            if (currentTime - lastWarningTime[CELL_CRITICAL] > WARNING_RATE_LIMIT) {
                DEBUG_PRINTLN("CRITICAL: Cell voltage too low!");
                displayManager.showWarning("CELL CRITICAL");
                lastWarningTime[CELL_CRITICAL] = currentTime;
            }
            vehicleControl.emergencyStop();
            xEventGroupSetBits(systemEvents, EVENT_EMERGENCY_STOP);
        }

        // Check if BMS is still alive
        if (!canManager.isBMSAlive() && stateManager.isDriving()) {
            DEBUG_PRINTLN("ERROR: BMS communication lost!");
            displayManager.showWarning("BMS OFFLINE");
            vehicleControl.emergencyStop();
            xEventGroupSetBits(systemEvents, EVENT_EMERGENCY_STOP);
        }

        // Feed watchdog
        FEED_WATCHDOG();

        // Wait for next cycle
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

/**
 * Input Manager Task
 * Reads all inputs and handles button events
 */
void taskInputManager(void* parameter) {
    DEBUG_PRINTLN("Task: Input Manager started");

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(20);  // 20ms cycle

    static unsigned long stopButtonHoldStart = 0;
    static bool stopButtonHeld = false;

    while (1) {
        // Update all inputs
        inputManager.update();

        // Handle START/STOP button
        // - 2 second hold: toggle start/stop
        // - 15 second hold: restart ESP32
        static bool startStopButtonHeld = false;
        static unsigned long startStopHoldStart = 0;
        // Handle Start/Stop button (short press ignored, 0.7s = start/stop, 4s = reset)
        // Logic: Press starts timer, Release evaluates duration and performs action
        const unsigned long START_STOP_HOLD_TIME = 700;    // 0.7 seconds
        const unsigned long RESET_HOLD_TIME = 2000;        // 4 seconds

        bool startStopPressedNow = inputManager.isStartStopPressed();

        // Detect button press (transition from not pressed to pressed)
        if (startStopPressedNow && !startStopButtonHeld) {
            // Button just pressed - start timer, don't do anything yet
            startStopButtonHeld = true;
            startStopHoldStart = millis();
            DEBUG_PRINTLN("Start/Stop button pressed - waiting for release");
        }

        // Detect button release (transition from pressed to not pressed)
        if (!startStopPressedNow && startStopButtonHeld) {
            // Button just released - evaluate how long it was held
            unsigned long holdDuration = millis() - startStopHoldStart;

            if (holdDuration >= RESET_HOLD_TIME) {
                // Long press (4+ seconds) - Reset/Restart ESP32
                DEBUG_PRINTLN("Start/Stop button held 4s - Restarting ESP32...");
                vTaskDelay(pdMS_TO_TICKS(100));
                ESP.restart();
            } else if (holdDuration >= START_STOP_HOLD_TIME) {
                // Medium press (0.7-4 seconds) - Stop system if in DRIVE + NEUTRAL
                VehicleState currentState = stateManager.getCurrentState();
                GearState currentGear = vehicleControl.getCurrentGear();

                if (currentState == VehicleState::DRIVE && currentGear == GearState::NEUTRAL) {
                    DEBUG_PRINTLN("Stop request from DRIVE/NEUTRAL");
                    stateManager.handleStopRequest();
                }
                // Ignore in other states (SLEEP/INIT handled by wakeup, CHARGING ignores button)
            }
            // Short press (< 0.7 seconds) - do nothing

            startStopButtonHeld = false;
        }

        // Monitor interlock
        static bool ilWasClosed = true;
        bool ilClosed = inputManager.isILClosed();

        if (ilWasClosed && !ilClosed) {
            DEBUG_PRINTLN("INTERLOCK OPEN!");
            stateManager.handleEmergencyShutdown();
            xEventGroupClearBits(systemEvents, EVENT_IL_CLOSED);
        } else if (!ilWasClosed && ilClosed) {
            xEventGroupSetBits(systemEvents, EVENT_IL_CLOSED);
        }
        ilWasClosed = ilClosed;

        // Monitor charger pin (sends 12V when connected)
        static bool chargerWasConnected = false;
        bool chargerConnected = inputManager.isChargerConnected();

        if (chargerConnected != chargerWasConnected) {
            stateManager.handleChargerPinChange(chargerConnected);
            if (chargerConnected) {
                xEventGroupSetBits(systemEvents, EVENT_CHARGER_CONNECTED);
            } else {
                xEventGroupClearBits(systemEvents, EVENT_CHARGER_CONNECTED);
            }
            chargerWasConnected = chargerConnected;
        }

        // Handle direction toggle (short press = toggle D/R, 0.7s hold = Neutral)
        // IGNORE in CHARGING state - buttons disabled during charging
        // Logic: Press starts timer, Release evaluates duration and performs action
        VehicleState currentState = stateManager.getCurrentState();

        if (currentState != VehicleState::CHARGING) {
            static bool directionButtonHeld = false;
            static unsigned long directionPressTime = 0;
            const unsigned long NEUTRAL_HOLD_TIME = 700;   // 0.7 seconds

            // Get raw button state
            bool directionHeldNow = inputManager.isDirectionToggleHeld();

        // Detect button press (transition from not held to held)
        if (directionHeldNow && !directionButtonHeld) {
            // Button just pressed - start timer, don't do anything yet
            directionButtonHeld = true;
            directionPressTime = millis();
            DEBUG_PRINTLN("Direction button pressed - waiting for release");
        }

        // Detect button release (transition from held to not held)
        if (!directionHeldNow && directionButtonHeld) {
            // Button just released - evaluate how long it was held
            unsigned long holdDuration = millis() - directionPressTime;

            if (holdDuration >= NEUTRAL_HOLD_TIME) {
                // Long press (0.7+ seconds) - set to Neutral
                DEBUG_PRINTLN("Direction button held 0.7s - Setting to NEUTRAL");
                vehicleControl.setNeutral();
            } else {
                // Short press - toggle between D and R
                DEBUG_PRINTLN("Direction button short press - switching D/R");
                vehicleControl.handleDirectionToggle();
            }

            directionButtonHeld = false;
        }
        }  // End if (currentState != VehicleState::CHARGING)

        // Feed watchdog
        FEED_WATCHDOG();

        // Wait for next cycle
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

/**
 * Display Manager Task
 * Updates Nextion display
 */
void taskDisplayManager(void* parameter) {
    DEBUG_PRINTLN("Task: Display Manager started");

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(50);  // 50ms cycle

    while (1) {
        BMSData bmsData = sharedBMSData.get();
        DMCData dmcData = sharedDMCData.get();

        // Calculate speed from motor RPM
        float motorSpeed = abs(dmcData.speedActual);
        float wheelSpeed = motorSpeed / Motor::GEAR_RATIO;
        float speedKmh = (wheelSpeed * Motor::WHEEL_DIAMETER_M * 3.14159f * 60.0f) / 1000.0f;

        // Calculate power (kW)
        float powerKw = (dmcData.dcVoltage * dmcData.dcCurrent) / 1000.0f;

        // Update display values
        displayManager.setSpeed((int)speedKmh);
        displayManager.setPower((int)powerKw);
        displayManager.setSOC(bmsData.soc);
        displayManager.setVoltage(bmsData.voltage);
        displayManager.setGear(vehicleControl.getCurrentGear());
        displayManager.setTempMotor(dmcData.tempMotor);
        displayManager.setTempInverter(dmcData.tempInverter);
        displayManager.setTempBattery(bmsData.temperature);
        displayManager.setMinCellVoltage(bmsData.minCellVoltage);

        // Update display
        displayManager.update();

        // Feed watchdog
        FEED_WATCHDOG();

        // Wait for next cycle
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

/**
 * WiFi Manager Task
 * Handles WiFi connection and reconnection
 */
void taskWiFiManager(void* parameter) {
    DEBUG_PRINTLN("Task: WiFi Manager started");

    while (1) {
        // Update WiFi manager
        wifiManager.update();

        // Feed watchdog
        FEED_WATCHDOG();

        // Longer delay - WiFi doesn't need frequent updates
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/**
 * Web Server Task
 * Handles HTTP/WebSocket requests and telemetry broadcasting
 */
void taskWebServer(void* parameter) {
    DEBUG_PRINTLN("Task: Web Server started");

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(50);  // 50ms for WebSocket updates

    while (1) {
        // Update telemetry structure for webserver
        VehicleTelemetry telem;
        BMSData bmsData = sharedBMSData.get();
        DMCData dmcData = sharedDMCData.get();

        telem.state = stateManager.getCurrentState();
        telem.gear = vehicleControl.getCurrentGear();
        telem.systemReady = stateManager.isSystemReady();

        // Calculate speed
        float motorSpeed = abs(dmcData.speedActual);
        float wheelSpeed = motorSpeed / Motor::GEAR_RATIO;
        telem.speedKmh = (wheelSpeed * Motor::WHEEL_DIAMETER_M * 3.14159f * 60.0f) / 1000.0f;

        telem.torqueDemand = vehicleControl.getTorqueDemand();
        telem.torqueActual = dmcData.torqueActual;
        telem.motorRPM = dmcData.speedActual;

        telem.soc = bmsData.soc;
        telem.voltage = bmsData.voltage;
        telem.current = bmsData.current;
        telem.powerKw = (dmcData.dcVoltage * dmcData.dcCurrent) / 1000.0f;
        telem.minCellVoltage = bmsData.minCellVoltage;
        telem.maxCellVoltage = bmsData.maxCellVoltage;

        telem.tempMotor = dmcData.tempMotor;
        telem.tempInverter = dmcData.tempInverter;
        telem.tempBattery = bmsData.temperature;

#if HARDWARE_TEST_MODE
        // Simulated input data
        telem.throttlePercent = (dmcData.torqueActual > 0) ? random(30, 80) : 0;
        telem.regenPercent = (dmcData.torqueActual < 0) ? random(20, 60) : 0;
        telem.throttleRawADC = random(0, 26400);
        telem.regenRawADC = random(0, 26400);
        telem.brakePressed = (dmcData.torqueActual < 0);
        telem.ilClosed = true;
        telem.bmsAlive = true;
        telem.dmcReady = true;
        telem.chargerConnected = false;
#else
        // Real hardware data
        telem.throttlePercent = inputManager.getThrottlePercent();
        telem.regenPercent = inputManager.getRegenPercent();
        telem.throttleRawADC = inputManager.getThrottleRawADC();
        telem.regenRawADC = inputManager.getRegenRawADC();
        telem.brakePressed = inputManager.isBrakePressed();
        telem.ilClosed = inputManager.isILClosed();
        telem.bmsAlive = canManager.isBMSAlive();
        telem.dmcReady = canManager.isDMCReady();
        telem.chargerConnected = inputManager.isChargerConnected();
#endif

        telem.timestamp = millis();

        // Update shared telemetry
        sharedTelemetry.set(telem);

        // Update webserver
        webServer.update();

        // Feed watchdog
        FEED_WATCHDOG();

        // Wait for next cycle
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

/**
 * Task Monitor
 * Monitors all tasks, restarts failed tasks, collects statistics
 */
void taskTaskMonitor(void* parameter) {
    DEBUG_PRINTLN("Task: Task Monitor started");

    while (1) {
        // Update task monitor (checks watchdogs, updates stats)
        taskMonitor.update();

        // Longer delay - monitoring doesn't need frequent updates
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/**
 * LED Control Task
 * Controls status and direction LEDs with different blink patterns
 */
void taskLEDControl(void* parameter) {
    DEBUG_PRINTLN("Task: LED Control started");

    // Initialize LED pins
    pinMode(Pins::STATUS_LED, OUTPUT);
    pinMode(Pins::DIRECTION_LED, OUTPUT);
    digitalWrite(Pins::STATUS_LED, LOW);
    digitalWrite(Pins::DIRECTION_LED, LOW);

    unsigned long lastStatusToggle = 0;
    unsigned long lastDirectionToggle = 0;
    bool statusLedState = false;
    bool directionLedState = false;

    while (1) {
        unsigned long currentTime = millis();
        VehicleState currentState = stateManager.getCurrentState();
        GearState currentGear = vehicleControl.getCurrentGear();
        EventBits_t eventBits = xEventGroupGetBits(systemEvents);

        // Check for errors
        bool hasEmergencyStop = (eventBits & EVENT_EMERGENCY_STOP) != 0;
        bool bmsAlive = (eventBits & EVENT_BMS_ALIVE) != 0;
        bool lowBattery = false;
        bool hasContactorError = contactorManager.hasError();

        // Check for low battery
        BMSData bmsData = sharedBMSData.get();
        if (bmsData.soc <= Battery::MIN_SOC) {
            lowBattery = true;
        }

        // Check for any error condition
        bool hasError = hasEmergencyStop || !bmsAlive || lowBattery || hasContactorError;

        // ===== ERROR STATE: 20Hz FLASH ON BOTH LEDS =====
        if (hasError) {
            // 20Hz = 50ms period = 25ms on, 25ms off
            if (currentTime - lastStatusToggle >= 25) {
                statusLedState = !statusLedState;
                digitalWrite(Pins::STATUS_LED, statusLedState ? HIGH : LOW);
                digitalWrite(Pins::DIRECTION_LED, statusLedState ? HIGH : LOW);  // Both LEDs in sync
                lastStatusToggle = currentTime;
                lastDirectionToggle = currentTime;
            }
        }
        // ===== NORMAL OPERATION =====
        else if (currentState == VehicleState::INIT || currentState == VehicleState::READY) {
            // Slow blink (1Hz: 500ms on, 500ms off)
            if (currentTime - lastStatusToggle >= 500) {
                statusLedState = !statusLedState;
                digitalWrite(Pins::STATUS_LED, statusLedState ? HIGH : LOW);
                lastStatusToggle = currentTime;
            }

        } else if (currentState == VehicleState::DRIVE) {
            // Solid ON
            digitalWrite(Pins::STATUS_LED, HIGH);

        } else if (currentState == VehicleState::CHARGING) {
            // Alternating blink with direction LED (2Hz: 250ms on, 250ms off)
            if (currentTime - lastStatusToggle >= 250) {
                statusLedState = !statusLedState;
                digitalWrite(Pins::STATUS_LED, statusLedState ? HIGH : LOW);
                lastStatusToggle = currentTime;
            }

        } else {
            // SLEEP: OFF
            digitalWrite(Pins::STATUS_LED, LOW);
        }

        // ===== DIRECTION BUTTON LED =====
        if (currentState == VehicleState::INIT || currentState == VehicleState::READY) {
            // Slow blink (1Hz: 500ms on, 500ms off)
            if (currentTime - lastDirectionToggle >= 500) {
                directionLedState = !directionLedState;
                digitalWrite(Pins::DIRECTION_LED, directionLedState ? HIGH : LOW);
                lastDirectionToggle = currentTime;
            }

        } else if (currentState == VehicleState::DRIVE) {
            // Gear-based logic
            if (currentGear == GearState::DRIVE) {
                digitalWrite(Pins::DIRECTION_LED, HIGH);  // Solid ON
            } else if (currentGear == GearState::REVERSE) {
                // 2Hz blink (500ms period = 250ms on, 250ms off)
                if (currentTime - lastDirectionToggle >= 250) {
                    directionLedState = !directionLedState;
                    digitalWrite(Pins::DIRECTION_LED, directionLedState ? HIGH : LOW);
                    lastDirectionToggle = currentTime;
                }
            } else {
                digitalWrite(Pins::DIRECTION_LED, LOW);  // OFF in NEUTRAL
            }

        } else if (currentState == VehicleState::CHARGING) {
            // Alternating blink (opposite of start LED)
            if (currentTime - lastDirectionToggle >= 250) {
                digitalWrite(Pins::DIRECTION_LED, !statusLedState ? HIGH : LOW);  // Opposite of start LED
                lastDirectionToggle = currentTime;
            }

        } else {
            // SLEEP: OFF
            digitalWrite(Pins::DIRECTION_LED, LOW);
        }

        // Update at 50Hz (20ms) for smooth 20Hz error blink
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

//=============================================================================
// ERROR AGGREGATOR TASK
//=============================================================================

/**
 * Error Aggregator Task - Collects errors from all subsystems
 * Runs at 1Hz and populates the shared error status
 */
void taskErrorAggregator(void* parameter) {
    DEBUG_PRINTLN("Task: Error Aggregator started");

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000);  // 1Hz (every 1 second)

    while (1) {
        // Get local copy of error status
        SystemErrorStatus errorStatus = sharedErrorStatus.get();

        // Clear summary counters
        errorStatus.criticalCount = 0;
        errorStatus.warningCount = 0;
        errorStatus.infoCount = 0;
        errorStatus.hasAnyError = false;

        // Collect errors from all subsystems
        bmsManager.populateErrorStatus(errorStatus);
        nlg5Manager.populateErrorStatus(errorStatus);
        canManager.populateDMCErrorStatus(errorStatus);
        populateVCUErrors(errorStatus);

        // Count active errors by severity
        // BMS errors (9 total)
        if (errorStatus.bmsOverTemp.active) errorStatus.criticalCount++;
        if (errorStatus.bmsOverCharge.active) errorStatus.criticalCount++;
        if (errorStatus.bmsCellError.active) errorStatus.criticalCount++;
        if (errorStatus.bmsOverCurrent.active) errorStatus.criticalCount++;
        if (errorStatus.bmsOverDischarge.active) errorStatus.criticalCount++;
        if (errorStatus.bmsTempNegative.active) errorStatus.warningCount++;
        if (errorStatus.bmsTimeout.active) errorStatus.criticalCount++;
        if (errorStatus.bmsChargeMOSDisabled.active) errorStatus.criticalCount++;
        if (errorStatus.bmsDischargeMOSDisabled.active) errorStatus.criticalCount++;

        // Charger errors (11 critical + 4 warnings)
        if (errorStatus.chargerMainsFuse.active) errorStatus.criticalCount++;
        if (errorStatus.chargerOutputFuse.active) errorStatus.criticalCount++;
        if (errorStatus.chargerShortCircuit.active) errorStatus.criticalCount++;
        if (errorStatus.chargerMainsOV.active) errorStatus.criticalCount++;
        if (errorStatus.chargerBatteryOV.active) errorStatus.criticalCount++;
        if (errorStatus.chargerPolarity.active) errorStatus.criticalCount++;
        if (errorStatus.chargerCANTimeout.active) errorStatus.criticalCount++;
        if (errorStatus.chargerCANOff.active) errorStatus.criticalCount++;
        if (errorStatus.chargerTempSensor.active) errorStatus.criticalCount++;
        if (errorStatus.chargerCRCError.active) errorStatus.criticalCount++;
        if (errorStatus.chargerTimeout.active) errorStatus.criticalCount++;
        if (errorStatus.chargerLowMainsV.active) errorStatus.warningCount++;
        if (errorStatus.chargerLowBattV.active) errorStatus.warningCount++;
        if (errorStatus.chargerHighTemp.active) errorStatus.warningCount++;
        if (errorStatus.chargerControlOOR.active) errorStatus.warningCount++;

        // DMC errors (17 critical + 3 warnings)
        if (errorStatus.dmcCANTimeout.active) errorStatus.criticalCount++;
        if (errorStatus.dmcInverterOvertemp.active) errorStatus.criticalCount++;
        if (errorStatus.dmcMotorOvertemp.active) errorStatus.criticalCount++;
        if (errorStatus.dmcSpeedSensor.active) errorStatus.criticalCount++;
        if (errorStatus.dmcUndervoltage.active) errorStatus.criticalCount++;
        if (errorStatus.dmcOvervoltage.active) errorStatus.criticalCount++;
        if (errorStatus.dmcDCCurrentError.active) errorStatus.criticalCount++;
        if (errorStatus.dmcInitError.active) errorStatus.criticalCount++;
        if (errorStatus.dmcShortCircuit.active) errorStatus.criticalCount++;
        if (errorStatus.dmcACOvercurrent.active) errorStatus.criticalCount++;
        if (errorStatus.dmcTimeout.active) errorStatus.criticalCount++;
        if (errorStatus.dmcSpeedSensorSupply.active) errorStatus.criticalCount++;
        if (errorStatus.dmcLimitsInvalid.active) errorStatus.criticalCount++;
        if (errorStatus.dmcControlInvalid.active) errorStatus.criticalCount++;
        if (errorStatus.dmcVoltageMeas.active) errorStatus.criticalCount++;
        if (errorStatus.dmcEEPROMError.active) errorStatus.criticalCount++;
        if (errorStatus.dmcStorageError.active) errorStatus.criticalCount++;
        if (errorStatus.dmcGeneralWarning.active) errorStatus.warningCount++;
        if (errorStatus.dmcHVUndervoltage.active) errorStatus.warningCount++;
        if (errorStatus.dmcTempSensorWarning.active) errorStatus.warningCount++;

        // VCU errors (9 critical + 1 warning)
        if (errorStatus.vcuContactorError.active) errorStatus.criticalCount++;
        if (errorStatus.vcuEmergencyStop.active) errorStatus.criticalCount++;
        if (errorStatus.vcuPrechargeTimeout.active) errorStatus.criticalCount++;
        if (errorStatus.vcuMotorOvertemp.active) errorStatus.criticalCount++;
        if (errorStatus.vcuInverterOvertemp.active) errorStatus.criticalCount++;
        if (errorStatus.vcuBatteryOvertemp.active) errorStatus.criticalCount++;
        if (errorStatus.vcuLowVoltage.active) errorStatus.criticalCount++;
        if (errorStatus.vcuCellCritical.active) errorStatus.criticalCount++;
        if (errorStatus.vcuInterlock.active) errorStatus.criticalCount++;
        if (errorStatus.vcuLowSOC.active) errorStatus.warningCount++;

        // Set hasAnyError flag
        errorStatus.hasAnyError = (errorStatus.criticalCount > 0 ||
                                    errorStatus.warningCount > 0 ||
                                    errorStatus.infoCount > 0);

        // Write back to shared data
        sharedErrorStatus.set(errorStatus);

        // Wait for next cycle
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

#if HARDWARE_TEST_MODE
//=============================================================================
// SIMULATED DATA (For hardware-less testing)
//=============================================================================

/**
 * Initialize simulated data with realistic values
 */
void initializeSimulatedData() {
    DEBUG_PRINTLN("Initializing simulated telemetry data...");

    // Set initial BMS data
    BMSData bmsData;
    bmsData.soc = 75;  // 75% battery
    bmsData.voltage = 360;  // 360V
    bmsData.current = 0;  // No current initially
    bmsData.maxDischarge = 200;  // 200A max discharge
    bmsData.maxCharge = 50;  // 50A max charge
    bmsData.minCellVoltage = 3.5f;
    bmsData.maxCellVoltage = 3.7f;
    bmsData.temperature = 25.0f;  // 25°C
    sharedBMSData.set(bmsData);

    // Set initial DMC data
    DMCData dmcData;
    dmcData.ready = true;
    dmcData.running = false;
    dmcData.speedActual = 0.0f;
    dmcData.torqueActual = 0.0f;
    dmcData.dcVoltage = 360.0f;
    dmcData.dcCurrent = 0.0f;
    dmcData.tempInverter = 30.0f;
    dmcData.tempMotor = 28.0f;
    sharedDMCData.set(dmcData);

    // Set initial NLG data
    NLGData nlgData;
    nlgData.state = 0;  // Not charging
    nlgData.dcVoltage = 0;
    nlgData.dcCurrent = 0;
    nlgData.connectorLocked = false;
    nlgData.temperature = 25.0f;
    sharedNLGData.set(nlgData);

    DEBUG_PRINTLN("Simulated data initialized");
}

/**
 * Task to update simulated data (replaces CAN, Input, etc.)
 * Generates realistic changing values for testing dashboard
 */
void taskSimulatedData(void* parameter) {
    DEBUG_PRINTLN("Task: Simulated Data Generator started");

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(100);  // Update every 100ms

    float simulatedSpeed = 0.0f;
    float simulatedTorque = 0.0f;
    float speedTarget = 0.0f;
    bool accelerating = true;
    unsigned long lastSpeedChange = 0;

    while (1) {
        // Change speed target every 5 seconds
        if (millis() - lastSpeedChange > 5000) {
            if (accelerating) {
                speedTarget = random(1000, 3000);  // Random RPM between 1000-3000
            } else {
                speedTarget = random(0, 500);  // Slow down
            }
            accelerating = !accelerating;
            lastSpeedChange = millis();
        }

        // Smoothly approach target speed
        if (simulatedSpeed < speedTarget) {
            simulatedSpeed += 10.0f;
        } else if (simulatedSpeed > speedTarget) {
            simulatedSpeed -= 15.0f;
        }

        if (simulatedSpeed < 0) simulatedSpeed = 0;

        // Calculate torque based on speed change
        if (simulatedSpeed < speedTarget) {
            simulatedTorque = random(50, 150);  // Accelerating
        } else if (simulatedSpeed > speedTarget) {
            simulatedTorque = random(-50, -20);  // Regen braking
        } else {
            simulatedTorque = random(-10, 10);  // Coasting
        }

        // Update BMS data
        BMSData bmsData = sharedBMSData.get();
        bmsData.current = (simulatedTorque > 0) ? random(50, 150) : random(-30, 0);
        bmsData.minCellVoltage = 3.5f + (random(0, 20) / 100.0f);
        bmsData.maxCellVoltage = 3.6f + (random(0, 20) / 100.0f);
        bmsData.temperature = 25.0f + (abs(bmsData.current) / 10.0f);  // Temp rises with current

        // Slowly discharge battery
        static unsigned long lastSocUpdate = 0;
        if (millis() - lastSocUpdate > 10000 && bmsData.soc > 20) {
            bmsData.soc -= 1;
            lastSocUpdate = millis();
        }
        sharedBMSData.set(bmsData);

        // Update DMC data
        DMCData dmcData = sharedDMCData.get();
        dmcData.speedActual = simulatedSpeed;
        dmcData.torqueActual = simulatedTorque;
        dmcData.dcCurrent = bmsData.current;
        dmcData.tempInverter = 30.0f + (abs(dmcData.dcCurrent) / 8.0f);
        dmcData.tempMotor = 28.0f + (abs(simulatedSpeed) / 100.0f);
        dmcData.running = (simulatedSpeed > 100);
        sharedDMCData.set(dmcData);

        // Feed watchdog
        FEED_WATCHDOG();

        // Wait for next cycle
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
#endif

//=============================================================================
// SETUP
//=============================================================================
void setup() {
    // Initialize serial for debugging
    Serial.begin(115200);
    //delay(500);

    DEBUG_PRINTLN("\n\n========================================");
    DEBUG_PRINTLN("  VCU FreeRTOS - Starting Up");
    DEBUG_PRINTLN("========================================\n");

    // Check wakeup reason FIRST - before initializing anything
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    bool wokeFromCharger = false;

    DEBUG_PRINT("Wakeup reason: ");
    DEBUG_PRINTLN((int)wakeup_reason);

    if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT1) {
        uint64_t wakeup_pin_mask = esp_sleep_get_ext1_wakeup_status();

        Serial.print("GPIO that triggered wake: GPIO ");
        Serial.println((log(wakeup_pin_mask))/log(2), 0);

        // Check which pin caused the wake-up
        bool startButtonTriggered = (wakeup_pin_mask & BUTTON_PIN_BITMASK(WAKEUP_GPIO_START)) != 0;
        bool chargerTriggered = (wakeup_pin_mask & BUTTON_PIN_BITMASK(WAKEUP_GPIO_CHARGER)) != 0;

        if (chargerTriggered) {
            DEBUG_PRINTLN("Woke from CHARGER");
            wokeFromCharger = true;
        } else if (startButtonTriggered) {
            DEBUG_PRINTLN("Woke from START button");
            wokeFromCharger = false;
        } else {
            // Unknown pin - go back to sleep
            DEBUG_PRINTLN("Unknown wakeup pin - returning to sleep");
            uint64_t wakeup_bitmask = (1ULL << GPIO_NUM_7) | (1ULL << GPIO_NUM_9);
            esp_sleep_enable_ext1_wakeup(wakeup_bitmask, ESP_EXT1_WAKEUP_ANY_HIGH);
            rtc_gpio_pulldown_en(GPIO_NUM_7);
            rtc_gpio_pullup_dis(GPIO_NUM_7);
            rtc_gpio_pulldown_en(GPIO_NUM_9);
            rtc_gpio_pullup_dis(GPIO_NUM_9);
            delay(100);
            esp_deep_sleep_start();
        }
    } else {
        // Any other wakeup reason (reset button, power-on, etc.) - go to deep sleep
        DEBUG_PRINTLN("Reset/unknown wakeup - entering deep sleep");
        uint64_t wakeup_bitmask = (1ULL << GPIO_NUM_7) | (1ULL << GPIO_NUM_9);
        esp_sleep_enable_ext1_wakeup(wakeup_bitmask, ESP_EXT1_WAKEUP_ANY_HIGH);
        rtc_gpio_pulldown_en(GPIO_NUM_7);
        rtc_gpio_pullup_dis(GPIO_NUM_7);
        rtc_gpio_pulldown_en(GPIO_NUM_9);
        rtc_gpio_pullup_dis(GPIO_NUM_9);
        delay(100);
        esp_deep_sleep_start();
    }

    // Create queues
    canRxQueue = xQueueCreate(FreeRTOS::QUEUE_CAN_RX, sizeof(CANMessage));
    inputEventQueue = xQueueCreate(FreeRTOS::QUEUE_INPUT, sizeof(InputEvent));
    systemEvents = xEventGroupCreate();

    if (canRxQueue == NULL || inputEventQueue == NULL || systemEvents == NULL) {
        DEBUG_PRINTLN("FATAL: Failed to create queues/events!");
        while (1) { delay(1000); }
    }

    // Initialize runtime config with defaults
    sharedRuntimeConfig.set(getDefaultRuntimeConfig());

#if HARDWARE_TEST_MODE
    DEBUG_PRINTLN("\n***********************************************");
    DEBUG_PRINTLN("  HARDWARE TEST MODE - Simulated Data");
    DEBUG_PRINTLN("  Web server and WiFi will run normally");
    DEBUG_PRINTLN("  Hardware managers will be skipped");
    DEBUG_PRINTLN("***********************************************\n");

    // Initialize only software managers
    DEBUG_PRINTLN("Initializing State Manager...");
    stateManager.begin();

    DEBUG_PRINTLN("Initializing Vehicle Control...");
    vehicleControl.begin();

    // Populate simulated data
    initializeSimulatedData();
#else
    // Initialize all managers (normal hardware mode)
    DEBUG_PRINTLN("Initializing Input Manager...");
    if (!inputManager.begin()) {
        DEBUG_PRINTLN("FATAL: Input Manager initialization failed!");
        while (1) { delay(1000); }
    }

    DEBUG_PRINTLN("Initializing CAN Manager...");
    if (!canManager.begin()) {
        DEBUG_PRINTLN("FATAL: CAN Manager initialization failed!");
        while (1) { delay(1000); }
    }

    // Set task handle for CAN ISR notification (must be done after task is created)
    // This will be set later after CAN RX task is created

    // Initialize Contactor Manager first (sets up contactor pins)
    DEBUG_PRINTLN("Initializing Contactor Manager...");
    contactorManager.begin(&bmsManager, &inputManager);

    // Initialize StateManager (powers on BMS, waits 2s)
    DEBUG_PRINTLN("Initializing State Manager...");
    stateManager.begin(&bmsManager, &contactorManager);  // Pass BMS and Contactor manager pointers

    // Now BMS is powered and ready - initialize BMS Manager (sends config)
    DEBUG_PRINTLN("Initializing BMS Manager...");
    bmsManager.begin(&canManager);
    canManager.setBMSManager(&bmsManager);

    // Initialize NLG5 Manager
    DEBUG_PRINTLN("Initializing NLG5 Manager...");
    nlg5Manager.begin();
    canManager.setNLG5Manager(&nlg5Manager);

    DEBUG_PRINTLN("Initializing Vehicle Control...");
    vehicleControl.begin(&bmsManager, &inputManager, &contactorManager);  // Pass all managers for safety
#endif

    // Initialize Display Manager (works in both test and normal mode)
    DEBUG_PRINTLN("Initializing Display Manager...");
    if (!displayManager.begin()) {
        DEBUG_PRINTLN("WARNING: Display Manager initialization failed!");
    }

    DEBUG_PRINTLN("Initializing WiFi Manager...");
    if (!wifiManager.begin()) {
        DEBUG_PRINTLN("WARNING: WiFi Manager initialization failed!");
    }

    DEBUG_PRINTLN("Initializing Web Server...");
    if (!webServer.begin()) {
        DEBUG_PRINTLN("WARNING: Web Server initialization failed!");
    }

    DEBUG_PRINTLN("Initializing Task Monitor...");
    taskMonitor.begin();

    DEBUG_PRINTLN("\n========================================");
    DEBUG_PRINTLN("  Creating FreeRTOS Tasks");
    DEBUG_PRINTLN("========================================\n");

#if HARDWARE_TEST_MODE
    // Hardware test mode - create only essential tasks + simulated data
    DEBUG_PRINTLN("Creating simulated data task...");
    xTaskCreatePinnedToCore(taskSimulatedData, "SimData", 3072,
        NULL, FreeRTOS::PRIORITY_CAN_RX, &taskHandleSimData, FreeRTOS::CORE_PROTOCOL);

    // Still need State and Vehicle tasks for logic
    xTaskCreatePinnedToCore(taskStateManager, "State", FreeRTOS::STACK_STATE_MGR,
        NULL, FreeRTOS::PRIORITY_STATE, &taskHandleState, FreeRTOS::CORE_PROTOCOL);

    xTaskCreatePinnedToCore(taskVehicleControl, "Vehicle", FreeRTOS::STACK_VEHICLE_CTRL,
        NULL, FreeRTOS::PRIORITY_VEHICLE, &taskHandleVehicle, FreeRTOS::CORE_PROTOCOL);
#else
    // Normal hardware mode - create all hardware tasks
    // Create tasks pinned to Core 0 (Protocol Core)
    xTaskCreatePinnedToCore(taskCANRx, "CAN_RX", FreeRTOS::STACK_CAN_RX,
        NULL, FreeRTOS::PRIORITY_CAN_RX, &taskHandleCANRx, FreeRTOS::CORE_PROTOCOL);

    // Set CAN RX task handle for ISR notification
    canManager.canRxTaskHandle = taskHandleCANRx;

    xTaskCreatePinnedToCore(taskCANTx, "CAN_TX", FreeRTOS::STACK_CAN_TX,
        NULL, FreeRTOS::PRIORITY_CAN_TX, &taskHandleCANTx, FreeRTOS::CORE_PROTOCOL);

    xTaskCreatePinnedToCore(taskVehicleControl, "Vehicle", FreeRTOS::STACK_VEHICLE_CTRL,
        NULL, FreeRTOS::PRIORITY_VEHICLE, &taskHandleVehicle, FreeRTOS::CORE_PROTOCOL);

    xTaskCreatePinnedToCore(taskStateManager, "State", FreeRTOS::STACK_STATE_MGR,
        NULL, FreeRTOS::PRIORITY_STATE, &taskHandleState, FreeRTOS::CORE_PROTOCOL);

    xTaskCreatePinnedToCore(taskSafetyMonitor, "Safety", FreeRTOS::STACK_SAFETY,
        NULL, FreeRTOS::PRIORITY_SAFETY, &taskHandleSafety, FreeRTOS::CORE_PROTOCOL);

    // Create tasks pinned to Core 1 (Application Core)
    xTaskCreatePinnedToCore(taskInputManager, "Input", FreeRTOS::STACK_INPUT,
        NULL, FreeRTOS::PRIORITY_INPUT, &taskHandleInput, FreeRTOS::CORE_APPLICATION);
#endif

    // Display task runs in both modes (shows real data in hardware mode, simulated data in test mode)
    xTaskCreatePinnedToCore(taskDisplayManager, "Display", FreeRTOS::STACK_DISPLAY,
        NULL, FreeRTOS::PRIORITY_DISPLAY, &taskHandleDisplay, FreeRTOS::CORE_APPLICATION);

    xTaskCreatePinnedToCore(taskWiFiManager, "WiFi", FreeRTOS::STACK_WIFI,
        NULL, FreeRTOS::PRIORITY_WIFI, &taskHandleWiFi, FreeRTOS::CORE_APPLICATION);

    xTaskCreatePinnedToCore(taskWebServer, "WebServer", FreeRTOS::STACK_WEBSERVER,
        NULL, FreeRTOS::PRIORITY_WEBSERVER, &taskHandleWeb, FreeRTOS::CORE_APPLICATION);

    xTaskCreatePinnedToCore(taskTaskMonitor, "Monitor", FreeRTOS::STACK_MONITOR,
        NULL, FreeRTOS::PRIORITY_MONITOR, &taskHandleMonitor, FreeRTOS::CORE_APPLICATION);

    xTaskCreatePinnedToCore(taskLEDControl, "LED", 2048,
        NULL, 3, &taskHandleLED, FreeRTOS::CORE_APPLICATION);

    // Task: Error Aggregator (Priority: 4 - Medium, after safety)
    xTaskCreatePinnedToCore(taskErrorAggregator, "ErrorAgg", 4096,
        NULL, 4, &taskHandleErrorAggregator, FreeRTOS::CORE_APPLICATION);

    // Register tasks with monitor
#if HARDWARE_TEST_MODE
    taskMonitor.registerTask(taskHandleSimData, "SimData", 500);
    taskMonitor.registerTask(taskHandleVehicle, "Vehicle", 500);
    taskMonitor.registerTask(taskHandleState, "State", 1000);
#else
    taskMonitor.registerTask(taskHandleCANRx, "CAN_RX", 500);
    taskMonitor.registerTask(taskHandleCANTx, "CAN_TX", 500);
    taskMonitor.registerTask(taskHandleVehicle, "Vehicle", 500);
    taskMonitor.registerTask(taskHandleState, "State", 1000);
    taskMonitor.registerTask(taskHandleSafety, "Safety", 1000);
    taskMonitor.registerTask(taskHandleInput, "Input", 1000);
    taskMonitor.registerTask(taskHandleDisplay, "Display", 1000);
#endif
    taskMonitor.registerTask(taskHandleWiFi, "WiFi", 5000);
    taskMonitor.registerTask(taskHandleWeb, "WebServer", 5000);
    taskMonitor.registerTask(taskHandleErrorAggregator, "ErrorAgg", 2000);  // 1Hz task, 2s watchdog
    taskMonitor.registerTask(taskHandleMonitor, "Monitor", 0);  // No watchdog for monitor

    DEBUG_PRINTLN("\n========================================");
    DEBUG_PRINTLN("  VCU FreeRTOS Initialized!");
    DEBUG_PRINTLN("  Tasks running on dual cores");
    DEBUG_PRINTLN("========================================\n");

    // Handle wakeup - StateManager will transition to appropriate mode
    stateManager.handleWakeup(wokeFromCharger);
}

//=============================================================================
// LOOP (not used - FreeRTOS tasks handle everything)
//=============================================================================
void loop() {
    // Empty - all work is done in FreeRTOS tasks
    // This loop() still exists for Arduino compatibility
    vTaskDelay(portMAX_DELAY);
}
