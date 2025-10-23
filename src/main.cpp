#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/event_groups.h>
#include "config.h"
#include "data_structures.h"
#include "input_manager.h"
#include "state_manager.h"
#include "vehicle_control.h"
#include "can_manager.h"
#include "display_manager.h"
#include "wifi_manager.h"
#include "web_server.h"
#include "task_monitor.h"

//=============================================================================
// GLOBAL OBJECTS
//=============================================================================
InputManager inputManager;
StateManager stateManager;
VehicleControl vehicleControl;
CANManager canManager;
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

    while (1) {
        // Process all messages in queue
        canManager.processRxQueue();

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

        // Handle charging state
        if (stateManager.isCharging()) {
            uint8_t chargerState = 1;  // 1 = charging enabled
            canManager.sendNLGControl(chargerState);
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

    while (1) {
        BMSData bmsData = sharedBMSData.get();
        DMCData dmcData = sharedDMCData.get();
        RuntimeConfigData config = sharedRuntimeConfig.get();

        // Check motor temperature
        if (dmcData.tempMotor > config.maxMotorTemp) {
            DEBUG_PRINTLN("WARNING: Motor overtemperature!");
            displayManager.showWarning("MOTOR OVERHEAT");
        }

        // Check inverter temperature
        if (dmcData.tempInverter > config.maxInverterTemp) {
            DEBUG_PRINTLN("WARNING: Inverter overtemperature!");
            displayManager.showWarning("INVERTER OVERHEAT");
        }

        // Check battery voltage
        if (bmsData.voltage < Battery::MIN_VOLTAGE) {
            DEBUG_PRINTLN("WARNING: Battery undervoltage!");
            displayManager.showWarning("LOW VOLTAGE");
        }

        // Check SOC
        if (bmsData.soc < Battery::MIN_SOC) {
            DEBUG_PRINTLN("WARNING: Low battery!");
            displayManager.showWarning("LOW BATTERY");
        }

        // Check cell voltage
        if (bmsData.minCellVoltage < config.criticalCellVoltage) {
            DEBUG_PRINTLN("CRITICAL: Cell voltage too low!");
            displayManager.showWarning("CELL CRITICAL");
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

        // Handle START button
        static bool startButtonProcessed = false;
        if (inputManager.isStartPressed()) {
            if (!startButtonProcessed) {
                stateManager.handleStartButton();
                startButtonProcessed = true;
            }
        } else {
            startButtonProcessed = false;
        }

        // Handle STOP button (with hold-to-sleep)
        if (inputManager.isStopPressed()) {
            if (!stopButtonHeld) {
                stopButtonHoldStart = millis();
                stopButtonHeld = true;
                DEBUG_PRINTLN("Stop button pressed");
            }

            // Check for hold timeout
            unsigned long holdTime = millis() - stopButtonHoldStart;
            if (holdTime >= Timing::SLEEP_TIMEOUT) {
                RuntimeConfigData config = sharedRuntimeConfig.get();

                // Only enter sleep if not in debug mode
                if (!config.debugMode) {
                    DEBUG_PRINTLN("Stop button held - Entering sleep mode...");
                    stateManager.handleStopButton();
                    vTaskDelay(pdMS_TO_TICKS(100));
                    esp_deep_sleep_start();
                } else {
                    DEBUG_PRINTLN("Debug mode: Sleep disabled");
                    stopButtonHeld = false;
                }
            }
        } else {
            stopButtonHeld = false;
        }

        // Handle RESET button
        if (inputManager.isResetPressed()) {
            DEBUG_PRINTLN("Reset button pressed - Restarting ESP32...");
            vTaskDelay(pdMS_TO_TICKS(100));
            ESP.restart();
        }

        // Handle Interlock
        static bool ilWasClosed = true;
        bool ilClosed = inputManager.isILClosed();

        if (ilWasClosed && !ilClosed) {
            DEBUG_PRINTLN("Interlock opened!");
            stateManager.handleILOpen();
            xEventGroupClearBits(systemEvents, EVENT_IL_CLOSED);
        } else if (!ilWasClosed && ilClosed) {
            xEventGroupSetBits(systemEvents, EVENT_IL_CLOSED);
        }
        ilWasClosed = ilClosed;

        // Handle charger connection
        static bool chargerWasConnected = false;
        bool chargerConnected = inputManager.isChargerConnected();

        if (!chargerWasConnected && chargerConnected) {
            DEBUG_PRINTLN("Charger connected");
            stateManager.handleChargerConnected();
            xEventGroupSetBits(systemEvents, EVENT_CHARGER_CONNECTED);
        } else if (chargerWasConnected && !chargerConnected) {
            DEBUG_PRINTLN("Charger disconnected");
            stateManager.handleChargerDisconnected();
            xEventGroupClearBits(systemEvents, EVENT_CHARGER_CONNECTED);
        }
        chargerWasConnected = chargerConnected;

        // Handle direction toggle
        if (inputManager.isDirectionTogglePressed()) {
            vehicleControl.handleDirectionToggle();
        }

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
        telem.brakePressed = (dmcData.torqueActual < 0);
        telem.ilClosed = true;
        telem.bmsAlive = true;
        telem.dmcReady = true;
        telem.chargerConnected = false;
#else
        // Real hardware data
        telem.throttlePercent = inputManager.getThrottlePercent();
        telem.regenPercent = inputManager.getRegenPercent();
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
    delay(500);

    DEBUG_PRINTLN("\n\n========================================");
    DEBUG_PRINTLN("  VCU FreeRTOS - Starting Up");
    DEBUG_PRINTLN("========================================\n");

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

    DEBUG_PRINTLN("Initializing State Manager...");
    stateManager.begin();

    DEBUG_PRINTLN("Initializing Vehicle Control...");
    vehicleControl.begin();

    DEBUG_PRINTLN("Initializing CAN Manager...");
    if (!canManager.begin()) {
        DEBUG_PRINTLN("FATAL: CAN Manager initialization failed!");
        while (1) { delay(1000); }
    }
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
    taskMonitor.registerTask(taskHandleMonitor, "Monitor", 0);  // No watchdog for monitor

    DEBUG_PRINTLN("\n========================================");
    DEBUG_PRINTLN("  VCU FreeRTOS Initialized!");
    DEBUG_PRINTLN("  Tasks running on dual cores");
    DEBUG_PRINTLN("========================================\n");

    // Check wakeup reason
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT0) {
        DEBUG_PRINTLN("Woke up from START button");
        stateManager.handleStartButton();
    } else {
        DEBUG_PRINTLN("Power-on reset");
    }
}

//=============================================================================
// LOOP (not used - FreeRTOS tasks handle everything)
//=============================================================================
void loop() {
    // Empty - all work is done in FreeRTOS tasks
    // This loop() still exists for Arduino compatibility
    vTaskDelay(portMAX_DELAY);
}
