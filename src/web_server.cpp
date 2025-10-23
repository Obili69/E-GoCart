#include "web_server.h"
#include "task_monitor.h"
#include "wifi_manager.h"

extern TaskMonitor taskMonitor;
extern WiFiManager wifiManager;

WebServer::WebServer()
    : server(WiFi_Config::HTTP_PORT)
    , ws("/ws")
    , otaMode(false)
    , lastTelemetryBroadcast(0)
    , lastTaskStatsBroadcast(0)
{
}

bool WebServer::begin() {
    DEBUG_PRINTLN("WebServer: Initializing...");

    // Setup WebSocket
    setupWebSocket();

    // Setup routes
    setupRoutes();

    // Setup OTA (if enabled in config)
    RuntimeConfigData config = sharedRuntimeConfig.get();
    if (config.enableOTA) {
        setupOTA();
    }

    // Start server
    server.begin();

    DEBUG_PRINTF("WebServer: Started on port %d\n", WiFi_Config::HTTP_PORT);
    return true;
}

void WebServer::update() {
    // Handle OTA if active
    if (otaMode) {
        ArduinoOTA.handle();
    }

    // Broadcast telemetry via WebSocket
    unsigned long currentTime = millis();
    if (currentTime - lastTelemetryBroadcast >= WiFi_Config::WEBSOCKET_UPDATE) {
        lastTelemetryBroadcast = currentTime;
        broadcastTelemetry();
    }

    // Broadcast task stats every second
    if (currentTime - lastTaskStatsBroadcast >= 1000) {
        lastTaskStatsBroadcast = currentTime;
        broadcastTaskStats();
    }

    // Cleanup WebSocket clients
    ws.cleanupClients();

    // Yield to other tasks
    vTaskDelay(pdMS_TO_TICKS(10));
}

void WebServer::setupWebSocket() {
    ws.onEvent([this](AsyncWebSocket *server, AsyncWebSocketClient *client,
                     AwsEventType type, void *arg, uint8_t *data, size_t len) {
        this->onWebSocketEvent(server, client, type, arg, data, len);
    });

    server.addHandler(&ws);
}

void WebServer::setupRoutes() {
    // Serve static files from LittleFS
    server.serveStatic("/", LittleFS, "/www/").setDefaultFile("index.html");

    // REST API endpoints
    server.on("/api/telemetry", HTTP_GET, [this](AsyncWebServerRequest *request) {
        this->handleGetTelemetry(request);
    });

    server.on("/api/config", HTTP_GET, [this](AsyncWebServerRequest *request) {
        this->handleGetConfig(request);
    });

    // Handle POST with body parser
    server.on("/api/config", HTTP_POST,
        [this](AsyncWebServerRequest *request) {
            // This is called after body is received
            this->handleSetConfig(request);
        },
        NULL,
        [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
            // Store body data in request object
            // Note: We'll use URL params instead for simplicity
        });

    server.on("/api/tasks", HTTP_GET, [this](AsyncWebServerRequest *request) {
        this->handleGetTaskStats(request);
    });

    server.on("/api/ota/enter", HTTP_POST, [this](AsyncWebServerRequest *request) {
        this->handleEnterOTAMode(request);
    });

    server.on("/api/ota/exit", HTTP_POST, [this](AsyncWebServerRequest *request) {
        this->handleExitOTAMode(request);
    });

    server.on("/api/restart", HTTP_POST, [this](AsyncWebServerRequest *request) {
        this->handleRestart(request);
    });

    server.on("/api/wifi/scan", HTTP_GET, [this](AsyncWebServerRequest *request) {
        this->handleWiFiScan(request);
    });

    server.on("/api/wifi/test", HTTP_POST, [this](AsyncWebServerRequest *request) {
        this->handleWiFiTest(request);
    });

    server.on("/api/wifi/status", HTTP_GET, [this](AsyncWebServerRequest *request) {
        this->handleWiFiStatus(request);
    });

    // Charging API endpoints
    server.on("/api/charging/config", HTTP_GET, [this](AsyncWebServerRequest *request) {
        this->handleGetChargingConfig(request);
    });

    server.on("/api/charging/config", HTTP_POST, [this](AsyncWebServerRequest *request) {
        this->handleSetChargingConfig(request);
    });

    server.on("/api/charging/preset", HTTP_POST, [this](AsyncWebServerRequest *request) {
        this->handleSetChargingPreset(request);
    });

    server.on("/api/charging/status", HTTP_GET, [this](AsyncWebServerRequest *request) {
        this->handleGetChargingStatus(request);
    });

    server.on("/api/charging/start", HTTP_POST, [this](AsyncWebServerRequest *request) {
        this->handleStartCharging(request);
    });

    server.on("/api/charging/stop", HTTP_POST, [this](AsyncWebServerRequest *request) {
        this->handleStopCharging(request);
    });

    server.on("/api/charging/limits", HTTP_GET, [this](AsyncWebServerRequest *request) {
        this->handleGetChargingLimits(request);
    });

    // 404 handler
    server.onNotFound([](AsyncWebServerRequest *request) {
        request->send(404, "text/plain", "Not found");
    });
}

void WebServer::setupOTA() {
    DEBUG_PRINTLN("WebServer: Configuring OTA...");

    ArduinoOTA.setHostname(WiFi_Config::OTA_HOSTNAME);
    ArduinoOTA.setPassword(WiFi_Config::OTA_PASSWORD);
    ArduinoOTA.setPort(WiFi_Config::OTA_PORT);

    ArduinoOTA.onStart([]() {
        String type = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";
        DEBUG_PRINTF("OTA: Start updating %s\n", type.c_str());

        // Stop critical tasks during OTA
        // TODO: Implement task suspension
    });

    ArduinoOTA.onEnd([]() {
        DEBUG_PRINTLN("\nOTA: Update complete");
    });

    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        DEBUG_PRINTF("OTA Progress: %u%%\r", (progress / (total / 100)));
    });

    ArduinoOTA.onError([](ota_error_t error) {
        DEBUG_PRINTF("OTA Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) DEBUG_PRINTLN("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) DEBUG_PRINTLN("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) DEBUG_PRINTLN("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) DEBUG_PRINTLN("Receive Failed");
        else if (error == OTA_END_ERROR) DEBUG_PRINTLN("End Failed");
    });

    // Start OTA service
    DEBUG_PRINTLN("WebServer: Starting OTA service...");
    ArduinoOTA.begin();
    otaMode = true;  // Mark as active
    DEBUG_PRINTLN("WebServer: OTA service started");
}

void WebServer::setOTAMode(bool enable) {
    if (enable && !otaMode) {
        DEBUG_PRINTLN("WebServer: Entering OTA mode");
        ArduinoOTA.begin();
        otaMode = true;
        xEventGroupSetBits(systemEvents, EVENT_OTA_MODE);
    } else if (!enable && otaMode) {
        DEBUG_PRINTLN("WebServer: Exiting OTA mode");
        // ArduinoOTA.end(); // Not available in Arduino OTA
        otaMode = false;
        xEventGroupClearBits(systemEvents, EVENT_OTA_MODE);
    }
}

void WebServer::broadcastTelemetry() {
    if (ws.count() == 0) {
        return;  // No clients connected
    }

    String json = getTelemetryJSON();
    ws.textAll(json);

#if DEBUG_WEBSERVER
    DEBUG_PRINTF("WebSocket: Broadcast telemetry to %d clients\n", ws.count());
#endif
}

void WebServer::broadcastTaskStats() {
    if (ws.count() == 0) {
        return;  // No clients connected
    }

    String json = getTaskStatsJSON();
    ws.textAll(json);
}

void WebServer::handleRoot(AsyncWebServerRequest *request) {
    request->send(LittleFS, "/www/index.html", "text/html");
}

void WebServer::handleGetTelemetry(AsyncWebServerRequest *request) {
    String json = getTelemetryJSON();
    request->send(200, "application/json", json);
}

void WebServer::handleGetConfig(AsyncWebServerRequest *request) {
    String json = getConfigJSON();
    request->send(200, "application/json", json);
}

void WebServer::handleSetConfig(AsyncWebServerRequest *request) {
    // For simplicity, use individual query parameters
    // Example: /api/config?canFastCycle=20&maxTorque=800

    bool updated = false;

    sharedRuntimeConfig.update([&](RuntimeConfigData& config) {
        if (request->hasParam("canFastCycle")) {
            config.canFastCycle = request->getParam("canFastCycle")->value().toInt();
            updated = true;
        }
        if (request->hasParam("canSlowCycle")) {
            config.canSlowCycle = request->getParam("canSlowCycle")->value().toInt();
            updated = true;
        }
        if (request->hasParam("maxTorque")) {
            config.maxTorqueNm = request->getParam("maxTorque")->value().toInt();
            updated = true;
        }
        if (request->hasParam("maxRegen")) {
            config.maxRegenNm = request->getParam("maxRegen")->value().toInt();
            updated = true;
        }
        if (request->hasParam("maxReverse")) {
            config.maxReverseNm = request->getParam("maxReverse")->value().toInt();
            updated = true;
        }
        if (request->hasParam("maxMotorTemp")) {
            config.maxMotorTemp = request->getParam("maxMotorTemp")->value().toFloat();
            updated = true;
        }
        if (request->hasParam("maxInverterTemp")) {
            config.maxInverterTemp = request->getParam("maxInverterTemp")->value().toFloat();
            updated = true;
        }
        if (request->hasParam("maxBatteryTemp")) {
            config.maxBatteryTemp = request->getParam("maxBatteryTemp")->value().toFloat();
            updated = true;
        }
        if (request->hasParam("criticalCellV")) {
            config.criticalCellVoltage = request->getParam("criticalCellV")->value().toFloat();
            updated = true;
        }
        if (request->hasParam("enableAP")) {
            config.enableAP = (request->getParam("enableAP")->value() == "true");
            updated = true;
        }
        if (request->hasParam("enableSTA")) {
            config.enableSTA = (request->getParam("enableSTA")->value() == "true");
            updated = true;
        }
        if (request->hasParam("staSSID")) {
            strncpy(config.staSSID, request->getParam("staSSID")->value().c_str(), 31);
            config.staSSID[31] = '\0';
            updated = true;
        }
        if (request->hasParam("staPassword")) {
            strncpy(config.staPassword, request->getParam("staPassword")->value().c_str(), 63);
            config.staPassword[63] = '\0';
            updated = true;

            // Save credentials to WiFi manager for connection
            wifiManager.saveSTACredentials(config.staSSID, config.staPassword);
        }
        if (request->hasParam("debugMode")) {
            config.debugMode = (request->getParam("debugMode")->value() == "true");
            updated = true;
        }
        if (request->hasParam("enableOTA")) {
            config.enableOTA = (request->getParam("enableOTA")->value() == "true");
            updated = true;
        }

        // Recalculate checksum
        config.checksum = calculateConfigChecksum(config);
    });

    if (updated) {
        request->send(200, "text/plain", "Config updated");
    } else {
        request->send(400, "text/plain", "No valid parameters provided");
    }
}

void WebServer::handleGetTaskStats(AsyncWebServerRequest *request) {
    String json = getTaskStatsJSON();
    request->send(200, "application/json", json);
}

void WebServer::handleEnterOTAMode(AsyncWebServerRequest *request) {
    setOTAMode(true);
    request->send(200, "text/plain", "OTA mode enabled");
}

void WebServer::handleExitOTAMode(AsyncWebServerRequest *request) {
    setOTAMode(false);
    request->send(200, "text/plain", "OTA mode disabled");
}

void WebServer::handleRestart(AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "Restarting...");
    delay(1000);
    ESP.restart();
}

void WebServer::handleWiFiScan(AsyncWebServerRequest *request) {
    DEBUG_PRINTLN("WebServer: WiFi scan requested");

    // Perform scan
    String json = wifiManager.scanNetworks();

    // Send response
    request->send(200, "application/json", json);
}

void WebServer::handleWiFiTest(AsyncWebServerRequest *request) {
    DEBUG_PRINTLN("WebServer: WiFi test connection requested");

    // Get SSID and password from query parameters
    String ssid = "";
    String password = "";

    if (request->hasParam("ssid")) {
        ssid = request->getParam("ssid")->value();
    }
    if (request->hasParam("password")) {
        password = request->getParam("password")->value();
    }

    if (ssid.length() == 0) {
        request->send(400, "application/json", "{\"success\":false,\"message\":\"SSID required\"}");
        return;
    }

    // Test connection
    String json = wifiManager.testConnection(ssid.c_str(), password.c_str());

    // Send response
    request->send(200, "application/json", json);
}

void WebServer::handleWiFiStatus(AsyncWebServerRequest *request) {
    DEBUG_PRINTLN("WebServer: WiFi status requested");

    // Get status
    String json = wifiManager.getConnectionStatus();

    // Send response
    request->send(200, "application/json", json);
}

void WebServer::onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
                                 AwsEventType type, void *arg, uint8_t *data, size_t len) {
    if (type == WS_EVT_CONNECT) {
        DEBUG_PRINTF("WebSocket: Client #%u connected from %s\n",
                    client->id(), client->remoteIP().toString().c_str());

        // Send initial telemetry
        String json = getTelemetryJSON();
        client->text(json);

    } else if (type == WS_EVT_DISCONNECT) {
        DEBUG_PRINTF("WebSocket: Client #%u disconnected\n", client->id());

    } else if (type == WS_EVT_ERROR) {
        DEBUG_PRINTF("WebSocket: Client #%u error\n", client->id());

    } else if (type == WS_EVT_DATA) {
        // Handle incoming WebSocket data
        AwsFrameInfo *info = (AwsFrameInfo*)arg;
        if (info->final && info->index == 0 && info->len == len) {
            // Complete message received
            if (info->opcode == WS_TEXT) {
                data[len] = 0;  // Null-terminate
                DEBUG_PRINTF("WebSocket: Received: %s\n", (char*)data);

                // Parse and handle commands
                // TODO: Implement command handling
            }
        }
    }
}

String WebServer::getTelemetryJSON() {
    VehicleTelemetry telem = sharedTelemetry.get();

    StaticJsonDocument<1024> doc;

    // Vehicle state
    doc["state"] = (uint8_t)telem.state;
    doc["gear"] = (uint8_t)telem.gear;
    doc["ready"] = telem.systemReady;

    // Motor/Vehicle
    doc["speed"] = telem.speedKmh;
    doc["torqueDemand"] = telem.torqueDemand;
    doc["torqueActual"] = telem.torqueActual;
    doc["motorRPM"] = telem.motorRPM;

    // Battery
    doc["soc"] = telem.soc;
    doc["voltage"] = telem.voltage;
    doc["current"] = telem.current;
    doc["power"] = telem.powerKw;
    doc["minCell"] = telem.minCellVoltage;
    doc["maxCell"] = telem.maxCellVoltage;

    // Temperatures
    doc["tempMotor"] = telem.tempMotor;
    doc["tempInverter"] = telem.tempInverter;
    doc["tempBattery"] = telem.tempBattery;

    // Inputs
    doc["throttle"] = telem.throttlePercent;
    doc["regen"] = telem.regenPercent;
    doc["brake"] = telem.brakePressed;
    doc["il"] = telem.ilClosed;

    // Status
    doc["bmsAlive"] = telem.bmsAlive;
    doc["dmcReady"] = telem.dmcReady;
    doc["charger"] = telem.chargerConnected;

    doc["timestamp"] = telem.timestamp;

    String output;
    serializeJson(doc, output);
    return output;
}

String WebServer::getConfigJSON() {
    RuntimeConfigData config = sharedRuntimeConfig.get();

    StaticJsonDocument<512> doc;

    // CAN Timing
    doc["canFastCycle"] = config.canFastCycle;
    doc["canSlowCycle"] = config.canSlowCycle;

    // Motor Limits
    doc["maxTorque"] = config.maxTorqueNm;
    doc["maxRegen"] = config.maxRegenNm;
    doc["maxReverse"] = config.maxReverseNm;

    // Safety Limits
    doc["maxMotorTemp"] = config.maxMotorTemp;
    doc["maxInverterTemp"] = config.maxInverterTemp;
    doc["maxBatteryTemp"] = config.maxBatteryTemp;
    doc["criticalCellV"] = config.criticalCellVoltage;

    // WiFi
    doc["enableAP"] = config.enableAP;
    doc["enableSTA"] = config.enableSTA;
    doc["staSSID"] = config.staSSID;

    // Debug
    doc["debugMode"] = config.debugMode;
    doc["enableOTA"] = config.enableOTA;

    String output;
    serializeJson(doc, output);
    return output;
}

String WebServer::getTaskStatsJSON() {
    TaskStats stats[15];
    uint8_t count = taskMonitor.getTaskStats(stats, 15);

    StaticJsonDocument<2048> doc;
    JsonArray tasks = doc.createNestedArray("tasks");

    for (uint8_t i = 0; i < count; i++) {
        JsonObject task = tasks.createNestedObject();
        task["name"] = stats[i].name;
        task["priority"] = stats[i].priority;
        task["stack"] = stats[i].stackHighWater;
        task["cpu"] = stats[i].cpuPercent;
        task["state"] = stats[i].state;
        task["watchdog"] = stats[i].watchdogOK;
    }

    doc["freeHeap"] = taskMonitor.getFreeHeap();
    doc["cpuUsage"] = taskMonitor.getTotalCPUUsage();

    String output;
    serializeJson(doc, output);
    return output;
}

//=============================================================================
// CHARGING API HANDLERS
//=============================================================================

void WebServer::handleGetChargingConfig(AsyncWebServerRequest *request) {
    String json = getChargingConfigJSON();
    request->send(200, "application/json", json);
}

void WebServer::handleSetChargingConfig(AsyncWebServerRequest *request) {
    bool updated = false;

    sharedRuntimeConfig.update([&](RuntimeConfigData& config) {
        // Storage voltage
        if (request->hasParam("storageVoltagePerCell")) {
            float value = request->getParam("storageVoltagePerCell")->value().toFloat();
            if (value >= 3.70f && value <= 3.90f) {
                config.storageVoltagePerCell = value;
                updated = true;
            }
        }

        // Max charge voltage
        if (request->hasParam("maxChargeVoltagePerCell")) {
            float value = request->getParam("maxChargeVoltagePerCell")->value().toFloat();
            if (value >= 4.10f && value <= 4.20f) {
                config.maxChargeVoltagePerCell = value;
                updated = true;
            }
        }

        // Bulk charge voltage
        if (request->hasParam("bulkChargeVoltagePerCell")) {
            float value = request->getParam("bulkChargeVoltagePerCell")->value().toFloat();
            if (value >= 4.00f && value <= 4.15f) {
                config.bulkChargeVoltagePerCell = value;
                updated = true;
            }
        }

        // Max charge current
        if (request->hasParam("maxChargeCurrent")) {
            float value = request->getParam("maxChargeCurrent")->value().toFloat();
            if (value >= 1.0f && value <= 12.5f) {  // NLG5 hardware limit
                config.maxChargeCurrent = value;
                updated = true;
            }
        }

        // Storage charge current
        if (request->hasParam("storageChargeCurrent")) {
            float value = request->getParam("storageChargeCurrent")->value().toFloat();
            if (value >= 1.0f && value <= 10.0f) {
                config.storageChargeCurrent = value;
                updated = true;
            }
        }

        // Mains current limit (CRITICAL - prevents circuit breaker trips)
        if (request->hasParam("mainsCurrentLimit")) {
            float value = request->getParam("mainsCurrentLimit")->value().toFloat();
            if (value >= 6.0f && value <= 16.0f) {
                config.mainsCurrentLimit = value;
                updated = true;
            }
        }

        // Charge timeout
        if (request->hasParam("chargeTimeoutMinutes")) {
            uint16_t value = request->getParam("chargeTimeoutMinutes")->value().toInt();
            if (value >= 60 && value <= 600) {
                config.chargeTimeoutMinutes = value;
                updated = true;
            }
        }

        // Max charge temperature
        if (request->hasParam("maxChargeTemp")) {
            float value = request->getParam("maxChargeTemp")->value().toFloat();
            if (value >= 35.0f && value <= 50.0f) {
                config.maxChargeTemp = value;
                updated = true;
            }
        }

        // Min charge temperature
        if (request->hasParam("minChargeTemp")) {
            float value = request->getParam("minChargeTemp")->value().toFloat();
            if (value >= -5.0f && value <= 10.0f) {
                config.minChargeTemp = value;
                updated = true;
            }
        }

        // Auto-stop at storage
        if (request->hasParam("autoStopAtStorage")) {
            config.autoStopAtStorage = (request->getParam("autoStopAtStorage")->value() == "true");
            updated = true;
        }

        // Balancing enabled
        if (request->hasParam("balancingEnabled")) {
            config.balancingEnabled = (request->getParam("balancingEnabled")->value() == "true");
            updated = true;
        }

        // Recalculate checksum if updated
        if (updated) {
            config.checksum = calculateConfigChecksum(config);
        }
    });

    if (updated) {
        request->send(200, "application/json", "{\"status\":\"ok\",\"message\":\"Charging config updated\"}");
    } else {
        request->send(400, "application/json", "{\"status\":\"error\",\"message\":\"No valid parameters\"}");
    }
}

void WebServer::handleSetChargingPreset(AsyncWebServerRequest *request) {
    if (!request->hasParam("preset")) {
        request->send(400, "application/json", "{\"status\":\"error\",\"message\":\"Missing preset parameter\"}");
        return;
    }

    uint8_t preset = request->getParam("preset")->value().toInt();

    sharedRuntimeConfig.update([&](RuntimeConfigData& config) {
        switch (preset) {
            case 1: // Storage Mode - Gentle charge to storage voltage
                config.chargingPreset = 1;
                config.maxChargeVoltagePerCell = 3.80f;  // Storage voltage
                config.maxChargeCurrent = 5.0f;           // Gentle
                config.mainsCurrentLimit = 8.0f;          // Low AC current
                break;

            case 2: // Fast Charge - Maximum safe charging
                config.chargingPreset = 2;
                config.maxChargeVoltagePerCell = 4.17f;  // Safe max
                config.maxChargeCurrent = 12.5f;          // NLG5 max
                config.mainsCurrentLimit = 16.0f;         // Full AC current
                break;

            case 3: // Gentle Charge - Battery-friendly
                config.chargingPreset = 3;
                config.maxChargeVoltagePerCell = 4.15f;  // Slightly lower
                config.maxChargeCurrent = 6.0f;           // Gentle
                config.mainsCurrentLimit = 10.0f;         // Moderate AC
                break;

            case 4: // Eco Charge - Balanced efficiency
                config.chargingPreset = 4;
                config.maxChargeVoltagePerCell = 4.10f;  // Bulk voltage
                config.maxChargeCurrent = 8.0f;           // Moderate
                config.mainsCurrentLimit = 13.0f;         // Standard AC
                break;

            default: // Custom or invalid
                config.chargingPreset = 0;
                return;
        }

        // Recalculate checksum
        config.checksum = calculateConfigChecksum(config);
    });

    String json = "{\"status\":\"ok\",\"message\":\"Preset applied\",\"preset\":" + String(preset) + "}";
    request->send(200, "application/json", json);
}

void WebServer::handleGetChargingStatus(AsyncWebServerRequest *request) {
    String json = getChargingStatusJSON();
    request->send(200, "application/json", json);
}

void WebServer::handleStartCharging(AsyncWebServerRequest *request) {
    // Apply preset if mode is specified
    if (request->hasParam("mode")) {
        String mode = request->getParam("mode")->value();

        sharedRuntimeConfig.update([&](RuntimeConfigData& config) {
            if (mode == "storage") {
                config.chargingPreset = 1;
                config.maxChargeVoltagePerCell = 3.80f;
                config.maxChargeCurrent = 5.0f;
                config.mainsCurrentLimit = 8.0f;
            } else if (mode == "fast") {
                config.chargingPreset = 2;
                config.maxChargeVoltagePerCell = 4.17f;
                config.maxChargeCurrent = 12.5f;
                config.mainsCurrentLimit = 16.0f;
            } else if (mode == "gentle") {
                config.chargingPreset = 3;
                config.maxChargeVoltagePerCell = 4.15f;
                config.maxChargeCurrent = 6.0f;
                config.mainsCurrentLimit = 10.0f;
            } else if (mode == "eco") {
                config.chargingPreset = 4;
                config.maxChargeVoltagePerCell = 4.10f;
                config.maxChargeCurrent = 8.0f;
                config.mainsCurrentLimit = 13.0f;
            }

            config.checksum = calculateConfigChecksum(config);
        });
    }

    // TODO: Integrate with ChargingCoordinator when available
    request->send(200, "application/json", "{\"status\":\"ok\",\"message\":\"Charging start requested\"}");
}

void WebServer::handleStopCharging(AsyncWebServerRequest *request) {
    // TODO: Integrate with ChargingCoordinator when available
    request->send(200, "application/json", "{\"status\":\"ok\",\"message\":\"Charging stop requested\"}");
}

void WebServer::handleGetChargingLimits(AsyncWebServerRequest *request) {
    String json = getChargingLimitsJSON();
    request->send(200, "application/json", json);
}

String WebServer::getChargingConfigJSON() {
    RuntimeConfigData config = sharedRuntimeConfig.get();

    StaticJsonDocument<1024> doc;

    // Per-cell voltages
    doc["storageVoltagePerCell"] = serialized(String(config.storageVoltagePerCell, 2));
    doc["maxChargeVoltagePerCell"] = serialized(String(config.maxChargeVoltagePerCell, 2));
    doc["bulkChargeVoltagePerCell"] = serialized(String(config.bulkChargeVoltagePerCell, 2));

    // Pack voltages (calculated)
    doc["storageVoltagePack"] = serialized(String(getStoragePackVoltage(config), 1));
    doc["maxChargeVoltagePack"] = serialized(String(getMaxChargePackVoltage(config), 1));
    doc["bulkChargeVoltagePack"] = serialized(String(getBulkChargePackVoltage(config), 1));

    // Current limits
    doc["maxChargeCurrent"] = serialized(String(config.maxChargeCurrent, 1));
    doc["storageChargeCurrent"] = serialized(String(config.storageChargeCurrent, 1));
    doc["mainsCurrentLimit"] = serialized(String(config.mainsCurrentLimit, 1));

    // Safety limits
    doc["chargeTimeoutMinutes"] = config.chargeTimeoutMinutes;
    doc["maxChargeTemp"] = serialized(String(config.maxChargeTemp, 1));
    doc["minChargeTemp"] = serialized(String(config.minChargeTemp, 1));

    // Preferences
    doc["chargingPreset"] = config.chargingPreset;
    doc["autoStopAtStorage"] = config.autoStopAtStorage;
    doc["balancingEnabled"] = config.balancingEnabled;

    String output;
    serializeJson(doc, output);
    return output;
}

String WebServer::getChargingStatusJSON() {
    // Get current BMS and NLG5 data
    BMSData bmsData = sharedBMSData.get();
    NLGData nlgData = sharedNLGData.get();

    StaticJsonDocument<1024> doc;

    // Charger state
    doc["chargerState"] = nlgData.state;
    doc["chargerConnected"] = (xEventGroupGetBits(systemEvents) & EVENT_CHARGER_CONNECTED) != 0;

    // Voltage and current
    doc["actualVoltage"] = bmsData.voltage;
    doc["actualCurrent"] = nlgData.dcCurrent;
    doc["chargePower"] = (bmsData.voltage * nlgData.dcCurrent) / 100;  // Convert to watts

    // Battery status
    doc["socPercent"] = bmsData.soc;
    doc["batteryTemp"] = serialized(String(bmsData.temperature, 1));
    doc["chargerTemp"] = serialized(String(nlgData.temperature, 1));

    // Cell voltages
    doc["minCellVoltage"] = serialized(String(bmsData.minCellVoltage, 3));
    doc["maxCellVoltage"] = serialized(String(bmsData.maxCellVoltage, 3));

    String output;
    serializeJson(doc, output);
    return output;
}

String WebServer::getChargingLimitsJSON() {
    RuntimeConfigData config = sharedRuntimeConfig.get();
    BMSData bmsData = sharedBMSData.get();

    StaticJsonDocument<512> doc;

    // Calculate pack voltages from per-cell settings
    float storagePackV = getStoragePackVoltage(config);
    float maxChargePackV = getMaxChargePackVoltage(config);
    float bulkChargePackV = getBulkChargePackVoltage(config);

    // Calculate estimated DC current from AC mains limit at current voltage
    float currentPackV = bmsData.voltage / 10.0f;  // Convert to volts
    float estimatedDCCurrent = estimateDCCurrentFromMains(config.mainsCurrentLimit, currentPackV);

    // Calculate estimated charging power
    float estimatedPowerW = estimatedDCCurrent * currentPackV;
    float estimatedACPowerW = config.mainsCurrentLimit * 230.0f;

    doc["storagePackVoltage"] = serialized(String(storagePackV, 1));
    doc["maxChargePackVoltage"] = serialized(String(maxChargePackV, 1));
    doc["bulkChargePackVoltage"] = serialized(String(bulkChargePackV, 1));
    doc["maxDCCurrent"] = serialized(String(config.maxChargeCurrent, 1));
    doc["estimatedDCCurrent"] = serialized(String(estimatedDCCurrent, 1));
    doc["estimatedDCPower"] = serialized(String(estimatedPowerW, 0));
    doc["estimatedACPower"] = serialized(String(estimatedACPowerW, 0));
    doc["mainsCurrentLimit"] = serialized(String(config.mainsCurrentLimit, 1));
    doc["currentPackVoltage"] = serialized(String(currentPackV, 1));

    String output;
    serializeJson(doc, output);
    return output;
}

