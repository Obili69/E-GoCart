#pragma once
#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include <LittleFS.h>
#include "config.h"
#include "data_structures.h"

//=============================================================================
// WEB SERVER
// Handles HTTP server, WebSocket telemetry, REST API, OTA updates
//=============================================================================

class WebServer {
public:
    WebServer();

    /**
     * @brief Initialize web server
     * @return true if initialization successful
     */
    bool begin();

    /**
     * @brief Update web server (call from FreeRTOS task)
     */
    void update();

    /**
     * @brief Enable/disable OTA update mode
     */
    void setOTAMode(bool enable);

    /**
     * @brief Check if OTA mode is active
     */
    bool isOTAMode() { return otaMode; }

    /**
     * @brief Send telemetry update to all WebSocket clients
     */
    void broadcastTelemetry();

    /**
     * @brief Send task statistics to all WebSocket clients
     */
    void broadcastTaskStats();

private:
    AsyncWebServer server;
    AsyncWebSocket ws;

    bool otaMode;
    unsigned long lastTelemetryBroadcast;
    unsigned long lastTaskStatsBroadcast;

    // Setup handlers
    void setupRoutes();
    void setupWebSocket();
    void setupOTA();

    // HTTP handlers
    void handleRoot(AsyncWebServerRequest *request);
    void handleGetTelemetry(AsyncWebServerRequest *request);
    void handleGetConfig(AsyncWebServerRequest *request);
    void handleSetConfig(AsyncWebServerRequest *request);
    void handleGetTaskStats(AsyncWebServerRequest *request);
    void handleEnterOTAMode(AsyncWebServerRequest *request);
    void handleExitOTAMode(AsyncWebServerRequest *request);
    void handleRestart(AsyncWebServerRequest *request);
    void handleWiFiScan(AsyncWebServerRequest *request);
    void handleWiFiTest(AsyncWebServerRequest *request);
    void handleWiFiStatus(AsyncWebServerRequest *request);

    // Charging API handlers
    void handleGetChargingConfig(AsyncWebServerRequest *request);
    void handleSetChargingConfig(AsyncWebServerRequest *request);
    void handleSetChargingPreset(AsyncWebServerRequest *request);
    void handleGetChargingStatus(AsyncWebServerRequest *request);
    void handleStartCharging(AsyncWebServerRequest *request);
    void handleStopCharging(AsyncWebServerRequest *request);
    void handleGetChargingLimits(AsyncWebServerRequest *request);

    // WebSocket handlers
    void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
                         AwsEventType type, void *arg, uint8_t *data, size_t len);

    // Helper functions
    String getTelemetryJSON();
    String getConfigJSON();
    String getTaskStatsJSON();
    String getChargingConfigJSON();
    String getChargingStatusJSON();
    String getChargingLimitsJSON();
};
