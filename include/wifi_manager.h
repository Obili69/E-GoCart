#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <LittleFS.h>
#include "config.h"
#include "data_structures.h"

//=============================================================================
// WIFI MANAGER
// Handles dual-mode WiFi (AP + STA), credentials storage, auto-reconnect
//=============================================================================

class WiFiManager {
public:
    WiFiManager();

    /**
     * @brief Initialize WiFi manager
     * @return true if initialization successful
     */
    bool begin();

    /**
     * @brief Update WiFi manager (call from FreeRTOS task)
     */
    void update();

    /**
     * @brief Start Access Point mode
     * @return true if successful
     */
    bool startAP();

    /**
     * @brief Start Station mode (connect to existing WiFi)
     * @param ssid WiFi SSID
     * @param password WiFi password
     * @return true if connection successful
     */
    bool startSTA(const char* ssid, const char* password);

    /**
     * @brief Stop WiFi
     */
    void stop();

    /**
     * @brief Check if AP is running
     */
    bool isAPActive() { return apActive; }

    /**
     * @brief Check if STA is connected
     */
    bool isSTAConnected() { return staConnected; }

    /**
     * @brief Get AP IP address
     */
    String getAPIP();

    /**
     * @brief Get STA IP address
     */
    String getSTAIP();

    /**
     * @brief Get number of connected clients (AP mode)
     */
    uint8_t getClientCount();

    /**
     * @brief Save STA credentials to LittleFS
     */
    bool saveSTACredentials(const char* ssid, const char* password);

    /**
     * @brief Load STA credentials from LittleFS
     */
    bool loadSTACredentials(char* ssid, char* password);

    /**
     * @brief Enable/disable auto-reconnect (STA mode)
     */
    void setAutoReconnect(bool enable) { autoReconnect = enable; }

    /**
     * @brief Scan for available WiFi networks
     * @return JSON string with network list
     */
    String scanNetworks();

    /**
     * @brief Test connection to a WiFi network
     * @param ssid WiFi SSID
     * @param password WiFi password
     * @return JSON string with connection result
     */
    String testConnection(const char* ssid, const char* password);

    /**
     * @brief Get current connection status
     * @return JSON string with status info
     */
    String getConnectionStatus();

    /**
     * @brief Finish test connection and restore normal operation
     * @param saveCredentials If true, save the tested credentials
     */
    void finishTestConnection(bool saveCredentials);

private:
    bool apActive;
    bool staConnected;
    bool autoReconnect;

    unsigned long lastReconnectAttempt;
    uint32_t reconnectInterval;  // ms between reconnect attempts

    char savedSSID[32];
    char savedPassword[64];

    /**
     * @brief Attempt to reconnect to WiFi (STA mode)
     */
    void attemptReconnect();

    /**
     * @brief Load WiFi config from LittleFS
     */
    bool loadConfig();

    /**
     * @brief Save WiFi config to LittleFS
     */
    bool saveConfig();
};
