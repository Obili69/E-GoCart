#include "wifi_manager.h"

WiFiManager::WiFiManager()
    : apActive(false)
    , staConnected(false)
    , autoReconnect(true)
    , lastReconnectAttempt(0)
    , reconnectInterval(5000)  // 5 seconds
{
    memset(savedSSID, 0, sizeof(savedSSID));
    memset(savedPassword, 0, sizeof(savedPassword));
}

bool WiFiManager::begin() {
    DEBUG_PRINTLN("WiFiManager: Initializing...");

    // Set hostname
    WiFi.setHostname(WiFi_Config::OTA_HOSTNAME);
    DEBUG_PRINTF("WiFi: Hostname set to %s\n", WiFi_Config::OTA_HOSTNAME);

    // Initialize LittleFS
    if (!LittleFS.begin(true)) {
        DEBUG_PRINTLN("ERROR: LittleFS mount failed!");
        return false;
    }
    DEBUG_PRINTLN("LittleFS mounted");

    // Load saved credentials
    loadConfig();

    // Get runtime config to determine WiFi mode
    RuntimeConfigData config = sharedRuntimeConfig.get();

    // Start in appropriate mode(s)
    if (config.enableAP) {
        startAP();
    }

    if (config.enableSTA && strlen(savedSSID) > 0) {
        startSTA(savedSSID, savedPassword);
    }

    DEBUG_PRINTLN("WiFiManager: Initialized");
    return true;
}

void WiFiManager::update() {
    // Check STA connection status
    if (autoReconnect && !staConnected) {
        unsigned long currentTime = millis();
        if (currentTime - lastReconnectAttempt >= reconnectInterval) {
            lastReconnectAttempt = currentTime;
            attemptReconnect();
        }
    }

    // Update connection status
    bool wasConnected = staConnected;
    staConnected = (WiFi.status() == WL_CONNECTED);

    // Log connection changes
    if (staConnected && !wasConnected) {
        DEBUG_PRINTLN("WiFi: STA connected!");
        DEBUG_PRINTF("  IP: %s\n", WiFi.localIP().toString().c_str());

        // Set system event flag
        xEventGroupSetBits(systemEvents, EVENT_WIFI_CONNECTED);
    } else if (!staConnected && wasConnected) {
        DEBUG_PRINTLN("WiFi: STA disconnected!");

        // Clear system event flag
        xEventGroupClearBits(systemEvents, EVENT_WIFI_CONNECTED);
    }

    // Yield to other tasks
    vTaskDelay(pdMS_TO_TICKS(100));
}

bool WiFiManager::startAP() {
    DEBUG_PRINTLN("WiFi: Starting Access Point...");

    // Configure AP
    WiFi.mode(staConnected ? WIFI_AP_STA : WIFI_AP);

    // Set static IP
    IPAddress local_IP;
    IPAddress gateway;
    IPAddress subnet;

    local_IP.fromString(WiFi_Config::AP_IP);
    gateway.fromString(WiFi_Config::AP_GATEWAY);
    subnet.fromString(WiFi_Config::AP_SUBNET);

    if (!WiFi.softAPConfig(local_IP, gateway, subnet)) {
        DEBUG_PRINTLN("ERROR: AP config failed!");
        return false;
    }

    // Start AP
    if (!WiFi.softAP(WiFi_Config::AP_SSID,
                     WiFi_Config::AP_PASSWORD,
                     WiFi_Config::AP_CHANNEL,
                     0,  // SSID not hidden
                     WiFi_Config::AP_MAX_CLIENTS)) {
        DEBUG_PRINTLN("ERROR: AP start failed!");
        return false;
    }

    apActive = true;

    DEBUG_PRINTF("WiFi: AP started\n");
    DEBUG_PRINTF("  SSID: %s\n", WiFi_Config::AP_SSID);
    DEBUG_PRINTF("  IP: %s\n", WiFi.softAPIP().toString().c_str());

    return true;
}

bool WiFiManager::startSTA(const char* ssid, const char* password) {
    if (strlen(ssid) == 0) {
        DEBUG_PRINTLN("WiFi: STA SSID empty, skipping");
        return false;
    }

    DEBUG_PRINTF("WiFi: Connecting to '%s'...\n", ssid);

    // Set mode
    WiFi.mode(apActive ? WIFI_AP_STA : WIFI_STA);

    // Set hostname
    WiFi.setHostname(WiFi_Config::OTA_HOSTNAME);

    // Connect
    WiFi.begin(ssid, password);

    // Wait up to 10 seconds for connection
    uint8_t attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        DEBUG_PRINT(".");
        attempts++;
    }
    DEBUG_PRINTLN();

    if (WiFi.status() == WL_CONNECTED) {
        staConnected = true;
        DEBUG_PRINTLN("WiFi: STA connected!");
        DEBUG_PRINTF("  IP: %s\n", WiFi.localIP().toString().c_str());

        // Save credentials
        strncpy(savedSSID, ssid, 31);
        strncpy(savedPassword, password, 63);

        return true;
    } else {
        DEBUG_PRINTLN("WiFi: STA connection failed!");
        staConnected = false;
        return false;
    }
}

void WiFiManager::stop() {
    DEBUG_PRINTLN("WiFi: Stopping...");

    if (apActive) {
        WiFi.softAPdisconnect(true);
        apActive = false;
    }

    if (staConnected || WiFi.status() != WL_DISCONNECTED) {
        WiFi.disconnect(true);
        staConnected = false;
    }

    WiFi.mode(WIFI_OFF);

    DEBUG_PRINTLN("WiFi: Stopped");
}

String WiFiManager::getAPIP() {
    return WiFi.softAPIP().toString();
}

String WiFiManager::getSTAIP() {
    return WiFi.localIP().toString();
}

uint8_t WiFiManager::getClientCount() {
    return WiFi.softAPgetStationNum();
}

bool WiFiManager::saveSTACredentials(const char* ssid, const char* password) {
    strncpy(savedSSID, ssid, 31);
    savedSSID[31] = '\0';

    strncpy(savedPassword, password, 63);
    savedPassword[63] = '\0';

    return saveConfig();
}

bool WiFiManager::loadSTACredentials(char* ssid, char* password) {
    if (strlen(savedSSID) > 0) {
        strcpy(ssid, savedSSID);
        strcpy(password, savedPassword);
        return true;
    }
    return false;
}

void WiFiManager::attemptReconnect() {
    if (strlen(savedSSID) == 0) {
        return;  // No saved credentials
    }

    if (WiFi.status() == WL_CONNECTED) {
        return;  // Already connected
    }

    DEBUG_PRINTLN("WiFi: Attempting to reconnect...");
    WiFi.reconnect();
}

bool WiFiManager::loadConfig() {
    File file = LittleFS.open("/wifi_config.dat", "r");
    if (!file) {
        DEBUG_PRINTLN("WiFi: No saved config found");
        return false;
    }

    // Read SSID
    file.readBytes(savedSSID, 32);
    savedSSID[31] = '\0';

    // Read password
    file.readBytes(savedPassword, 64);
    savedPassword[63] = '\0';

    file.close();

    DEBUG_PRINTF("WiFi: Loaded config (SSID: %s)\n", savedSSID);
    return true;
}

bool WiFiManager::saveConfig() {
    File file = LittleFS.open("/wifi_config.dat", "w");
    if (!file) {
        DEBUG_PRINTLN("ERROR: Failed to save WiFi config");
        return false;
    }

    // Write SSID
    file.write((uint8_t*)savedSSID, 32);

    // Write password
    file.write((uint8_t*)savedPassword, 64);

    file.close();

    DEBUG_PRINTLN("WiFi: Config saved");
    return true;
}

String WiFiManager::scanNetworks() {
    DEBUG_PRINTLN("WiFi: Scanning for networks...");

    // Check if scan is already in progress
    int16_t scanResult = WiFi.scanComplete();

    DEBUG_PRINTF("WiFi: Scan status: %d\n", scanResult);

    if (scanResult == WIFI_SCAN_RUNNING) {
        // Scan still in progress
        DEBUG_PRINTLN("WiFi: Scan already in progress");
        return "{\"status\":\"scanning\"}";
    }

    if (scanResult == WIFI_SCAN_FAILED) {
        // Previous scan failed - clean up and restart
        DEBUG_PRINTLN("WiFi: Previous scan failed - cleaning up");
        WiFi.scanDelete();
        delay(100);
        int16_t result = WiFi.scanNetworks(true);  // Start new async scan
        DEBUG_PRINTF("WiFi: Restart scan result: %d\n", result);
        return "{\"status\":\"started\"}";
    }

    // Start async scan (non-blocking)
    if (scanResult < 0) {  // No scan results available
        DEBUG_PRINTLN("WiFi: Starting new async scan");
        int16_t result = WiFi.scanNetworks(true);  // true = async
        DEBUG_PRINTF("WiFi: Scan start result: %d\n", result);
        return "{\"status\":\"started\"}";
    }

    // Scan completed, process results
    int numNetworks = scanResult;
    DEBUG_PRINTF("WiFi: Found %d networks\n", numNetworks);

    // Build JSON response
    String json = "{\"status\":\"complete\",\"networks\":[";

    for (int i = 0; i < numNetworks; i++) {
        if (i > 0) json += ",";

        json += "{";
        json += "\"ssid\":\"" + WiFi.SSID(i) + "\",";
        json += "\"rssi\":" + String(WiFi.RSSI(i)) + ",";
        json += "\"encryption\":" + String(WiFi.encryptionType(i)) + ",";

        // Calculate signal strength percentage
        int rssi = WiFi.RSSI(i);
        int quality = 0;
        if (rssi <= -100) {
            quality = 0;
        } else if (rssi >= -50) {
            quality = 100;
        } else {
            quality = 2 * (rssi + 100);
        }
        json += "\"quality\":" + String(quality);
        json += "}";
    }

    json += "]}";

    // Clean up
    WiFi.scanDelete();

    return json;
}

String WiFiManager::testConnection(const char* ssid, const char* password) {
    DEBUG_PRINTF("WiFi: Testing connection to '%s'...\n", ssid);

    // IMPORTANT: Don't disconnect if we have AP with active clients
    // This prevents TCP connection disruption that causes crashes
    uint8_t clientCount = WiFi.softAPgetStationNum();

    if (apActive && clientCount > 0) {
        DEBUG_PRINTF("WiFi: %d clients connected to AP, testing without disconnect\n", clientCount);

        // Check current STA status
        wl_status_t currentStatus = WiFi.status();

        if (currentStatus == WL_CONNECTED) {
            // Already connected to a STA network
            String currentSSID = WiFi.SSID();

            if (currentSSID == ssid) {
                // Already connected to this network
                DEBUG_PRINTLN("WiFi: Already connected to this network");
                return "{\"status\":\"already_connected\"}";
            }

            DEBUG_PRINTLN("WiFi: Currently connected to different network, will test on save");
            return "{\"status\":\"test_on_save\",\"message\":\"Currently connected to another network. Connection will be tested when you save configuration.\"}";
        }
    } else {
        // No active clients or no AP, safe to disconnect
        if (WiFi.status() == WL_CONNECTED) {
            DEBUG_PRINTLN("WiFi: Disconnecting from current STA network...");
            WiFi.disconnect(false, false);
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }

    // Temporarily disable auto-reconnect to avoid interference
    WiFi.setAutoReconnect(false);

    // Ensure we're in AP+STA mode to keep web dashboard accessible
    if (apActive) {
        WiFi.mode(WIFI_AP_STA);
    } else {
        WiFi.mode(WIFI_STA);
    }

    // Set hostname
    WiFi.setHostname(WiFi_Config::OTA_HOSTNAME);

    // Begin connection (non-blocking)
    WiFi.begin(ssid, password);

    DEBUG_PRINTLN("WiFi: Connection test started");

    // Return immediately with "testing" status
    return "{\"status\":\"testing\"}";
}

String WiFiManager::getConnectionStatus() {
    wl_status_t status = WiFi.status();

    String json = "{";

    // Connection state
    json += "\"status\":" + String(status) + ",";
    json += "\"connected\":" + String((status == WL_CONNECTED) ? "true" : "false") + ",";

    // Connection result
    if (status == WL_CONNECTED) {
        json += "\"success\":true,";
        json += "\"message\":\"Connected successfully!\",";
        json += "\"ip\":\"" + WiFi.localIP().toString() + "\",";
        json += "\"rssi\":" + String(WiFi.RSSI()) + ",";
        json += "\"ssid\":\"" + WiFi.SSID() + "\",";
        json += "\"hostname\":\"" + String(WiFi.getHostname()) + "\"";
    } else {
        json += "\"success\":false,";
        json += "\"message\":\"";

        switch (status) {
            case WL_NO_SSID_AVAIL:
                json += "Network not found. Check SSID.";
                break;
            case WL_CONNECT_FAILED:
                json += "Wrong password or connection rejected.";
                break;
            case WL_CONNECTION_LOST:
                json += "Connection lost.";
                break;
            case WL_DISCONNECTED:
                json += "Connecting...";
                break;
            case WL_IDLE_STATUS:
                json += "Connecting...";
                break;
            default:
                json += "Connection status: " + String(status);
                break;
        }
        json += "\",";
        json += "\"ip\":\"\",";
        json += "\"rssi\":0,";
        json += "\"ssid\":\"\",";
        json += "\"hostname\":\"" + String(WiFi.getHostname()) + "\"";
    }

    // AP info
    json += ",\"apActive\":" + String(apActive ? "true" : "false");
    json += ",\"apIP\":\"" + getAPIP() + "\"";

    json += "}";
    return json;
}

void WiFiManager::finishTestConnection(bool saveCredentials) {
    DEBUG_PRINTLN("WiFi: Finishing connection test");

    // Re-enable auto-reconnect
    WiFi.setAutoReconnect(true);

    wl_status_t status = WiFi.status();

    if (status == WL_CONNECTED) {
        // Test was successful
        staConnected = true;

        if (saveCredentials) {
            // Save the credentials
            String ssid = WiFi.SSID();
            String psk = WiFi.psk();

            if (ssid.length() > 0) {
                strncpy(savedSSID, ssid.c_str(), 31);
                savedSSID[31] = '\0';

                strncpy(savedPassword, psk.c_str(), 63);
                savedPassword[63] = '\0';

                saveConfig();
                DEBUG_PRINTF("WiFi: Saved credentials for '%s'\n", savedSSID);
            }
        }

        DEBUG_PRINTLN("WiFi: Test connection successful, keeping connection");
    } else {
        // Test failed, restore AP mode if needed
        DEBUG_PRINTLN("WiFi: Test connection failed");

        // Disconnect and go back to AP mode
        WiFi.disconnect(true);

        if (apActive) {
            WiFi.mode(WIFI_AP);
            DEBUG_PRINTLN("WiFi: Restored to AP-only mode");
        }

        staConnected = false;
    }
}
