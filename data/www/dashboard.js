// WebSocket connection
let ws = null;
let reconnectTimer = null;
let telemetryData = {};
let canLogBuffer = [];
let errorLogBuffer = [];

// Chart data
let speedHistory = [];
let powerHistory = [];
const maxDataPoints = 50;

// Initialize on page load
window.addEventListener('load', () => {
    initWebSocket();
    loadConfig();
    setInterval(updateTaskStats, 2000);
    setInterval(updateUptime, 1000);
});

// WebSocket Functions
function initWebSocket() {
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const wsUrl = `${protocol}//${window.location.host}/ws`;

    ws = new WebSocket(wsUrl);

    ws.onopen = () => {
        console.log('WebSocket connected');
        updateConnectionStatus(true);
    };

    ws.onclose = () => {
        console.log('WebSocket disconnected');
        updateConnectionStatus(false);
        reconnectTimer = setTimeout(initWebSocket, 3000);
    };

    ws.onerror = (error) => {
        console.error('WebSocket error:', error);
        updateConnectionStatus(false);
    };

    ws.onmessage = (event) => {
        try {
            const data = JSON.parse(event.data);

            // Check if it's telemetry or task stats
            if (data.speed !== undefined) {
                handleTelemetryUpdate(data);
            } else if (data.tasks !== undefined) {
                handleTaskStatsUpdate(data);
            }
        } catch (e) {
            console.error('Failed to parse message:', e);
        }
    };
}

function updateConnectionStatus(connected) {
    const statusDot = document.getElementById('wsStatus');
    const statusText = document.getElementById('wsText');

    if (connected) {
        statusDot.classList.remove('offline');
        statusText.textContent = 'Connected';
    } else {
        statusDot.classList.add('offline');
        statusText.textContent = 'Disconnected';
    }
}

// Telemetry Update
function handleTelemetryUpdate(data) {
    telemetryData = data;

    // Update dashboard values
    updateElement('speed', Math.round(data.speed));
    updateElement('power', data.power.toFixed(1));
    updateElement('torqueDemand', data.torqueDemand);
    updateElement('soc', data.soc);
    updateElement('voltage', data.voltage);
    updateElement('current', data.current.toFixed(1));
    updateElement('minCell', data.minCell.toFixed(2));
    updateElement('tempMotor', Math.round(data.tempMotor));
    updateElement('tempInverter', Math.round(data.tempInverter));
    updateElement('tempBattery', Math.round(data.tempBattery));
    updateElement('throttle', Math.round(data.throttle));
    updateElement('regen', Math.round(data.regen));
    updateElement('brake', data.brake ? 'Yes' : 'No');
    updateElement('interlock', data.il ? 'Closed' : 'Open');

    // Update gear display
    updateGearDisplay(data.gear);

    // Update vehicle state
    updateVehicleState(data.state);

    // Update temperature warnings
    checkTemperatureWarnings(data);

    // Update charts
    updateCharts(data);

    // Log CAN activity
    logCANActivity(data);

    // Check for errors
    checkForErrors(data);
}

function updateElement(id, value) {
    const element = document.getElementById(id);
    if (element) {
        element.textContent = value;
    }
}

function updateGearDisplay(gear) {
    const gearDisplay = document.getElementById('gearDisplay');
    let gearText = 'N';
    let gearClass = 'gear-N';

    switch(gear) {
        case 0:
            gearText = 'N';
            gearClass = 'gear-N';
            break;
        case 1:
            gearText = 'D';
            gearClass = 'gear-D';
            break;
        case 2:
            gearText = 'R';
            gearClass = 'gear-R';
            break;
    }

    gearDisplay.textContent = gearText;
    gearDisplay.className = gearClass;
}

function updateVehicleState(state) {
    const stateNames = ['SLEEP', 'INIT', 'READY', 'DRIVE', 'CHARGING'];
    const stateElement = document.getElementById('vehicleState');
    if (stateElement && state >= 0 && state < stateNames.length) {
        stateElement.textContent = stateNames[state];
    }
}

function checkTemperatureWarnings(data) {
    // Motor temperature
    const motorTemp = document.getElementById('tempMotor');
    if (data.tempMotor > 120) {
        motorTemp.classList.add('danger');
    } else if (data.tempMotor > 100) {
        motorTemp.classList.add('warning');
    } else {
        motorTemp.classList.remove('warning', 'danger');
    }

    // Inverter temperature
    const inverterTemp = document.getElementById('tempInverter');
    if (data.tempInverter > 90) {
        inverterTemp.classList.add('danger');
    } else if (data.tempInverter > 80) {
        inverterTemp.classList.add('warning');
    } else {
        inverterTemp.classList.remove('warning', 'danger');
    }

    // Battery temperature
    const batteryTemp = document.getElementById('tempBattery');
    if (data.tempBattery > 50) {
        batteryTemp.classList.add('danger');
    } else if (data.tempBattery > 45) {
        batteryTemp.classList.add('warning');
    } else {
        batteryTemp.classList.remove('warning', 'danger');
    }

    // SOC warning
    const socElement = document.getElementById('soc');
    if (data.soc < 15) {
        socElement.classList.add('danger');
    } else if (data.soc < 25) {
        socElement.classList.add('warning');
    } else {
        socElement.classList.remove('warning', 'danger');
    }

    // Min cell voltage
    const minCellElement = document.getElementById('minCell');
    if (data.minCell < 3.0) {
        minCellElement.classList.add('danger');
    } else if (data.minCell < 3.2) {
        minCellElement.classList.add('warning');
    } else {
        minCellElement.classList.remove('warning', 'danger');
    }
}

function updateCharts(data) {
    // Add to history
    speedHistory.push(data.speed);
    powerHistory.push(data.power);

    // Keep only last N points
    if (speedHistory.length > maxDataPoints) {
        speedHistory.shift();
    }
    if (powerHistory.length > maxDataPoints) {
        powerHistory.shift();
    }

    // Draw charts (simple canvas drawing)
    drawLineChart('speedChart', speedHistory, '#4ade80', 0, 60);
    drawLineChart('powerChart', powerHistory, '#667eea', -20, 50);
}

function drawLineChart(canvasId, data, color, minY, maxY) {
    const canvas = document.getElementById(canvasId);
    if (!canvas) return;

    const ctx = canvas.getContext('2d');
    const width = canvas.offsetWidth;
    const height = canvas.offsetHeight;

    canvas.width = width;
    canvas.height = height;

    // Clear
    ctx.clearRect(0, 0, width, height);

    if (data.length < 2) return;

    // Draw grid
    ctx.strokeStyle = '#3a3a3a';
    ctx.lineWidth = 1;
    for (let i = 0; i <= 4; i++) {
        const y = (height / 4) * i;
        ctx.beginPath();
        ctx.moveTo(0, y);
        ctx.lineTo(width, y);
        ctx.stroke();
    }

    // Draw line
    ctx.strokeStyle = color;
    ctx.lineWidth = 2;
    ctx.beginPath();

    const xStep = width / (data.length - 1);
    const yRange = maxY - minY;

    data.forEach((value, index) => {
        const x = index * xStep;
        const normalizedValue = (value - minY) / yRange;
        const y = height - (normalizedValue * height);

        if (index === 0) {
            ctx.moveTo(x, y);
        } else {
            ctx.lineTo(x, y);
        }
    });

    ctx.stroke();

    // Draw current value
    const lastValue = data[data.length - 1];
    ctx.fillStyle = color;
    ctx.font = '14px Arial';
    ctx.fillText(lastValue.toFixed(1), 10, 20);
}

// CAN Monitor
function logCANActivity(data) {
    const timestamp = new Date().toLocaleTimeString();

    // Log BMS message
    if (data.bmsAlive) {
        addCANLog(timestamp, 'BMS', `SOC:${data.soc}% V:${data.voltage}V I:${data.current.toFixed(1)}A`);
    }

    // Log DMC message
    if (data.dmcReady) {
        addCANLog(timestamp, 'DMC', `Torque:${data.torqueActual.toFixed(1)}Nm Speed:${data.motorRPM.toFixed(0)}RPM`);
    }
}

function addCANLog(timestamp, id, message) {
    const logContainer = document.getElementById('canLog');
    if (!logContainer) return;

    // Add to buffer
    canLogBuffer.push({ timestamp, id, message });

    // Keep only last 100 entries
    if (canLogBuffer.length > 100) {
        canLogBuffer.shift();
    }

    // Update display (throttled)
    if (canLogBuffer.length % 5 === 0) {
        updateCANLogDisplay();
    }
}

function updateCANLogDisplay() {
    const logContainer = document.getElementById('canLog');
    if (!logContainer) return;

    logContainer.innerHTML = canLogBuffer.map(entry => {
        return `<div class="log-entry">
            <span class="log-timestamp">${entry.timestamp}</span>
            <span class="log-can">[${entry.id}]</span>
            ${entry.message}
        </div>`;
    }).join('');

    // Auto-scroll
    if (document.getElementById('canAutoScroll').checked) {
        logContainer.scrollTop = logContainer.scrollHeight;
    }
}

function clearCANLog() {
    canLogBuffer = [];
    document.getElementById('canLog').innerHTML = '<div class="log-entry">Log cleared</div>';
}

// Error Logging
function checkForErrors(data) {
    // Check for BMS offline
    if (!data.bmsAlive && data.state === 3) { // state 3 = DRIVE
        addErrorLog('BMS communication lost!');
    }

    // Check critical temperatures
    if (data.tempMotor > 140) {
        addErrorLog(`Motor overtemp: ${data.tempMotor.toFixed(0)}°C`);
    }
    if (data.tempInverter > 100) {
        addErrorLog(`Inverter overtemp: ${data.tempInverter.toFixed(0)}°C`);
    }

    // Check critical voltage
    if (data.minCell < 3.0) {
        addErrorLog(`Critical cell voltage: ${data.minCell.toFixed(2)}V`);
    }

    // Check interlock
    if (!data.il) {
        addErrorLog('Interlock open!');
    }
}

function addErrorLog(message) {
    const timestamp = new Date().toLocaleTimeString();

    // Check if error already logged recently
    const recentError = errorLogBuffer.find(e =>
        e.message === message && (Date.now() - e.time) < 5000
    );

    if (!recentError) {
        errorLogBuffer.push({ timestamp, message, time: Date.now() });
        updateErrorLogDisplay();
    }
}

function updateErrorLogDisplay() {
    const logContainer = document.getElementById('errorLog');
    if (!logContainer) return;

    if (errorLogBuffer.length === 0) {
        logContainer.innerHTML = '<div class="log-entry">No errors</div>';
        return;
    }

    logContainer.innerHTML = errorLogBuffer.map(entry => {
        return `<div class="log-entry">
            <span class="log-timestamp">${entry.timestamp}</span>
            <span class="log-error">[ERROR]</span>
            ${entry.message}
        </div>`;
    }).reverse().join('');
}

function clearErrorLog() {
    errorLogBuffer = [];
    updateErrorLogDisplay();
}

// Task Stats
function handleTaskStatsUpdate(data) {
    const tbody = document.getElementById('taskTableBody');
    if (!tbody) return;

    if (!data.tasks || data.tasks.length === 0) {
        tbody.innerHTML = '<tr><td colspan="5" style="text-align: center;">No tasks</td></tr>';
        return;
    }

    tbody.innerHTML = data.tasks.map(task => {
        const stateClass = task.state === 0 ? 'state-running' :
                          task.state === 1 ? 'state-ready' : 'state-blocked';
        const stateName = task.state === 0 ? 'Running' :
                         task.state === 1 ? 'Ready' : 'Blocked';
        const watchdogStatus = task.watchdog ? '✓' : '✗';
        const watchdogColor = task.watchdog ? '#4ade80' : '#ef4444';

        return `<tr>
            <td>${task.name}</td>
            <td>${task.priority}</td>
            <td>${task.stack} bytes</td>
            <td><span class="state-badge ${stateClass}">${stateName}</span></td>
            <td style="color: ${watchdogColor}">${watchdogStatus}</td>
        </tr>`;
    }).join('');

    // Update system stats
    updateElement('freeHeap', data.freeHeap);
    updateElement('cpuUsage', data.cpuUsage);
}

function updateTaskStats() {
    fetch('/api/tasks')
        .then(response => response.json())
        .then(data => handleTaskStatsUpdate(data))
        .catch(error => console.error('Failed to fetch task stats:', error));
}

// Configuration
function loadConfig() {
    fetch('/api/config')
        .then(response => response.json())
        .then(data => {
            document.getElementById('canFastCycle').value = data.canFastCycle || 10;
            document.getElementById('canSlowCycle').value = data.canSlowCycle || 100;
            document.getElementById('maxTorque').value = data.maxTorque || 850;
            document.getElementById('maxRegen').value = data.maxRegen || 420;
            document.getElementById('maxReverse').value = data.maxReverse || 200;
            document.getElementById('maxMotorTemp').value = data.maxMotorTemp || 140;
            document.getElementById('maxInverterTemp').value = data.maxInverterTemp || 100;
            document.getElementById('maxBatteryTemp').value = data.maxBatteryTemp || 55;
            document.getElementById('criticalCellV').value = data.criticalCellV || 3.0;
            document.getElementById('enableAP').checked = data.enableAP !== false;
            document.getElementById('enableSTA').checked = data.enableSTA || false;
            document.getElementById('staSSID').value = data.staSSID || '';
            document.getElementById('debugMode').checked = data.debugMode !== false;
            document.getElementById('enableOTA').checked = data.enableOTA || false;
        })
        .catch(error => console.error('Failed to load config:', error));
}

function saveConfig() {
    const config = {
        canFastCycle: parseInt(document.getElementById('canFastCycle').value),
        canSlowCycle: parseInt(document.getElementById('canSlowCycle').value),
        maxTorque: parseInt(document.getElementById('maxTorque').value),
        maxRegen: parseInt(document.getElementById('maxRegen').value),
        maxReverse: parseInt(document.getElementById('maxReverse').value),
        maxMotorTemp: parseFloat(document.getElementById('maxMotorTemp').value),
        maxInverterTemp: parseFloat(document.getElementById('maxInverterTemp').value),
        maxBatteryTemp: parseFloat(document.getElementById('maxBatteryTemp').value),
        criticalCellV: parseFloat(document.getElementById('criticalCellV').value),
        enableAP: document.getElementById('enableAP').checked ? 'true' : 'false',
        enableSTA: document.getElementById('enableSTA').checked ? 'true' : 'false',
        staSSID: document.getElementById('staSSID').value,
        staPassword: document.getElementById('staPassword').value,
        debugMode: document.getElementById('debugMode').checked ? 'true' : 'false',
        enableOTA: document.getElementById('enableOTA').checked ? 'true' : 'false'
    };

    // Build query string
    const params = new URLSearchParams(config).toString();

    fetch(`/api/config?${params}`, { method: 'POST' })
        .then(response => response.text())
        .then(message => {
            alert(message);
            loadConfig(); // Reload to confirm
        })
        .catch(error => {
            console.error('Failed to save config:', error);
            alert('Failed to save configuration');
        });
}

// OTA Functions
function enterOTAMode() {
    if (!confirm('Enter OTA mode? This will prepare the VCU for firmware updates.')) {
        return;
    }

    fetch('/api/ota/enter', { method: 'POST' })
        .then(response => response.text())
        .then(message => {
            document.getElementById('otaStatus').innerHTML =
                `<div class="alert alert-info">${message}</div>`;
        })
        .catch(error => {
            console.error('Failed to enter OTA mode:', error);
            alert('Failed to enter OTA mode');
        });
}

function exitOTAMode() {
    fetch('/api/ota/exit', { method: 'POST' })
        .then(response => response.text())
        .then(message => {
            document.getElementById('otaStatus').innerHTML =
                `<div class="alert alert-info">${message}</div>`;
        })
        .catch(error => {
            console.error('Failed to exit OTA mode:', error);
            alert('Failed to exit OTA mode');
        });
}

function restartVCU() {
    if (!confirm('Restart the VCU? This will disconnect all clients.')) {
        return;
    }

    fetch('/api/restart', { method: 'POST' })
        .then(() => {
            alert('VCU is restarting... Please wait 10 seconds and refresh the page.');
        })
        .catch(error => {
            console.error('Failed to restart:', error);
        });
}

// Tab switching
function switchTab(tabName) {
    // Hide all tabs
    document.querySelectorAll('.tab-content').forEach(tab => {
        tab.classList.remove('active');
    });

    // Remove active from all tab buttons
    document.querySelectorAll('.tab').forEach(btn => {
        btn.classList.remove('active');
    });

    // Show selected tab
    document.getElementById(tabName).classList.add('active');

    // Mark button as active
    event.target.classList.add('active');
}

// Uptime counter
let startTime = Date.now();
function updateUptime() {
    const uptime = Math.floor((Date.now() - startTime) / 1000);
    const hours = Math.floor(uptime / 3600);
    const minutes = Math.floor((uptime % 3600) / 60);
    const seconds = uptime % 60;

    const uptimeElement = document.getElementById('uptime');
    if (uptimeElement) {
        uptimeElement.textContent = `${hours}h ${minutes}m ${seconds}s`;
    }
}

// WiFi Scanning
let scannedNetworks = [];
let scanPollInterval = null;

function scanWiFi() {
    const button = document.getElementById('scanButtonText');
    const networkList = document.getElementById('networkList');
    const networkSelect = document.getElementById('networkSelect');

    // Show scanning state
    button.textContent = 'Scanning...';
    button.disabled = true;

    // Start the scan
    fetch('/api/wifi/scan')
        .then(response => response.json())
        .then(data => {
            if (data.status === 'started' || data.status === 'scanning') {
                // Scan initiated, start polling for results
                pollWiFiScanResults();
            } else if (data.status === 'complete') {
                // Results already available
                displayScanResults(data.networks);
            }
        })
        .catch(error => {
            console.error('WiFi scan error:', error);
            button.textContent = 'Scan Failed - Retry';
            button.disabled = false;
            alert('WiFi scan failed. Please try again.');
        });
}

function pollWiFiScanResults() {
    const button = document.getElementById('scanButtonText');

    // Clear any existing poll interval
    if (scanPollInterval) {
        clearInterval(scanPollInterval);
    }

    let pollCount = 0;
    const maxPolls = 20; // Max 10 seconds (500ms * 20)

    scanPollInterval = setInterval(() => {
        pollCount++;

        fetch('/api/wifi/scan')
            .then(response => response.json())
            .then(data => {
                if (data.status === 'complete') {
                    // Scan finished
                    clearInterval(scanPollInterval);
                    displayScanResults(data.networks);
                } else if (data.status === 'scanning') {
                    // Still scanning
                    button.textContent = `Scanning... (${Math.round(pollCount * 0.5)}s)`;
                } else if (pollCount >= maxPolls) {
                    // Timeout
                    clearInterval(scanPollInterval);
                    button.textContent = 'Scan Timeout - Retry';
                    button.disabled = false;
                }
            })
            .catch(error => {
                clearInterval(scanPollInterval);
                console.error('WiFi scan poll error:', error);
                button.textContent = 'Scan Failed - Retry';
                button.disabled = false;
            });
    }, 500); // Poll every 500ms
}

function displayScanResults(networks) {
    const button = document.getElementById('scanButtonText');
    const networkList = document.getElementById('networkList');
    const networkSelect = document.getElementById('networkSelect');

    scannedNetworks = networks;

    // Clear previous results
    networkSelect.innerHTML = '';

    // Sort by signal strength
    networks.sort((a, b) => b.quality - a.quality);

    // Populate network list
    networks.forEach((network, index) => {
        const option = document.createElement('option');
        option.value = index;

        // Signal strength indicator
        const bars = getSignalBars(network.quality);

        // Encryption indicator
        const lock = network.encryption !== 0 ? '🔒' : '';

        option.textContent = `${bars} ${network.ssid} ${lock} (${network.quality}%)`;
        networkSelect.appendChild(option);
    });

    // Show network list
    networkList.style.display = 'block';
    button.textContent = `Found ${networks.length} Networks - Rescan`;
    button.disabled = false;

    // Add selection handler
    networkSelect.onchange = function() {
        const index = parseInt(this.value);
        if (index >= 0 && index < scannedNetworks.length) {
            document.getElementById('staSSID').value = scannedNetworks[index].ssid;
            document.getElementById('enableSTA').checked = true;
            // Focus on password field
            document.getElementById('staPassword').focus();
        }
    };
}

function getSignalBars(quality) {
    if (quality >= 80) return '▂▄▆█';  // Excellent
    if (quality >= 60) return '▂▄▆_';  // Good
    if (quality >= 40) return '▂▄__';  // Fair
    if (quality >= 20) return '▂___';  // Weak
    return '____';                     // Very weak
}

// WiFi Connection Test
let testPollInterval = null;

function testWiFiConnection() {
    const buttonText = document.getElementById('testButtonText');
    const testButton = buttonText.parentElement;  // Get the actual button element
    const statusBox = document.getElementById('connectionStatus');
    const statusTitle = document.getElementById('statusTitle');
    const statusMessage = document.getElementById('statusMessage');

    const ssid = document.getElementById('staSSID').value;
    const password = document.getElementById('staPassword').value;

    if (!ssid) {
        alert('Please enter a WiFi network name (SSID)');
        return;
    }

    // Show testing state
    buttonText.textContent = 'Testing...';
    testButton.disabled = true;
    statusBox.style.display = 'block';
    statusBox.style.background = '#2a2a2a';
    statusBox.style.border = '1px solid #404040';
    statusTitle.textContent = 'Testing Connection...';
    statusMessage.textContent = 'Please wait...';

    // Build query string
    const params = new URLSearchParams({
        ssid: ssid,
        password: password
    }).toString();

    // Start connection test
    fetch(`/api/wifi/test?${params}`, { method: 'POST' })
        .then(response => response.json())
        .then(data => {
            if (data.status === 'testing') {
                // Test started, begin polling
                pollConnectionStatus(ssid);
            } else if (data.status === 'already_connected') {
                // Already connected to this network
                buttonText.textContent = 'Test Connection';
                testButton.disabled = false;
                statusBox.style.background = '#1a4d1a';
                statusBox.style.border = '1px solid #2d7a2d';
                statusTitle.textContent = '✓ Already Connected!';
                statusMessage.innerHTML = `You are already connected to <strong>${ssid}</strong>`;
                document.getElementById('enableSTA').checked = true;
            } else if (data.status === 'test_on_save') {
                // Can't test while connected to another network
                buttonText.textContent = 'Test Connection';
                testButton.disabled = false;
                statusBox.style.background = '#2a2a2a';
                statusBox.style.border = '1px solid #404040';
                statusTitle.textContent = 'ℹ Info';
                statusMessage.innerHTML = data.message || 'Cannot test while connected to another network.';
            }
        })
        .catch(error => {
            console.error('WiFi test error:', error);
            buttonText.textContent = 'Test Connection';
            testButton.disabled = false;

            statusBox.style.background = '#4d1a1a';
            statusBox.style.border = '1px solid #7a2d2d';
            statusTitle.textContent = '✗ Test Failed';
            statusMessage.textContent = 'Connection test failed. Please try again.';
        });
}

function pollConnectionStatus(ssid) {
    const buttonText = document.getElementById('testButtonText');
    const testButton = buttonText.parentElement;  // Get the actual button element
    const statusBox = document.getElementById('connectionStatus');
    const statusTitle = document.getElementById('statusTitle');
    const statusMessage = document.getElementById('statusMessage');

    // Clear any existing poll interval
    if (testPollInterval) {
        clearInterval(testPollInterval);
    }

    let pollCount = 0;
    const maxPolls = 30; // Max 15 seconds (500ms * 30)

    testPollInterval = setInterval(() => {
        pollCount++;

        fetch('/api/wifi/status')
            .then(response => response.json())
            .then(data => {
                if (data.success) {
                    // Connected successfully
                    clearInterval(testPollInterval);
                    buttonText.textContent = 'Test Connection';
                    testButton.disabled = false;

                    statusBox.style.background = '#1a4d1a';
                    statusBox.style.border = '1px solid #2d7a2d';
                    statusTitle.textContent = '✓ Connection Successful!';
                    statusMessage.innerHTML = `
                        Connected to <strong>${ssid}</strong><br>
                        IP Address: ${data.ip}<br>
                        Signal: ${data.rssi} dBm<br>
                        Hostname: ${data.hostname || 'egocart-vcu'}<br>
                        <br>
                        <small>You can now enable Station Mode and save configuration.</small>
                    `;

                    // Auto-enable station mode
                    document.getElementById('enableSTA').checked = true;

                    // Auto-hide after 30 seconds
                    setTimeout(() => {
                        statusBox.style.display = 'none';
                    }, 30000);

                } else if (data.message !== 'Connecting...') {
                    // Connection failed with error
                    clearInterval(testPollInterval);
                    buttonText.textContent = 'Test Connection';
                    testButton.disabled = false;

                    statusBox.style.background = '#4d1a1a';
                    statusBox.style.border = '1px solid #7a2d2d';
                    statusTitle.textContent = '✗ Connection Failed';
                    statusMessage.innerHTML = `
                        <strong>${data.message}</strong><br>
                        <br>
                        <small>Please check your credentials and try again.</small>
                    `;

                } else if (pollCount >= maxPolls) {
                    // Timeout
                    clearInterval(testPollInterval);
                    buttonText.textContent = 'Test Connection';
                    testButton.disabled = false;

                    statusBox.style.background = '#4d1a1a';
                    statusBox.style.border = '1px solid #7a2d2d';
                    statusTitle.textContent = '✗ Connection Timeout';
                    statusMessage.textContent = 'Connection attempt timed out. Please try again.';

                } else {
                    // Still connecting
                    statusMessage.textContent = `Connecting... (${Math.round(pollCount * 0.5)}s)`;
                }
            })
            .catch(error => {
                clearInterval(testPollInterval);
                console.error('Status poll error:', error);
                buttonText.textContent = 'Test Connection';
                testButton.disabled = false;
            });
    }, 500); // Poll every 500ms
}

//=============================================================================
// CHARGING FUNCTIONS
//=============================================================================

// Load charging configuration from VCU
async function loadChargingConfig() {
    try {
        const response = await fetch('/api/charging/config');
        const config = await response.json();
        
        // Update voltage fields
        document.getElementById('storageVoltagePerCell').value = parseFloat(config.storageVoltagePerCell);
        document.getElementById('maxChargeVoltagePerCell').value = parseFloat(config.maxChargeVoltagePerCell);
        document.getElementById('bulkChargeVoltagePerCell').value = parseFloat(config.bulkChargeVoltagePerCell);
        
        // Update current fields
        document.getElementById('maxChargeCurrent').value = parseFloat(config.maxChargeCurrent);
        document.getElementById('storageChargeCurrent').value = parseFloat(config.storageChargeCurrent);
        document.getElementById('mainsCurrentLimit').value = parseFloat(config.mainsCurrentLimit);
        updateMainsCurrentDisplay(config.mainsCurrentLimit);
        
        // Update safety fields
        document.getElementById('chargeTimeoutMinutes').value = parseInt(config.chargeTimeoutMinutes);
        document.getElementById('maxChargeTemp').value = parseFloat(config.maxChargeTemp);
        document.getElementById('minChargeTemp').value = parseFloat(config.minChargeTemp);
        document.getElementById('autoStopAtStorage').checked = config.autoStopAtStorage;
        document.getElementById('balancingEnabled').checked = config.balancingEnabled;
        
        // Update limits display
        updateChargingLimits();
        
        alert('✓ Charging configuration loaded successfully');
    } catch (error) {
        console.error('Failed to load charging config:', error);
        alert('✗ Failed to load charging configuration');
    }
}

// Save charging configuration to VCU
async function saveChargingConfig() {
    const config = {
        storageVoltagePerCell: parseFloat(document.getElementById('storageVoltagePerCell').value),
        maxChargeVoltagePerCell: parseFloat(document.getElementById('maxChargeVoltagePerCell').value),
        bulkChargeVoltagePerCell: parseFloat(document.getElementById('bulkChargeVoltagePerCell').value),
        maxChargeCurrent: parseFloat(document.getElementById('maxChargeCurrent').value),
        storageChargeCurrent: parseFloat(document.getElementById('storageChargeCurrent').value),
        mainsCurrentLimit: parseFloat(document.getElementById('mainsCurrentLimit').value),
        chargeTimeoutMinutes: parseInt(document.getElementById('chargeTimeoutMinutes').value),
        maxChargeTemp: parseFloat(document.getElementById('maxChargeTemp').value),
        minChargeTemp: parseFloat(document.getElementById('minChargeTemp').value),
        autoStopAtStorage: document.getElementById('autoStopAtStorage').checked,
        balancingEnabled: document.getElementById('balancingEnabled').checked
    };
    
    const params = new URLSearchParams(config);
    
    try {
        const response = await fetch('/api/charging/config?' + params.toString(), {
            method: 'POST'
        });
        const result = await response.json();
        
        if (result.status === 'ok') {
            alert('✓ Charging configuration saved successfully');
            updateChargingLimits(); // Refresh limits display
        } else {
            alert('✗ Error: ' + result.message);
        }
    } catch (error) {
        console.error('Failed to save charging config:', error);
        alert('✗ Failed to save charging configuration');
    }
}

// Update charging limits display
async function updateChargingLimits() {
    try {
        const response = await fetch('/api/charging/limits');
        const limits = await response.json();
        
        document.getElementById('storagePackVoltage').textContent = parseFloat(limits.storagePackVoltage).toFixed(1);
        document.getElementById('maxPackVoltage').textContent = parseFloat(limits.maxChargePackVoltage).toFixed(1);
        document.getElementById('maxDCCurrent').textContent = parseFloat(limits.maxDCCurrent).toFixed(1);
        document.getElementById('estimatedDCPower').textContent = Math.round(parseFloat(limits.estimatedDCPower));
        document.getElementById('mainsLimit').textContent = parseFloat(limits.mainsCurrentLimit).toFixed(1);
    } catch (error) {
        console.error('Failed to load charging limits:', error);
    }
}

// Update mains current display
function updateMainsCurrentDisplay(value) {
    const power = (value * 230 / 1000).toFixed(1);
    document.getElementById('mainsCurrentValue').textContent = `${value}A (${power}kW)`;
}

// Apply charging preset
async function applyPreset(preset) {
    const presetNames = ['Custom', 'Storage', 'Fast', 'Gentle', 'Eco'];
    const confirmMsg = `Apply ${presetNames[preset]} charging preset?`;
    
    if (!confirm(confirmMsg)) return;
    
    try {
        const response = await fetch(`/api/charging/preset?preset=${preset}`, {
            method: 'POST'
        });
        const result = await response.json();
        
        if (result.status === 'ok') {
            alert(`✓ ${presetNames[preset]} preset applied`);
            loadChargingConfig(); // Reload to show new values
        } else {
            alert('✗ Failed to apply preset');
        }
    } catch (error) {
        console.error('Failed to apply preset:', error);
        alert('✗ Failed to apply preset');
    }
}

// Start charging
async function startCharging() {
    if (!confirm('Start charging now?')) return;
    
    try {
        const response = await fetch('/api/charging/start', {
            method: 'POST'
        });
        const result = await response.json();
        
        if (result.status === 'ok') {
            alert('✓ Charging started');
            updateChargingStatus();
        } else {
            alert('✗ Failed to start charging: ' + result.message);
        }
    } catch (error) {
        console.error('Failed to start charging:', error);
        alert('✗ Failed to start charging');
    }
}

// Stop charging
async function stopCharging() {
    if (!confirm('Stop charging?')) return;
    
    try {
        const response = await fetch('/api/charging/stop', {
            method: 'POST'
        });
        const result = await response.json();
        
        if (result.status === 'ok') {
            alert('✓ Charging stopped');
            updateChargingStatus();
        } else {
            alert('✗ Failed to stop charging');
        }
    } catch (error) {
        console.error('Failed to stop charging:', error);
        alert('✗ Failed to stop charging');
    }
}

// Update charging status display
async function updateChargingStatus() {
    try {
        const response = await fetch('/api/charging/status');
        const status = await response.json();
        
        const stateNames = ['Idle', 'Precheck', 'Starting', 'Bulk Charge', 'Absorption', 'Balancing', 'Complete', 'Error', 'Disabled'];
        document.getElementById('chargerState').textContent = stateNames[status.chargerState] || 'Unknown';
        document.getElementById('chargeVoltage').textContent = (status.actualVoltage / 10).toFixed(1);
        document.getElementById('chargeCurrent').textContent = (status.actualCurrent / 10).toFixed(1);
        document.getElementById('chargePower').textContent = Math.round(status.chargePower);
        document.getElementById('chargeSoc').textContent = status.socPercent;
    } catch (error) {
        console.error('Failed to update charging status:', error);
    }
}

// Auto-load charging config and start periodic status updates when page loads
document.addEventListener('DOMContentLoaded', function() {
    // Load charging config if on charging tab
    if (document.getElementById('charging').classList.contains('active')) {
        loadChargingConfig();
    }
    
    // Update charging status every 2 seconds
    setInterval(updateChargingStatus, 2000);
    
    // Update charging limits every 5 seconds
    setInterval(updateChargingLimits, 5000);
});
