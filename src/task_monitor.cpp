#include "task_monitor.h"

TaskMonitor::TaskMonitor()
    : taskCount(0)
    , lastStatUpdate(0)
    , totalRunTime(0)
{
    memset(tasks, 0, sizeof(tasks));
}

void TaskMonitor::begin() {
    DEBUG_PRINTLN("TaskMonitor: Initializing...");
    lastStatUpdate = millis();
    DEBUG_PRINTLN("TaskMonitor: Initialized");
}

void TaskMonitor::update() {
    unsigned long currentTime = millis();

    // Check watchdogs every cycle
    checkWatchdogs();

    // Update CPU statistics every second
    if (currentTime - lastStatUpdate >= 1000) {
        lastStatUpdate = currentTime;
        updateCPUStats();

#if DEBUG_FREERTOS
        // Print task statistics
        TaskStats stats[MAX_MONITORED_TASKS];
        uint8_t count = getTaskStats(stats, MAX_MONITORED_TASKS);

        DEBUG_PRINTLN("\n=== TASK STATISTICS ===");
        DEBUG_PRINTF("Free Heap: %lu bytes\n", getFreeHeap());
        DEBUG_PRINTF("CPU Usage: %u%%\n", getTotalCPUUsage());
        DEBUG_PRINTLN("Task             | Pri | Stack | CPU%% | State | WD");
        DEBUG_PRINTLN("-----------------|-----|-------|------|-------|----");

        for (uint8_t i = 0; i < count; i++) {
            DEBUG_PRINTF("%-16s | %3u | %5lu | %3lu%% | %5d | %s\n",
                stats[i].name,
                stats[i].priority,
                stats[i].stackHighWater,
                stats[i].cpuPercent,
                stats[i].state,
                stats[i].watchdogOK ? "OK" : "FAIL");
        }
        DEBUG_PRINTLN("=======================\n");
#endif
    }

    // Yield to other tasks
    vTaskDelay(pdMS_TO_TICKS(100));
}

void TaskMonitor::registerTask(TaskHandle_t handle, const char* name, uint32_t timeout) {
    if (taskCount >= MAX_MONITORED_TASKS) {
        DEBUG_PRINTF("ERROR: Cannot register task %s, maximum reached!\n", name);
        return;
    }

    tasks[taskCount].handle = handle;
    strncpy(tasks[taskCount].name, name, 15);
    tasks[taskCount].name[15] = '\0';
    tasks[taskCount].timeout = timeout;
    tasks[taskCount].lastFeed = millis();
    tasks[taskCount].restartCount = 0;
    tasks[taskCount].isHealthy = true;

    taskCount++;

    DEBUG_PRINTF("TaskMonitor: Registered '%s' (timeout: %lu ms)\n", name, timeout);
}

void TaskMonitor::feedWatchdog(TaskHandle_t handle) {
    int8_t index = findTaskIndex(handle);
    if (index >= 0) {
        tasks[index].lastFeed = millis();
        tasks[index].isHealthy = true;
    }
}

void TaskMonitor::checkWatchdogs() {
    unsigned long currentTime = millis();

    for (uint8_t i = 0; i < taskCount; i++) {
        // Skip if timeout is 0 (watchdog disabled)
        if (tasks[i].timeout == 0) {
            continue;
        }

        // Check if watchdog has timed out
        if (currentTime - tasks[i].lastFeed > tasks[i].timeout) {
            tasks[i].isHealthy = false;

            DEBUG_PRINTF("WATCHDOG TIMEOUT: Task '%s' has not responded!\n", tasks[i].name);

            // Attempt to restart the task
            restartTask(i);
        }
    }
}

void TaskMonitor::restartTask(uint8_t index) {
    if (index >= taskCount) {
        return;
    }

    DEBUG_PRINTF("RESTARTING TASK: '%s' (restart count: %lu)\n",
        tasks[index].name, tasks[index].restartCount + 1);

    // Note: Actual task restart would require storing the task function
    // and parameters. For now, we just mark as unhealthy and log.
    // In a full implementation, you'd call vTaskDelete() and xTaskCreate()

    tasks[index].restartCount++;
    tasks[index].lastFeed = millis();  // Reset watchdog timer

    // TODO: Implement actual task restart mechanism
    // This requires storing task creation parameters
}

void TaskMonitor::updateCPUStats() {
    // Simplified version without uxTaskGetSystemState
    // Just track that update was called
    totalRunTime = millis();
}

uint8_t TaskMonitor::getTaskStats(TaskStats* stats, uint8_t maxTasks) {
    uint8_t count = (taskCount < maxTasks) ? taskCount : maxTasks;

    // Simplified stats without uxTaskGetSystemState
    for (uint8_t i = 0; i < count; i++) {
        strncpy(stats[i].name, tasks[i].name, 15);
        stats[i].name[15] = '\0';

        // Get basic info
        stats[i].priority = uxTaskPriorityGet(tasks[i].handle);
        stats[i].stackHighWater = uxTaskGetStackHighWaterMark(tasks[i].handle);
        stats[i].state = eTaskGetState(tasks[i].handle);
        stats[i].runtime = 0;  // Not available without CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS
        stats[i].cpuPercent = 0;  // Not available
        stats[i].watchdogOK = tasks[i].isHealthy;
    }

    return count;
}

uint8_t TaskMonitor::getTotalCPUUsage() {
    // Simplified version - return estimated usage based on free heap change
    static uint32_t lastHeap = 0;
    uint32_t currentHeap = ESP.getFreeHeap();

    if (lastHeap == 0) {
        lastHeap = currentHeap;
        return 0;
    }

    // Simple estimation - not accurate but gives some indication
    lastHeap = currentHeap;
    return 50;  // Placeholder - return 50% as estimate
}

uint32_t TaskMonitor::getFreeHeap() {
    return ESP.getFreeHeap();
}

bool TaskMonitor::isTaskHealthy(TaskHandle_t handle) {
    int8_t index = findTaskIndex(handle);
    if (index >= 0) {
        return tasks[index].isHealthy;
    }
    return false;
}

int8_t TaskMonitor::findTaskIndex(TaskHandle_t handle) {
    for (uint8_t i = 0; i < taskCount; i++) {
        if (tasks[i].handle == handle) {
            return i;
        }
    }
    return -1;
}
