#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "config.h"
#include "data_structures.h"

//=============================================================================
// TASK MONITOR & WATCHDOG
// Monitors all tasks, restarts failed tasks, collects CPU statistics
//=============================================================================

class TaskMonitor {
public:
    TaskMonitor();

    /**
     * @brief Initialize task monitor
     */
    void begin();

    /**
     * @brief Main monitor task (call from FreeRTOS task)
     */
    void update();

    /**
     * @brief Register a task for monitoring
     * @param handle Task handle
     * @param name Task name
     * @param timeout Watchdog timeout in milliseconds
     */
    void registerTask(TaskHandle_t handle, const char* name, uint32_t timeout);

    /**
     * @brief Feed the watchdog for a specific task (call from task periodically)
     * @param handle Task handle
     */
    void feedWatchdog(TaskHandle_t handle);

    /**
     * @brief Get statistics for all tasks
     * @param stats Array to store statistics
     * @param maxTasks Maximum number of tasks to report
     * @return Number of tasks reported
     */
    uint8_t getTaskStats(TaskStats* stats, uint8_t maxTasks);

    /**
     * @brief Get CPU usage percentage (0-100%)
     */
    uint8_t getTotalCPUUsage();

    /**
     * @brief Get free heap memory (bytes)
     */
    uint32_t getFreeHeap();

    /**
     * @brief Check if a specific task is healthy
     */
    bool isTaskHealthy(TaskHandle_t handle);

private:
    static constexpr uint8_t MAX_MONITORED_TASKS = 15;

    struct MonitoredTask {
        TaskHandle_t handle;
        char name[16];
        uint32_t timeout;           // Watchdog timeout (ms)
        unsigned long lastFeed;     // Last watchdog feed time
        uint32_t restartCount;      // Number of times restarted
        bool isHealthy;
    };

    MonitoredTask tasks[MAX_MONITORED_TASKS];
    uint8_t taskCount;

    // Statistics tracking
    uint32_t lastStatUpdate;
    uint32_t totalRunTime;
    TaskStatus_t taskStatusArray[MAX_MONITORED_TASKS];

    /**
     * @brief Check all watchdog timers
     */
    void checkWatchdogs();

    /**
     * @brief Restart a failed task
     */
    void restartTask(uint8_t index);

    /**
     * @brief Update CPU statistics
     */
    void updateCPUStats();

    /**
     * @brief Find task index by handle
     */
    int8_t findTaskIndex(TaskHandle_t handle);
};

//-----------------------------------------------------------------------------
// WATCHDOG HELPER MACRO
// Use in each task to feed watchdog
//-----------------------------------------------------------------------------
extern TaskMonitor taskMonitor;

#define FEED_WATCHDOG() taskMonitor.feedWatchdog(xTaskGetCurrentTaskHandle())
