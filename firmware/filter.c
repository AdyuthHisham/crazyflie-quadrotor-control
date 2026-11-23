#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <memory.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "filter.h"
#include "constants.h"

void filterTask(void *pvParameters) {
    struct FilterParams *params = (struct FilterParams*)pvParameters;

    double gyro_data[3];
    double acc_data[3];

    SemaphoreHandle_t sensors_sem = params->sensors_sem;
    SemaphoreHandle_t estimate_sem = params->estimate_sem;

    TickType_t xLastWakeTime = xTaskGetTickCount();

    // Previous estimates (roll, pitch, yaw)
    double estimate[3] = {0.0}; // roll, pitch, yaw

    const int sample_rate = 5; // ms

    // Complementary filter constants
    const double h = 0.001*sample_rate;          // 10 ms sample time
    const double alpha = 6;      // tunable parameter
    const double gamma = alpha / (h + alpha);
    const double rad2deg = 180.0 / M_PI;

    while (1) {
        // Copy sensor data safely (optionally use semaphore if race conditions)
        xSemaphoreTake(sensors_sem, portMAX_DELAY);
        memcpy(gyro_data, params->gyro_data, sizeof(gyro_data));
        memcpy(acc_data, params->acc_data, sizeof(acc_data));
        xSemaphoreGive(sensors_sem);

        // Compute angles from accelerometer (in degrees)
        double acc_roll  = atan2(acc_data[1], acc_data[2]) * rad2deg;
        double acc_pitch = atan2(-acc_data[0], sqrt(acc_data[1]*acc_data[1] + acc_data[2]*acc_data[2])) * rad2deg;

        // Complementary filter update
        estimate[0] = (1 - gamma) * acc_roll + gamma * (estimate[0] + h * gyro_data[0]);
        estimate[1] = (1 - gamma) * acc_pitch + gamma * (estimate[1] + h * gyro_data[1]);
        estimate[2] = 0; // yaw from gyro only

        // Store intermediate debug data (optional)
        params->log_data[0] = estimate[0];
        params->log_data[1] = estimate[1];
        params->log_data[2] = estimate[2]; // filtered yaw

        // Write filtered estimate
        xSemaphoreTake(estimate_sem, portMAX_DELAY);
        memcpy(params->estimate, estimate, sizeof(estimate));
        xSemaphoreGive(estimate_sem);

        // Wait 5 ms (100Hz)
        vTaskDelayUntil( &xLastWakeTime, (sample_rate / portTICK_PERIOD_MS) );
    }
}

