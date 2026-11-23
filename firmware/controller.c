#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <memory.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "controller.h"
#include "constants.h"

void controlSystemTask(void *pvParameters) {
    struct ControlSystemParams *params =
        (struct ControlSystemParams*)pvParameters;

    TickType_t xLastWakeTime = xTaskGetTickCount();

    // we keep local copies of the global state + semaphores
    #define NUM_OUTPUTS 4
    #define NUM_INPUTS 5

    unsigned short motors[4];
    double gyro_data[3];
    double acc_data[3];
    double r_rpdy[3];
    double estimate[3] = {0.0};
    double refError[3] = {0.0};
    double ctrlInput[NUM_INPUTS] = {0.0}; //Maybe add the base thrust here 29483.0
    double baseThrust = 29483.0f;
    const double deg2rad = 1/(180.0 / M_PI);
   
    // LQ gain matrix
    float K[NUM_OUTPUTS][NUM_INPUTS] =  {
    {-0.0198f, -0.0020f, -0.0203f, -0.0024f, -0.0067f},
    {-0.0198f, -0.0020f,  0.0203f,  0.0024f,  0.0067f},
    { 0.0198f,  0.0020f,  0.0203f,  0.0024f, -0.0067f},
    { 0.0198f,  0.0020f, -0.0203f, -0.0024f,  0.0067f}
    };

    // copy the semaphore handles for convenience
    SemaphoreHandle_t motors_sem = params->motors_sem;
    SemaphoreHandle_t references_sem = params->references_sem;
    SemaphoreHandle_t sensors_sem = params->sensors_sem;
    SemaphoreHandle_t estimate_sem = params->estimate_sem;

    while(1) {
        // read sensor data (gyro)
        xSemaphoreTake(sensors_sem, portMAX_DELAY);
        memcpy(gyro_data, params->gyro_data, sizeof(gyro_data));
        xSemaphoreGive(sensors_sem);

        // read filter data (angle estimates)
        xSemaphoreTake(estimate_sem, portMAX_DELAY);
        memcpy(estimate, params->estimate, sizeof(estimate));
        xSemaphoreGive(estimate_sem);

        // read latest references
        xSemaphoreTake(references_sem, portMAX_DELAY);
        memcpy(r_rpdy, params->r_rpdy, sizeof(r_rpdy));
        xSemaphoreGive(references_sem);

        // compute error
        refError[0] = estimate[0] - r_rpdy[0]; // roll
        refError[1] = estimate[1] - r_rpdy[1]; // pitch
        refError[2] = gyro_data[2] - r_rpdy[2]; // yaw rate


        // example of how to log some intermediate calculation
        // and use the provided constants
        


        // compute motor outputs
        ctrlInput[0] = refError[0]*deg2rad; // reference roll
        ctrlInput[1] = gyro_data[0]*deg2rad; // roll rate
        ctrlInput[2] = refError[1]*deg2rad; // reference pitch
        ctrlInput[3] = gyro_data[1]*deg2rad; // pitch rate
        ctrlInput[4] = refError[2]*deg2rad; // yaw rate reference
        // Compute motor outputs using LQ gain
        for (int i = 0; i < NUM_OUTPUTS; i++) {
            float control = 0.0f;
            for (int j = 0; j < NUM_INPUTS; j++) {
                control -= K[i][j] * ctrlInput[j];
            }
            control = control * (1 / crazyflie_constants.pwm_n); // convert to pwm
            control += baseThrust;
            
            // Clamp and convert to unsigned short
            if (control < 0.0f) control = 0.0f;
            if (control > 65535.0f) control = 65535.0f;
            motors[i] = (uint16_t)roundf(control);
        }



        // write motor output
        xSemaphoreTake(motors_sem, portMAX_DELAY);
        memcpy(params->motors, motors, sizeof(motors));
        xSemaphoreGive(motors_sem);

        // sleep 10ms to make this task run at 100Hz
        vTaskDelayUntil( &xLastWakeTime, (10 / portTICK_PERIOD_MS) );
    }
    
}
