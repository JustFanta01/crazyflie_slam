/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2019 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * LAB8: Optical Flow Sensor data communication to the host
 *
 * my_app.c - App layer application code
 *  
 */

#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"
#include "app_channel.h"

#define DEBUG_MODULE "HELLOWORLD"
#include "debug.h"
#include "log.h"
#include "param.h"
#include "static_mem.h"
#include "FreeRTOS.h"
#include "task.h"
#include "crtp.h"
#include "pmw3901.h" // Optical flow deck driver
#include "deck.h"    // For deck GPIO pin definitions

#define CUSTOM_CTRP_PORT 0x09 // Custom CRTP port for sensor data

#define NCS_PIN DECK_GPIO_IO3

// typedef struct {
//     int16_t deltaX;
//     int16_t deltaY;
//     uint8_t squal; // Surface quality
//     uint16_t shutter;
// } SensorData_t;

typedef struct
{
    // attitude
    float yaw;

    // state
    float x;
    float y;

    // obstacles
    uint16_t front;
    uint16_t back;
    uint16_t left;
    uint16_t right;
    uint16_t up;
} SensorData_t;


struct testPacket {
    char msg[10];
    SensorData_t s_data;
} __attribute__((packed));

void appMain() {
    DEBUG_PRINT("Initializing...\n");
    // Ensure the Flow Deck is initialized
    if ((pmw3901Init(NCS_PIN) == false)) { 
        DEBUG_PRINT("Failed to initialize PMW3901\n");
        vTaskDelete(NULL);
        return;
    }
    
    DEBUG_PRINT("Initialize finished.\n");
    struct testPacket xPacket;
    SensorData_t sensorData = {0};

    DEBUG_PRINT("This is MyApp!\n");

    logVarId_t logIdStateEstimateYaw = logGetVarId("stateEstimate", "yaw");

    logVarId_t logIdStateEstimateX = logGetVarId("stateEstimate", "x");
    logVarId_t logIdStateEstimateY = logGetVarId("stateEstimate", "y");

    logVarId_t logIdRangeFront = logGetVarId("range", "front");
    logVarId_t logIdRangeBack = logGetVarId("range", "back");
    logVarId_t logIdRangeLeft = logGetVarId("range", "left");
    logVarId_t logIdRangeRight = logGetVarId("range", "right");
    logVarId_t logIdRangeUp = logGetVarId("range", "up");

    // paramVarId_t idEstimator = paramGetVarId("stabilizer", "estimator");
    // uint8_t estimator_type = 0;

    // float yaw = 0.0f;

    // float x = 0.0f;
    // float y = 0.0f;

    // float front = 0.0f;
    // float back = 0.0f;
    // float left = 0.0f;
    // float right = 0.0f;
    // float up = 0.0f;

    while (1) {
        const char hi_string[10] = "SLAM data";
        // Read optical flow data
        // motionBurst_t currentMotion;
        // pmw3901ReadMotion(NCS_PIN, &currentMotion);
        // Negate the deltas to align with Crazyflie's frame of reference
        // sensorData.deltaX = -currentMotion.deltaX;
        // sensorData.deltaY = -currentMotion.deltaY;
        // sensorData.squal = currentMotion.squal;
        
        sensorData.yaw = logGetFloat(logIdStateEstimateYaw);

        sensorData.x = logGetFloat(logIdStateEstimateX);
        sensorData.y = logGetFloat(logIdStateEstimateY);

        sensorData.front = logGetUint(logIdRangeFront);
        sensorData.back = logGetUint(logIdRangeBack);
        sensorData.left = logGetUint(logIdRangeLeft);
        sensorData.right = logGetUint(logIdRangeRight);
        sensorData.up = logGetUint(logIdRangeUp);

        // Add another reading of a pwm3901 sensor data
        // Look at crazyflie-firmware/src/drivers/pwm3901.c crazyflie-firmware/src/drivers/interface/pwm3901.h 
        // and crazyflie-firmware/src/deck/drivers/src/flowdeck_v1v2.c for reference
        // sensorData.shutter = currentMotion.shutter;
        
        // Modify the sensorData struct to include the new data
        // Modify the xPacket struct to include the new data
        
        // Replace the hi_string msg with a description of the data sent ex: "Optical Flow Data" - also modify the size of the msg array
        // Modify the memcpy to include the new data

        // DEBUG_PRINT("deltaX: %d, deltaY: %d squal: %d, shutter: %d\n", sensorData.deltaX, sensorData.deltaY, sensorData.squal, sensorData.shutter);
        DEBUG_PRINT("front: %u", sensorData.front);


        // Serialize the sensor data into the CRTP packet
        memcpy(&xPacket.s_data, &sensorData, sizeof(SensorData_t));
        memcpy(xPacket.msg, hi_string, sizeof(hi_string));

        // Send the CRTP packet
        appchannelSendDataPacketBlock(&xPacket, sizeof(xPacket));
        // Wait 100ms before sending the next packet
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
