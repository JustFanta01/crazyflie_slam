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
 * 
 * my_app_slam.c - Code for extracting and sending via app_channel the data for the SLAM algorithm
 *  
 */

#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"
#include "app_channel.h"

#define DEBUG_MODULE "MY_APP_SLAM"
#include "debug.h"
#include "log.h"
#include "param.h"
#include "FreeRTOS.h"
#include "task.h"

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
    // NOTE: in the `src/modules/interface/crtp.h` the max CRTP data is defined as: CRTP_MAX_DATA_SIZE=30
    // char msg[10];
    SensorData_t s_data;
} __attribute__((packed));

void appMain() {
    struct testPacket xPacket;
    SensorData_t sensorData = {0};

    logVarId_t logIdStateEstimateYaw = logGetVarId("stateEstimate", "yaw");

    logVarId_t logIdStateEstimateX = logGetVarId("stateEstimate", "x");
    logVarId_t logIdStateEstimateY = logGetVarId("stateEstimate", "y");

    logVarId_t logIdRangeFront = logGetVarId("range", "front");
    logVarId_t logIdRangeBack = logGetVarId("range", "back");
    logVarId_t logIdRangeLeft = logGetVarId("range", "left");
    logVarId_t logIdRangeRight = logGetVarId("range", "right");
    logVarId_t logIdRangeUp = logGetVarId("range", "up");

    while (1) {
        // const char hi_string[10] = "SLAM data";
        
        sensorData.yaw = logGetFloat(logIdStateEstimateYaw);

        sensorData.x = logGetFloat(logIdStateEstimateX);
        sensorData.y = logGetFloat(logIdStateEstimateY);

        sensorData.front = logGetUint(logIdRangeFront);
        sensorData.back = logGetUint(logIdRangeBack);
        sensorData.left = logGetUint(logIdRangeLeft);
        sensorData.right = logGetUint(logIdRangeRight);
        sensorData.up = logGetUint(logIdRangeUp);

        // DEBUG_PRINT("front: %u", sensorData.front);

        // Serialize the sensor data into the CRTP packet
        // memcpy(xPacket.msg, hi_string, sizeof(hi_string));
        memcpy(&xPacket.s_data, &sensorData, sizeof(SensorData_t));

        // Send the CRTP packet
        appchannelSendDataPacketBlock(&xPacket, sizeof(xPacket));
        
        // Wait 25ms before sending the next packet (run at ~40Hz)
        vTaskDelay(pdMS_TO_TICKS(25));
    }
}
