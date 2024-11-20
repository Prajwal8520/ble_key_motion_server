/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*
 * DEFINES
 ****************************************************************************************
 */

#define HRPS_HT_MEAS_MAX_LEN (13)

#define HRPS_MANDATORY_MASK (0x0F)
#define HRPS_BODY_SENSOR_LOC_MASK (0x30)
#define HRPS_HR_CTNL_PT_MASK (0xC0)

// /*service and characteristics UUID define*/
// static const uint8_t BIKE_SERVICE_UUID[16] = {0x11, 0xab, 0x5e, 0x32, 0xf4, 0x1c, 0x94, 0xae, 0x7d, 0x2a, 0xa9, 0x5b, 0x34, 0x67, 0xbb, 0x9f};
// static const uint8_t SPEED_CHARACTERSTICS_UUID[16] = {0x12, 0xab, 0x5e, 0x32, 0xf4, 0x1c, 0x94, 0xae, 0x7d, 0x2a, 0xa9, 0x5b, 0x34, 0x67, 0xbb, 0x9f};
// static const uint8_t  GPS_LOCATION_CHARACTERSITICS_UUID[16] = {0x13, 0xab, 0x5e, 0x32, 0xf4, 0x1c, 0x94, 0xae, 0x7d, 0x2a, 0xa9, 0x5b, 0x34, 0x67, 0xbb, 0x9f};
// static const uint8_t UPLOAD_POINT_CHARACTERISTICS_UUID[16]   = {0x14, 0xab, 0x5e, 0x32, 0xf4, 0x1c, 0x94, 0xae, 0x7d, 0x2a, 0xa9, 0x5b, 0x34, 0x67, 0xbb, 0x9f};
// /*service and characteristics UUID define*/

/// Attributes State Machine
enum
{
    key_motion_service,

    bike_characteristics_index,
    bike_characteristics_value,
    bike_characteristics_descriptor,

    key_characteristics_index,
    key_characteristics_value,
    key_characteristics_descriptor,

    control_point_characteristics_index,
    control_point_characteristics_value,
    control_point_characteristics_descriptor,

    HRS_IDX_NB,
};
