/** ***************************************************************************
 * @file platform_version.h
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *****************************************************************************/
/*******************************************************************************
Copyright 2018 ACEINNA, INC

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*******************************************************************************/
#ifndef PLATFORM_VERSION_H
#define PLATFORM_VERSION_H

// DO NOT CHANGE THESE NUMBERS FROM ZERO!  CAUSES A CONFLICT WITH
//   IMUTest RESULTING IN ACCELEROMETER VALUES THAT ARE FLIPPED (WHAT
//   SHOULD BE POSITIVE BECOMES NEGATIVE).
#define VERSION_MAJOR 0
#define VERSION_MINOR 0
#define VERSION_PATCH 0
#define VERSION_STAGE 0
#define VERSION_BUILD 0

#define VERSION_MAJOR_NUM 1
#define VERSION_MINOR_NUM 1
#define VERSION_PATCH_NUM 1
#define VERSION_STAGE_NUM 0
#define VERSION_BUILD_NUM 4

/// Software-version/part-number
// changing the version number changes the CRC for the EEPROM memory so to avoid
// BIT failure the CRC must be updated if the version number is incremented
// Note: if the part number is changed before calibration the cal calculates
// and sets the CRC in EEPROM
//
//                                    1         2
//                           12345678901234567890

#ifdef   IMU383
#define  SOFTWARE_PART      "5020-1398-01 1.1.4"   // 383  
#else
#define  SOFTWARE_PART      "5020-1800-01 1.1.4"   // 330
#endif


#define  SOFTWARE_PART_LEN  50
#define  VERSION_STR        SOFTWARE_PART 
#define  N_VERSION_STR      128

#endif
