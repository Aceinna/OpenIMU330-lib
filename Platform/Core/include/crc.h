/** ***************************************************************************
 * @file crc.h 
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

#ifndef CRC_H
#define CRC_H
#include <stdint.h>

typedef uint16_t CrcCcittType;
typedef uint32_t Crc32Type;
 
#define CRC_CCITT_LENGTH	2
#define CRC_32_LENGTH       4
 
#define CRC_CCITT_INITIAL_SEED	0x1d0f
#define CRC_32_INITIAL_SEED		0xffffffff 

extern CrcCcittType CrcCcitt			(const uint8_t data [], uint16_t length, const CrcCcittType seed);
extern Crc32Type    Crc32				(const uint8_t data [], uint16_t length, const Crc32Type seed); 
extern void         CrcCcittTypeToBytes	(CrcCcittType type, uint8_t bytes []);
extern CrcCcittType BytesToCrcCcittType (const uint8_t bytes []);
extern void         Crc32TypeToBytes    (Crc32Type type, uint8_t bytes []);
extern Crc32Type    BytesToCrc32Type    (const uint8_t bytes []);
uint16_t            initCRC_16bit       (uint16_t  v, uint16_t seed);

#endif

