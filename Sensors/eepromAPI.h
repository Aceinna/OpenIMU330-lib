/** ***************************************************************************
 * @file eeprom_api.h 
 * @Author
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 *****************************************************************************/
#ifndef _EEPROM_API_H
#define _EEPROM_API_H

#include <stdint.h> 
#include "GlobalConstants.h"

extern void EEPROM_ReadWords(uint16_t addr, uint16_t num, void *destination) ;
extern BOOL EEPROM_WriteWords(uint16_t addr, uint16_t num, void *source) ;
extern void EEPROM_ReadByte(uint16_t addr, uint16_t num, void *destination) ;
extern BOOL EEPROM_WriteByte(uint16_t addr, uint16_t num, void *source) ;
extern void EEPROM_ReadSerialNumber(void *destination) ;
extern void EEPROM_ReadProdConfig(void *destination) ;
extern void EEPROM_ReadCalibration(int idx, void* destination);
extern void EEPROM_ReadFactoryConfiguration(void* destination);
extern BOOL EEPROM_ReadFromCalPartition( uint16_t offset, uint16_t num, void  *destination);
extern BOOL EEPROM_WriteToCalPartition(uint16_t offset, uint16_t num, void *source);
extern BOOL EEPROM_WriteToAppPartition(uint32_t offset, uint16_t num, void *source);
extern uint8_t *EEPROM_GetEEPROMCalTabPtr(int idx);
extern BOOL EEPROM_IsCalSectorsLocked();
extern BOOL EEPROM_LockCalSectors(void);
extern BOOL EEPROM_UnlockCalSectors(void *payload);
extern BOOL EEPROM_ApplyAppSignature(BOOL bootMode);
extern BOOL EEPROM_IsFactoryMode(void);
extern uint8_t *EEPROM_GetCalTabPtr(int idx);

#endif /* S_EEPROM_H */ 


