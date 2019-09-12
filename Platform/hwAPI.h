/** ***************************************************************************
 * @file hwAPI.h 
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

#ifndef __HW_API_H
#define __HW_API_H

#include "stdint.h"
#include "GlobalConstants.h"

void    HW_Init();

// GPIO - related fucntions
void    HW_IO2_Init();
void    HW_IO3_Init();
void    HW_IO3_Toggle();
void    HW_IO2_Toggle();
void    HW_IO2_On();
void    HW_IO3_On();
void    HW_IO2_Off();
void    HW_IO3_Off();
void    HW_DRDY_Off();
void    HW_DRDY_On();
uint8_t HW_ReadConfig();
BOOL    HW_IsTestOK();
BOOL    HW_IsTestMode();
void    HW_SetTestMode(BOOL fOn);

// SPI - related functions
int     SPI_ProcessCommand(uint8_t cmd);
void    SPI_ProcessData(uint8_t* in);
void    SPI_ProcessData1(uint8_t* in, uint8_t *statPtr);
int     SPI_PrepareForDataTransmit(uint8_t *out, int outLen);
void    SPI_ActivateInterface();
void    SPI_ProcessBootData(uint8_t* in, int len, uint8_t *statusAddr);
void    SPI_ActivateInterfaceForBootLoading(uint16_t statusWord);
void    SPI_DeactivateInterface();

// UART - related functions
BOOL    UART_Init(int port, int baudrate);
int     UART_Read(int port, uint8_t *buffer, uint32_t size);
int     UART_Write(int port, uint8_t *buffer, uint32_t size);

// CAN - related functions
void    CAN_Init();
BOOL    CAN_Transmit(uint32_t id, uint8_t *data, int len); 
void    CAN_ConfigureCallback(void (*callback)(void));

// Timers -related functions

int     StartDacqTimer(int freq);
int     TIMER_IsDacqOverrun();
int     TIMER_WaitForNewDacqTick();
int     TIMER_GetSensToSyncDelay();
int     TIMER_GetSyncToDrdyDelay();
void    StartReferenceTimer(int precisionUs);
void    ConfigureTimerForSyncCapture();

// system related functions
void    HW_SystemReset(void);
void    HW_JumpToApp();
void    HW_EnforceBootMode();
void    HW_EnforceAppMode();
BOOL    HW_IsBootModeEnforced();
BOOL    HW_IsAppModeEnforced();
void    HW_ClearBootSignature();


#define NUM_SERIAL_PORTS  2

#define USER_SERIAL_PORT   0
#define DEBUG_SERIAL_PORT  1

extern BOOL fSPI;
extern BOOL fUART;


#endif //__UART_H