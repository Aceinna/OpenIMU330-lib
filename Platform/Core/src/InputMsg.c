/** ***************************************************************************
 * @file input_msg.c 
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

#include "ucb_packet.h"
#include "ucb_packet_struct.h"
#include "crc16.h"
#include "hwAPI.h"

int ucbPort = 0;   



/// List of allowed input packet codes 
ucbPacketTableEntry_t ucbPackets[] = {		//       
    {UCB_PING,               0x5555504B},   //  "PK" 
    {UCB_ECHO,               0x55554348},   //  "CH" 
    {UCB_GET_PACKET,         0x55554750},   //  "GP" 
    {UCB_SET_FIELDS,         0x55555346},   //  "SF" 
    {UCB_GET_FIELDS,         0x55554746},   //  "GF" 
    {UCB_READ_FIELDS,        0x55555246},   //  "RF" 
    {UCB_WRITE_FIELDS,       0x55555746},   //  "WF" 
    {UCB_UNLOCK_EEPROM,      0x55555545},   //  "UE" 
    {UCB_READ_EEPROM,        0x55555245},   //  "RE" 
    {UCB_WRITE_EEPROM,       0x55555745},   //  "WE" 
    {UCB_SOFTWARE_RESET,     0x55555352},   //  "SR" 
    {UCB_WRITE_CAL,          0x55555743},   //  "WC" 
    {UCB_READ_CAL,           0x55555243},   //  "RC" 
    {UCB_WRITE_APP,          0x55555741},   //  "WA" 
    {UCB_J2BOOT,             0x55554A42},   //  "JB" 
    {UCB_J2IAP,              0x55554A49},   //  "JI" 
    {UCB_J2APP,              0x55554A41},   //  "JA" 
    {UCB_LOCK_EEPROM,        0x55554C45},   //  "LE" 
    {UCB_INPUT_PACKET_MAX,   0x00000000},    //  "  "

    {UCB_IDENTIFICATION,     0x55554944},   //  "ID" 
    {UCB_VERSION_DATA,       0x55555652},   //  "VR" 
    {UCB_VERSION_ALL_DATA,   0x55555641},   //  "VA" 
    {UCB_SCALED_0,           0x55555330},   //  "S0" 
    {UCB_SCALED_1,           0x55555331},   //  "S1" 
    {UCB_SCALED_M,           0x5555534D},   //  "SM" 
    {UCB_SCALED_N,           0x5555534E},   //  "SN" 
    {UCB_FACTORY_1,          0x55554631},   //  "F1" 
    {UCB_FACTORY_2,          0x55554632},   //  "F2" 
    {UCB_FACTORY_M,          0x5555464D},   //  "FM" 
    {UCB_ANGLE_1,            0x55554131},   //  "A1" 
    {UCB_TEST_0,             0x55555430},   //  "T0" 
    {UCB_PACKET_LAST,        0x00000000},    //  "  "
};


//uint8_t pktBuffer[512];
uint8_t dataBuffer[512];
UcbPacketStruct ucbPacket;

__attribute__((weak)) int checkUserPacketType(uint16_t code)
{
    return UCB_ERROR_INVALID_TYPE;
}

int GetUcbPacket(int uartPort)
{
    static int bytesInBuffer = 0, state = 0, crcError = 0, len = 0;
    static uint8_t *ptr;
    static uint16_t crcMsg = 0;
	static uint32_t sync = 0;
    unsigned char tmp;
	static unsigned int  pos = 0, synced = 0, type;
	uint16_t crcCalc, code;
    ucbPacketTableEntry_t *syncTable;
	
    
	while(1){
        if(!bytesInBuffer){
            bytesInBuffer = UART_Read(uartPort, dataBuffer, sizeof (dataBuffer));
            if(!bytesInBuffer){
                return 0; // nothing to do
            }
            pos = 0; 
        }
        tmp = dataBuffer[pos++];
        bytesInBuffer--;
        sync = (sync << 8) | tmp;
        syncTable = ucbPackets;
        if((sync & 0xffff0000) == 0x55550000){
        while(syncTable->type != UCB_INPUT_PACKET_MAX){
            if(syncTable->sync == sync){
                    type = syncTable->type;
                synced       = 1;
                break;
            }
            if(synced){
                break;
            }
            syncTable++;
        }
            if(!synced){
                code = sync & 0xFFFF; 
                type = checkUserPacketType(code);
                if(type != UCB_ERROR_INVALID_TYPE){
                    synced = 1;
                }
            }
        }
        if(synced){
            ucbPacket.packetType    = type;
            ucbPacket.payloadLength = 0;
            ucbPacket.code_MSB      = (sync >> 8) & 0xff;
            ucbPacket.code_LSB      = sync & 0xff;
	        state	                = 1;
			len                     = 0;
            synced = 0;
            continue;
        }
        switch(state){
        case 0:
            break;
        case 1:
            ucbPacket.payloadLength = tmp;
            if(tmp == 0){
                state = 3;  // crc next
            }else{
                state = 2;  // data next
                len   = 0;
            }
            ptr   = ucbPacket.payload;
            break;
        case 2:
            if(len++ > UCB_MAX_PAYLOAD_LENGTH){
                state = 0;
                break;
            }
            *ptr++ = tmp;
            if(len == ucbPacket.payloadLength){
                //crc next
                state  = 3;
                crcMsg = 0; 
            }
            break;
        case 3:
            crcMsg = tmp;
            *ptr++ = tmp;   
            state = 4;
            break;
        case 4:
            state   = 0;
            crcMsg  = crcMsg | ((uint16_t)tmp << 8);
            *ptr++  = tmp;   
            crcCalc = CalculateCRC((uint8_t*)&ucbPacket.code_MSB, len + 3);
            if(crcMsg != crcCalc){
                crcError++;
            }else {
                // process message here
               HandleUcbPacket (ucbPort, &ucbPacket);
               return 0;   // will come back later
            }
            break;
        default:
            while(1){}; // should not be here
        }
    }
}


void  ProcessUserCommands(int port)
{
    GetUcbPacket(ucbPort);
}
