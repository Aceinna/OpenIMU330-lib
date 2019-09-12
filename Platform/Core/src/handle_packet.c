/** ***************************************************************************
 * @file handle_packet.c functions for handling serial UCB packets and CRM
 *       packets
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
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

//********************************
#include <stdint.h>
#include "ucb_packet.h"
#include "parameters.h"
#include "eepromAPI.h"
#include "crc16.h"
#include "osapi.h"
#include "BITStatus.h"
#include "calibrationAPI.h"
#include "configurationAPI.h"
#include "hwAPI.h"
#include "platformAPI.h"

static void _SetNak(uint16_t port, UcbPacketStruct *ptrUcbPacket);

void WriteMagAlignParamsToMemory( uint16_t port, UcbPacketStruct    *ptrUcbPacket );

__attribute__((weak)) int HandleUserInputPacket(UcbPacketStruct *ptrUcbPacket)
{
    return USER_PACKET_ERROR;
}


UcbPacketType UcbPacketBytesToPacketType(uint8_t *bytes, BOOL fGet)
{
    uint8_t ref[2];
    ucbPacketTableEntry_t *packetEntry = ucbPackets;
    uint16_t code;
    
    if(!fGet){
        code = (bytes[0] << 8) | bytes[1]; 
    }else {
        code = (bytes[1] << 8) | bytes[0]; 
    }
    
    while(packetEntry->type != UCB_PACKET_LAST){
        ref[0] = (packetEntry->sync >> 8) & 0xff;
        ref[1] = packetEntry->sync & 0xff;
        if(ref[0] == bytes[1] && ref[1] == bytes[0]){
            return (UcbPacketType)packetEntry->type;
        }
        packetEntry++;
    }
    
    return checkUserPacketType(code);
}

/** ****************************************************************************
 * @name UcbPacketIsAnOutputPacket API
 * @brief Returns TRUE if given packet type is an output packet type
 * Trace: [SDD_PORT_CFG_VALID_03 <-- SRC_UCB_PKT_OUTTYPE]
 * @param [in] UCB packet type
 * @retval TRUE if output packet type, FALSE otherwise
 ******************************************************************************/
BOOL UcbPacketIsAnOutputPacket (UcbPacketType type)
{
	BOOL isAnOutputPacket;
    int tt = type;
	switch (tt) {
        case UCB_IDENTIFICATION:
        case UCB_VERSION_DATA:
        case UCB_VERSION_ALL_DATA:
        case UCB_SCALED_0:
        case UCB_SCALED_1:
        case UCB_SCALED_M:
        case UCB_SCALED_N:
        case UCB_TEST_0:
        case UCB_FACTORY_1:
        case UCB_FACTORY_2:
        case UCB_FACTORY_M:
        case UCB_PING:
        case UCB_ANGLE_1:
        case UCB_USER_OUT:
            isAnOutputPacket = TRUE;
            break;
		default:
          isAnOutputPacket = FALSE;
	}
	return isAnOutputPacket;
}



/** ****************************************************************************
 * @name _UcbPing
 * @brief Reply to a PING command
 * Trace: [SDD_UCB_PING <-- SRC_UCB_PING]
 * @param [in] port - number request came in on, the reply will go out this port
 * @param [out] packetPtr - data part of packet
 * @retval N/A
 ******************************************************************************/
static void _UcbPing (uint16_t port,
                      UcbPacketStruct    *ptrUcbPacket)
{
	ptrUcbPacket->payloadLength = 0; /// return ping acknowledgement
	HandleUcbTx(port, ptrUcbPacket);
}

/** ****************************************************************************
 * @name _UcbEcho
 * @brief Reply to an ECHO command
 * Trace: [SDD_UCB_ECHO <-- SRC_UCB_ECHO]
 * @param [in] port - number request came in on, the reply will go out this port
 * @param [out] packetPtr - data part of packet
 * @retval N/A
 ******************************************************************************/
static void _UcbEcho (uint16_t port,
                      UcbPacketStruct    *ptrUcbPacket)
{
	HandleUcbTx(port, ptrUcbPacket);
}

/** ****************************************************************************
 * @name _UcbGetPacket
 * @brief Reply with the requested packet if it is an output packet type
 * Trace:
 *	[SDD_UCB_GETPACKET <-- SRC_UCB_GETPACKET]
 *	[SDD_RESP_ERROR <-- SRC_UCB_GETPACKET]
 * @param [in] port - number request came in on, the reply will go out this port
 * @param [out] packetPtr - data part of packet
 * @retval N/A
 ******************************************************************************/
static void _UcbGetPacket (uint16_t port,
                           UcbPacketStruct    *ptrUcbPacket)
{
    UcbPacketType requestedType;
    uint8_t bytes[2];
    
	if (ptrUcbPacket->payloadLength == 2) {
        bytes[0] = ptrUcbPacket->payload[1];
        bytes[1] = ptrUcbPacket->payload[0];
        
		requestedType = UcbPacketBytesToPacketType(bytes, TRUE);

		if (UcbPacketIsAnOutputPacket(requestedType) == TRUE) {
			ptrUcbPacket->packetType = requestedType; ///< response packet type
		 	SendUcbPacket(port, ptrUcbPacket); ///< generic response packet handler
            return;
		} else {
            _SetNak(port, ptrUcbPacket);
		}
	} else {
        _SetNak(port, ptrUcbPacket);
	}
	HandleUcbTx(port, ptrUcbPacket);
}

/** ****************************************************************************
 * @name _UcbSetFields
 * @brief handles a UCB set fields command packet
 * Trace:
 * [SDD_UCB_SETFIELDS <-- SRC_UCB_SETFIELDS]
 * [SDD_UCB_SETFIELDS_ID <-- SRC_UCB_SETFIELDS]
 * [SDD_UCB_SETFIELDS_DATA <-- SRC_UCB_SETFIELDS]
 * [SDD_UCB_SETFIELDS_NAK1 <-- SRC_UCB_SETFIELDS]
 * [SDD_UCB_SETFIELDS_NAK2 <-- SRC_UCB_SETFIELDS]
 * [SDD_UCB_SETFIELDS_PAIR_VALID <-- SRC_UCB_SETFIELDS]
 * @param [in] port - number request came in on, the reply will go out this port
 * @param [out] packetPtr - data part of packet
 * @retval N/A
 ******************************************************************************/
void _UcbSetFields (uint16_t port,
                           UcbPacketStruct    *ptrUcbPacket)

{
	uint8_t numFields = ptrUcbPacket->payload[0];
	uint8_t fieldCount;
	uint8_t validFieldCount;
    /** some fields need to be set together, so collect all field ID's and data
        in one set of arrays */
	uint16_t fieldId   [UCB_MAX_PAYLOAD_LENGTH / 4];
    /// array sizes are based on maximum number of fields to change
	uint16_t fieldData [UCB_MAX_PAYLOAD_LENGTH / 4];

	/// verify that the packet length matches packet specification
    if ((numFields > 0) &&
    	(ptrUcbPacket->payloadLength == (1 + numFields * 4)))
    {
	    /// loop through all fields and data specified in set fields request
	    for (fieldCount = 0; fieldCount < numFields; ++fieldCount) {
	    	/// read field ID and field data from packet into usable arrays
            fieldId[fieldCount]   = (uint16_t)((ptrUcbPacket->payload[(fieldCount * 4) + 1] << 8) |
                                                ptrUcbPacket->payload[(fieldCount * 4) + 2]);
            fieldData[fieldCount] = (uint16_t)((ptrUcbPacket->payload[(fieldCount * 4) + 3] << 8) |
                                                ptrUcbPacket->payload[(fieldCount * 4) + 4]);
	    }

	    validFieldCount = CheckRamFieldData(numFields, fieldId, fieldData, fieldId);
		if (validFieldCount > 0) {	/// all or some requested field changes valid?
			/// build and send positive acknowledgement packet
			ptrUcbPacket->payloadLength = (uint8_t)(1 + (validFieldCount * 2));
	    	ptrUcbPacket->payload[0]    = validFieldCount; /// number of valid fields

			/// place valid field ID's in payload
			for (fieldCount = 0; fieldCount < validFieldCount; ++fieldCount) {
				ptrUcbPacket->payload[(fieldCount * 2) + 1] = (uint8_t)((fieldId[fieldCount] >> 8) & 0xff);
				ptrUcbPacket->payload[(fieldCount * 2) + 2] = (uint8_t)( fieldId[fieldCount]       & 0xff);
			}
	        HandleUcbTx(port, ptrUcbPacket); ///< send acknowledgement
		}

		/// any invalid requested field changes?
		if (validFieldCount < numFields) {
            _SetNak(port, ptrUcbPacket);
     	    HandleUcbTx(port, ptrUcbPacket);
		}

		if (validFieldCount > 0) { /// apply any changes
			SetFieldData(); // xbowsp_fields.c
		}
	} else {
        _SetNak(port, ptrUcbPacket);
	    HandleUcbTx(port, ptrUcbPacket);
	}
}

/** ****************************************************************************
 * @name _UcbGetFields
 * @brief Handles UCB get fields command packet
 * Trace:
 * [SDD_UCB_GETFIELDS <-- SRC_UCB_GETFIELDS]
 * [SDD_UCB_GETFIELDS_ID <-- SRC_UCB_GETFIELDS]
 * [SDD_UCB_GETFIELDS_NAK1 <-- SRC_UCB_GETFIELDS]
 * [SDD_UCB_GETFIELDS_NAK2 <-- SRC_UCB_GETFIELDS]
 * @param [in] port - number request came in on, the reply will go out this port
 * @param [out] packetPtr - data part of packet
 * @retval N/A
 ******************************************************************************/
static void _UcbGetFields (uint16_t port,
                           UcbPacketStruct    *ptrUcbPacket)
{
	uint8_t  numFields = ptrUcbPacket->payload[0];
	uint8_t  fieldCount;
	uint8_t  validFieldCount = 0;
	uint16_t fieldId [UCB_MAX_PAYLOAD_LENGTH / 4];

	/// verify that the packet length matches packet specification
    if ((numFields > 0) &&
    	(ptrUcbPacket->payloadLength == (1 + numFields * 2))) {

        /// read all fields specified in get fields request
        for (fieldCount = 0; fieldCount < numFields; ++fieldCount) {
            /// read field ID from packet into usable array
            fieldId[validFieldCount] = (uint16_t)((ptrUcbPacket->payload[(fieldCount * 2) + 1] << 8) |
                                                   ptrUcbPacket->payload[(fieldCount * 2) + 2]);

            /// check get field address bounds
            if (((fieldId[validFieldCount] >= LOWER_CONFIG_ADDR_BOUND) &&
                 (fieldId[validFieldCount] <= UPPER_CONFIG_ADDR_BOUND)) ||
                 (fieldId[validFieldCount] == PRODUCT_CONFIGURATION_FIELD_ID)) {
                ++validFieldCount;
            }
        }

        if (validFieldCount > 0) {	/// all or some requested get field addresses valid?
            /// build and return valid get fields with data packet
            ptrUcbPacket->payloadLength = (uint8_t)(1 + validFieldCount * 4);

            /// number of fields being returned
            ptrUcbPacket->payload[0] = validFieldCount;

            /// retrieve all fields specified in get fields request
            for (fieldCount = 0; fieldCount < validFieldCount; ++fieldCount) {
                /** product configuration field is out of normal
                    configuration address range, needs to be fetched from
                    calibration structure */
                if (fieldId[fieldCount] == PRODUCT_CONFIGURATION_FIELD_ID) {
                    uint16_t cfg = GetProductConfiguration();

                    ptrUcbPacket->payload[(fieldCount * 4) + 1] = (uint8_t)((fieldId[fieldCount] >> 8) & 0xff);
                    ptrUcbPacket->payload[(fieldCount * 4) + 2] = (uint8_t)( fieldId[fieldCount]       & 0xff);
                    ptrUcbPacket->payload[(fieldCount * 4) + 3] = (uint8_t)((cfg >> 8) & 0xff);
                    ptrUcbPacket->payload[(fieldCount * 4) + 4] = (uint8_t)( cfg       & 0xff);
                }
                else {	/// normal field, exists in configuration structure
                    uint16_t param = configGetParam(fieldId[fieldCount]);

                    ptrUcbPacket->payload[(fieldCount * 4) + 1] = (uint8_t)((fieldId[fieldCount] >> 8) & 0xff);
                    ptrUcbPacket->payload[(fieldCount * 4) + 2] = (uint8_t)( fieldId[fieldCount]       & 0xff);
                    ptrUcbPacket->payload[(fieldCount * 4) + 3] = (uint8_t)((param >> 8) & 0xff);
                    ptrUcbPacket->payload[(fieldCount * 4) + 4] = (uint8_t)( param  & 0xff);
                }
            }
            HandleUcbTx(port, ptrUcbPacket);
        }

        /// any invalid get fields addresses?
        if (validFieldCount < numFields) {
            _SetNak(port, ptrUcbPacket);
            HandleUcbTx(port, ptrUcbPacket);
        }
    } else {
        _SetNak(port, ptrUcbPacket);
        HandleUcbTx(port, ptrUcbPacket);
    }
}

/** ****************************************************************************
 * @name _UcbReadFields
 * @brief Handles UCB read fields command
 * Trace:
 * [SDD_UCB_READFIELDS <-- SRC_UCB_READFIELDS]
 * [SDD_UCB_READFIELDS_ID <-- SRC_UCB_READFIELDS]
 * [SDD_UCB_READFIELDS_NAK1 <-- SRC_UCB_READFIELDS]
 * [SDD_UCB_READFIELDS_NAK2 <-- SRC_UCB_READFIELDS]
 * @param [in] port - number request came in on, the reply will go out this port
 * @param [out] packetPtr - data part of packet
 * @retval N/A
 ******************************************************************************/
void _UcbReadFields (uint16_t port,
                            UcbPacketStruct    *ptrUcbPacket)
{
	uint8_t  numFields = ptrUcbPacket->payload[0];
	uint8_t  fieldCount;
	uint8_t  validFieldCount = 0;
	uint16_t fieldId [UCB_MAX_PAYLOAD_LENGTH / 4];
	uint16_t fieldData;

//    SetMaxDelay_Watchdog(); // Set the watchdog delay to its maximum value

    /// verify that the packet length matches packet specification
    if ((numFields > 0) &&
    (ptrUcbPacket->payloadLength == (1 + numFields * 2))) {
        /// read all fields specified in get fields request
        for (fieldCount = 0; fieldCount < numFields; ++fieldCount) {
            /// read field ID from packet into usable array
            fieldId[validFieldCount] = (uint16_t)((ptrUcbPacket->payload[(fieldCount * 2) + 1] << 8) |
                                                   ptrUcbPacket->payload[(fieldCount * 2) + 2]);

            /// check read field address bounds
            if (((fieldId[validFieldCount] >= LOWER_CONFIG_ADDR_BOUND) &&
                 (fieldId[validFieldCount] <= UPPER_CONFIG_ADDR_BOUND)) ||
                 (fieldId[validFieldCount] == PRODUCT_CONFIGURATION_FIELD_ID)) {
                ++validFieldCount;
            }
        }

        if (validFieldCount > 0) { /// all or some requested addresses valid?
            /// build and return valid get fields with data packet
            ptrUcbPacket->payloadLength = (uint8_t)(1 + validFieldCount * 4);

            ptrUcbPacket->payload[0] = validFieldCount; ///< # being returned

            /// retrieve all fields specified in get fields request
            for (fieldCount = 0; fieldCount < validFieldCount; ++fieldCount) {
                /** product configuration field is out of normal configuration
                    address range, needs to be fetched from calibration
                    structure */
                if (fieldId[fieldCount] == PRODUCT_CONFIGURATION_FIELD_ID) {
                    ptrUcbPacket->payload[(fieldCount * 4) + 1] = (uint8_t)((fieldId[fieldCount] >> 8) & 0xff);
                    ptrUcbPacket->payload[(fieldCount * 4) + 2] = (uint8_t)( fieldId[fieldCount]       & 0xff);

                    EEPROM_ReadProdConfig( &fieldData);
                    ptrUcbPacket->payload[(fieldCount * 4) + 3] = (uint8_t)((fieldData >> 8) & 0xff);
                    ptrUcbPacket->payload[(fieldCount * 4) + 4] = (uint8_t)( fieldData       & 0xff);
                } else {	/// normal field, exists in configuration structure
                    ptrUcbPacket->payload[(fieldCount * 4) + 1] = (uint8_t)((fieldId[fieldCount] >> 8) & 0xff);
                    ptrUcbPacket->payload[(fieldCount * 4) + 2] = (uint8_t)( fieldId[fieldCount]       & 0xff);
                    /// read field from EEPROM
                    EEPROM_ReadByte(fieldId[fieldCount], sizeof(fieldData), &fieldData);
                    ptrUcbPacket->payload[(fieldCount * 4) + 3] = (uint8_t)((fieldData >> 8) & 0xff);
                    ptrUcbPacket->payload[(fieldCount * 4) + 4] = (uint8_t)( fieldData       & 0xff);
                }
            }
            HandleUcbTx(port, ptrUcbPacket);
        }

        /// invalid get fields addresses?
        if (validFieldCount < numFields) {
            _SetNak(port, ptrUcbPacket);
            HandleUcbTx(port, ptrUcbPacket);
        }
    } else {
        _SetNak(port, ptrUcbPacket);
        HandleUcbTx(port, ptrUcbPacket);
    }
//    RestoreDelay_Watchdog();
}

/** ****************************************************************************
 * @name _UcbWriteFields
 * @briefHandle UCB write fields command packet
 * Trace:
 * [SDD_UCB_WRITEFIELDS <-- SRC_UCB_WRITEFIELDS]
 * [SDD_UCB_WRITEFIELDS_ID <-- SRC_UCB_WRITEFIELDS]
 * [SDD_UCB_WRITEFIELDS_NAK1 <-- SRC_UCB_WRITEFIELDS]
 * [SDD_UCB_WRITEFIELDS_NAK2 <-- SRC_UCB_WRITEFIELDS]
 * [SDD_UCB_WRITEFIELDS_DATA <-- SRC_UCB_WRITEFIELDS]
 * [SDD_UCB_WRITEFIELDS_PAIR_VALID <-- SRC_UCB_WRITEFIELDS]
 *
 * @param [in] port - number request came in on, the reply will go out this port
 * @param [out] packetPtr - data part of packet
 * @retval N/A
 ******************************************************************************/
static void _UcbWriteFields (uint16_t port,
                             UcbPacketStruct    *ptrUcbPacket)
{
    uint8_t  numFields = ptrUcbPacket->payload[0];
    uint8_t  fieldCount;
    uint8_t  validFieldCount;
    /** some fields need to be set together, so collect all field ID's and data
       in one set of arrays */
    uint16_t fieldId   [UCB_MAX_PAYLOAD_LENGTH / 4];
    /// array sizes are based on maximum number of fields to change
    uint16_t fieldData [UCB_MAX_PAYLOAD_LENGTH / 4];

//    SetMaxDelay_Watchdog(); // Set the watchdog delay to its maximum value

    /// verify that the packet length matches packet specification
    if( ( numFields > 0 ) &&
        ( ptrUcbPacket->payloadLength == (1 + numFields * 4) ) )
    {
        /// loop through all fields and data specified in set fields request
        for (fieldCount = 0; fieldCount < numFields; ++fieldCount) {
            /// read field ID and field data from packet into usable arrays
            fieldId[fieldCount]   = (uint16_t)((ptrUcbPacket->payload[(fieldCount * 4) + 1] << 8) |
                                                ptrUcbPacket->payload[(fieldCount * 4) + 2]);
            fieldData[fieldCount] = (uint16_t)((ptrUcbPacket->payload[(fieldCount * 4) + 3] << 8) |
                                                ptrUcbPacket->payload[(fieldCount * 4) + 4]);
        }

        /// check if data to set is valid xbowsp_fields.c
        validFieldCount = CheckEepromFieldData(numFields,
                                               fieldId,
                                               fieldData,
                                               fieldId);
// there is no check for corect number of changed fields only that something has changed
        if (validFieldCount > 0) { ///< all or some requested field changes valid?
            /// apply any changes
            if (WriteFieldData() == TRUE) { // xbowsp_fields.c
                /// build and send positive acknowledgement packet
                ptrUcbPacket->payloadLength = (uint8_t)(1 + (validFieldCount * 2));

                /// number of valid fields
                ptrUcbPacket->payload[0] = validFieldCount;

                /// place valid field ID's in payload
                for (fieldCount = 0; fieldCount < validFieldCount; ++fieldCount) {
                    ptrUcbPacket->payload[(fieldCount * 2) + 1] = (uint8_t)((fieldId[fieldCount] >> 8) & 0xff);
                    ptrUcbPacket->payload[(fieldCount * 2) + 2] = (uint8_t)( fieldId[fieldCount]       & 0xff);
                }
                HandleUcbTx(port, ptrUcbPacket);
            } else {
                _SetNak(port, ptrUcbPacket);
                HandleUcbTx(port, ptrUcbPacket);
            }
        }

        /// any invalid requested field changes?
        if (validFieldCount < numFields) {
            _SetNak(port, ptrUcbPacket);
            HandleUcbTx(port, ptrUcbPacket);
        }
    } else {
        _SetNak(port, ptrUcbPacket);
        HandleUcbTx(port, ptrUcbPacket);
    }
//    RestoreDelay_Watchdog(); // Restore the watchdog delay to its original value
}


/** ****************************************************************************
 * @name _UcbUnlockEeprom
 * @brief unlock the EEPROM if the CRC of the unit serial number and payload is 0
 * Trace:
 *	[SDD_UCB_UNLOCK_EEPROM <-- SRC_UCB_UNLOCK_EEPROM]
 *
 * @param [in] port -  number request came in on, the reply will go out this port
 * @param [out] packetPtr - data part of packet
 * @retval N/A
 ******************************************************************************/
void _UcbUnlockEeprom (uint16_t port,
                              UcbPacketStruct    *ptrUcbPacket)
{
    BOOL res = EEPROM_UnlockCalSectors(ptrUcbPacket->payload);

 	if (res == TRUE) 
    { ///< correct unlock code?
        gBitStatus.hwStatus.bit.unlockedEEPROM = TRUE;
	    ptrUcbPacket->payloadLength = 0;
    }else {
        _SetNak(port, ptrUcbPacket);
	}
	HandleUcbTx(port, ptrUcbPacket);
} /* end HandleUcbUnlockEeprom() */
    
/** ****************************************************************************
 * @name _UcbLockEeprom
 * @brief lock the EEPROM
 * Trace:
 *	[SDD_UCB_LOCK_EEPROM <-- SRC_UCB_LOCK_EEPROM]
 *
 * @param [in] port -  number request came in on, the reply will go out this port
 * @param [out] packetPtr - data part of packet
 * @retval N/A
 ******************************************************************************/
void _UcbLockEeprom (uint16_t port,
                              UcbPacketStruct    *ptrUcbPacket)
{
    BOOL res = EEPROM_LockCalSectors();

 	if (res == TRUE) 
    { ///< correct unlock code?
        gBitStatus.hwStatus.bit.unlockedEEPROM = FALSE;
	    ptrUcbPacket->payloadLength = 0;
    }else {
        _SetNak(port, ptrUcbPacket);
    }
	HandleUcbTx(port, ptrUcbPacket);
} /* end HandleUcbUnlockEeprom() */


/** ****************************************************************************
 * @name _UcbReadEeprom
 * @brief Read 16 bit cells from EEPROM, passed in starting address and number
 * of cells in the packet payload
 *
 * Trace:
 *	[SDD_UCB_READ_EEPROM <-- SRC_UCB_READ_EEPROM]
 *   [SDD_UCB_READ_EEPROM_ERROR <-- SRC_UCB_READ_EEPROM]
 *
 * @param [in] port -  number request came in on, the reply will go out this port
 * @param [out] packetPtr - data part of packet
 * @retval N/A
 ******************************************************************************/
void _UcbReadEeprom (uint16_t port, UcbPacketStruct    *ptrUcbPacket)
{
    uint16_t startAddress;
    uint8_t  wordsToRead;
    uint8_t  bytesToRead;

//    SetMaxDelay_Watchdog(); ///< Set the watchdog delay to its maximum value

    startAddress = (uint16_t)((ptrUcbPacket->payload[0] << 8) | ptrUcbPacket->payload[1]);
    wordsToRead  = ptrUcbPacket->payload[2];
    bytesToRead  = (uint8_t)(wordsToRead * 2);

    /// verify that the packet length matches packet specification
    if (ptrUcbPacket->payloadLength == 3) {
        ptrUcbPacket->payloadLength = (uint8_t)(ptrUcbPacket->payloadLength + bytesToRead);
        EEPROM_ReadByte(startAddress, bytesToRead, &(ptrUcbPacket->payload[3]));
	} else {
        _SetNak(port, ptrUcbPacket);
    }
	HandleUcbTx(port, ptrUcbPacket);
//    RestoreDelay_Watchdog(); /// Restore the watchdog delay to its original value
}

/** ****************************************************************************
 * @name _UcbReadEeprom
 * @brief Read 16 bit cells from EEPROM, passed in starting address and number
 * of cells in the packet payload
 *
 * Trace:
 *	[SDD_UCB_READ_EEPROM <-- SRC_UCB_READ_EEPROM]
 *   [SDD_UCB_READ_EEPROM_ERROR <-- SRC_UCB_READ_EEPROM]
 *
 * @param [in] port -  number request came in on, the reply will go out this port
 * @param [out] packetPtr - data part of packet
 * @retval N/A
 ******************************************************************************/
static void _UcbReadCal (uint16_t port, UcbPacketStruct    *ptrUcbPacket)
{
    uint16_t startAddress;
    uint8_t  wordsToRead;
    uint8_t  bytesToRead;

//    SetMaxDelay_Watchdog(); ///< Set the watchdog delay to its maximum value

    startAddress  = (uint16_t)((ptrUcbPacket->payload[0] << 8) | ptrUcbPacket->payload[1]);
    startAddress *= SIZEOF_WORD;
    wordsToRead  = ptrUcbPacket->payload[2];
    bytesToRead  = (uint8_t)(wordsToRead * SIZEOF_WORD);

    /// verify that the packet length matches packet specification
    if (ptrUcbPacket->payloadLength == 3) {
        ptrUcbPacket->payloadLength = (uint8_t)(ptrUcbPacket->payloadLength + bytesToRead);
        EEPROM_ReadFromCalPartition(startAddress, bytesToRead, &(ptrUcbPacket->payload[3]));
	} else {
        _SetNak(port, ptrUcbPacket);
    }
	HandleUcbTx(port, ptrUcbPacket);
//    RestoreDelay_Watchdog(); /// Restore the watchdog delay to its original value
}


/** ****************************************************************************
 * @name _UcbWriteEeprom
 * @brief Write data as 16 bit cells into an unlocked EEPROM.
 * Trace:
 *	[SDD_UCB_WRITE_EEPROM <-- SRC_UCB_WRITE_EEPROM]
 *	[SDD_UCB_WRITE_EEPROM_ERROR <-- SRC_UCB_WRITE_EEPROM]
 *
 * @param [in] port -  number request came in on, the reply will go out this port
 * @param [out] packetPtr - data part of packet
 * @retval N/A
 ******************************************************************************/
void _UcbWriteEeprom (uint16_t port,
                             UcbPacketStruct    *ptrUcbPacket)
{
    uint16_t startAddress;
    uint8_t  wordsToWrite;
    uint16_t bytesToWrite;

    startAddress = (uint16_t)((ptrUcbPacket->payload[0] << 8) |
                               ptrUcbPacket->payload[1]);
    wordsToWrite = ptrUcbPacket->payload[2];
    bytesToWrite = (uint16_t)wordsToWrite * 2;

//    SetMaxDelay_Watchdog();

    /// verify that the packet length matches packet specification
    if ((ptrUcbPacket->payloadLength == (bytesToWrite + 3)) &&
        (gBitStatus.hwStatus.bit.unlockedEEPROM == TRUE) ) {
        /// flag current CRC as invalid
        gBitStatus.swDataBIT.bit.calibrationCRCError = TRUE;

        /// 0 means no errors
        if (EEPROM_WriteWords(startAddress,
                             wordsToWrite,
                             &(ptrUcbPacket->payload[3])) == 0) {
            ptrUcbPacket->payloadLength = 3;
        } else {
            _SetNak(port, ptrUcbPacket);
        }
    } else {
        _SetNak(port, ptrUcbPacket);
    }

    HandleUcbTx(port, ptrUcbPacket);
//    RestoreDelay_Watchdog();
}

/** ****************************************************************************
 * @name _UcbWriteCal
 * @brief Write data as 16 bit cells into an unlocked EEPROM.
 *
 * @param [in] port -  number request came in on, the reply will go out this port
 * @param [out] packetPtr - data part of packet
 * @retval N/A
 ******************************************************************************/
void _UcbWriteCal (uint16_t port,
                          UcbPacketStruct    *ptrUcbPacket)
{
    uint16_t startAddress;
    uint8_t  wordsToWrite;
    uint16_t bytesToWrite;

    startAddress = (uint16_t)((ptrUcbPacket->payload[0] << 8) | ptrUcbPacket->payload[1]);
    startAddress *= SIZEOF_WORD;
    wordsToWrite  = ptrUcbPacket->payload[2];
    bytesToWrite  = (uint16_t)wordsToWrite * SIZEOF_WORD;

//    SetMaxDelay_Watchdog();

    /// verify that the packet length matches packet specification
    if (ptrUcbPacket->payloadLength == (bytesToWrite + 3)) {
        /// 0 means no errors
        if (EEPROM_WriteToCalPartition(startAddress, bytesToWrite,
                             &(ptrUcbPacket->payload[3])) != 0) {
            ptrUcbPacket->payloadLength = 3;
        } else {
            _SetNak(port, ptrUcbPacket);
        }
    } else {
        _SetNak(port, ptrUcbPacket);
    }

    HandleUcbTx(port, ptrUcbPacket);
//    RestoreDelay_Watchdog();
}

/** ****************************************************************************
 * @name _UcbWriteApp
 * @brief Write data as 16 bit cells into an unlocked EEPROM.
 *
 * @param [in] port -  number request came in on, the reply will go out this port
 * @param [out] packetPtr - data part of packet
 * @retval N/A
 ******************************************************************************/
#ifdef BOOT_MODE
static void _UcbWriteApp (uint16_t port, UcbPacketStruct    *ptrUcbPacket)
{
    uint32_t startAddress;
    uint16_t bytesToWrite;

    startAddress = (uint32_t)((ptrUcbPacket->payload[0] << 24) |
                               ptrUcbPacket->payload[1] << 16  |
                               ptrUcbPacket->payload[2] << 8   |
                               ptrUcbPacket->payload[3]);

    bytesToWrite  = ptrUcbPacket->payload[4];

    /// verify that the packet length matches packet specification
    if (ptrUcbPacket->payloadLength == (bytesToWrite + 5)) {
        /// 0 means no errors
        if (EEPROM_WriteToAppPartition(startAddress, bytesToWrite,
                             &(ptrUcbPacket->payload[5])) != 0) {
            ptrUcbPacket->payloadLength = 5;
        } else {
            _SetNak(port, ptrUcbPacket);
        }
    } else {
        _SetNak(port, ptrUcbPacket);
    }

    HandleUcbTx(port, ptrUcbPacket);
//    RestoreDelay_Watchdog();
}
#endif

/** ****************************************************************************
 * @name _UcbJump2BOOT
 * @brief
 * Trace:
 *	[SDD_UCB_UNLOCK_EEPROM <-- SRC_UCB_UNLOCK_EEPROM]
 *
 * @param [in] port -  number request came in on, the reply will go out this port
 * @param [out] packetPtr - data part of packet
 * @retval N/A
 ******************************************************************************/
static void _UcbJump2BOOT (uint16_t port, UcbPacketStruct    *ptrUcbPacket)
{

#ifndef IMU383
    _SetNak(port, ptrUcbPacket);
	HandleUcbTx(port, ptrUcbPacket);
#else
	HandleUcbTx(port, ptrUcbPacket);
    DelayMs(10);
    HW_EnforceBootMode();
    HW_SystemReset();
#endif

} 

/** ****************************************************************************
 * @name _UcbJump2APP
 * @brief
 * Trace:
 *	[SDD_UCB_UNLOCK_EEPROM <-- SRC_UCB_UNLOCK_EEPROM]
 *
 * @param [in] port -  number request came in on, the reply will go out this port
 * @param [out] packetPtr - data part of packet
 * @retval N/A
 ******************************************************************************/
static void _UcbJump2APP (uint16_t port, UcbPacketStruct    *ptrUcbPacket)
{

    BOOL sigValid = FALSE;
#ifndef BOOT_MODE
	    HandleUcbTx(port, ptrUcbPacket);
        // nothing to do - already there
        return;
#endif
    sigValid = EEPROM_ApplyAppSignature(TRUE);
	if(!sigValid){
        _SetNak(port, ptrUcbPacket);
    }
    HandleUcbTx(port, ptrUcbPacket);
    DelayMs(10);
    if(sigValid){
        HW_EnforceAppMode();
        HW_SystemReset();
    }
} 



/** ****************************************************************************
 * @name _UcbSoftwareReset
 * @brief Force a watchdog reset from the above function.
 *
 * Trace: [SDD_UCB_SW_RESET <-- SRC_UCB_SW_RESET]
 * softwareReset
 * @param [in] port -  number request came in on, the reply will go out this port
 * @param [out] packetPtr - data part of packet
 * @retval N/A
 ******************************************************************************/
static void _UcbSoftwareReset (uint16_t port,
                                UcbPacketStruct    *ptrUcbPacket)
{
    /// return software reset acknowledgement
	HandleUcbTx(port, ptrUcbPacket);

    DelayMs(10);
   
    HW_SystemReset();
}



/** ****************************************************************************
 * @name _SetNak
 * @brief set up UCB error NAK packet. Return NAK with requested packet type in
 *        data field. HandleUcbTx() needs to be called.
 * Trace:
 * @param [in] port - port type UCB or CRM
 * @param [out] packetPtr - filled in packet from the mapped physical port
 * @retval N/A
 ******************************************************************************/
static void _SetNak (uint16_t port, UcbPacketStruct    *ptrUcbPacket)
{

	/// return NAK, requested packet type placed in data field by external port
	ptrUcbPacket->packetType 	= UCB_NAK;
    ptrUcbPacket->code_MSB      = 0x15;
    ptrUcbPacket->code_LSB      = 0x15;
	ptrUcbPacket->payloadLength = UCB_PACKET_TYPE_LENGTH;
}

/** ****************************************************************************
 * @name _UcbError
 * @brief UCB error packet
 * Trace: [SDD_UCB_UNKNOWN_02 <-- SRC_UCB_UNKNOWN]
 *        [SDD_UCB_TIMEOUT_02 <-- SRC_UCB_TIMEOUT_REPLY]
 *        [SDD_UCB_CRC_FAIL_02 <-- SRC_UCB_CRCFAIL_REPLY]
 * @param [in] port - port type UCB or CRM
 * @param [out] packetPtr - filled in packet from the mapped physical port
 * @retval N/A
 ******************************************************************************/
static void _UcbError (uint16_t port,
                       UcbPacketStruct    *ptrUcbPacket)
{
	/// return NAK, requested packet type placed in data field by external port
	ptrUcbPacket->packetType 	= UCB_NAK;
	ptrUcbPacket->payloadLength = UCB_PACKET_TYPE_LENGTH;
    HandleUcbTx(port, ptrUcbPacket);
}


/** ****************************************************************************
 * @name HandleUcbPacket - API
 * @brief general handler
 * Trace: [SDD_HANDLE_PKT <-- SRC_HANDLE_PACKET]
 * @param [in] port - port type UCB or CRM
 * @param [out] packetPtr - filled in packet from the mapped physical port
 * @retval N/A
 ******************************************************************************/
void HandleUcbPacket (uint16_t port, UcbPacketStruct *ptrUcbPacket)
{
		switch (ptrUcbPacket->packetType) {
            case UCB_PING:
                _UcbPing(port, ptrUcbPacket); break;
            case UCB_ECHO:
                _UcbEcho(port, ptrUcbPacket); break;
            case UCB_GET_PACKET:
                _UcbGetPacket(port, ptrUcbPacket); break;
            case UCB_GET_FIELDS:
                _UcbGetFields(port, ptrUcbPacket); break;
            case UCB_J2BOOT:
            case UCB_J2IAP:
                _UcbJump2BOOT (port, ptrUcbPacket); break;
            case UCB_J2APP:
                _UcbJump2APP (port, ptrUcbPacket); break;
            case UCB_READ_CAL:
                _UcbReadCal(port, ptrUcbPacket); break;
            case UCB_SOFTWARE_RESET:
                _UcbSoftwareReset(port, ptrUcbPacket); break;
#ifdef BOOT_MODE
            case UCB_WRITE_APP:
                _UcbWriteApp(port, ptrUcbPacket); break;
#else
            case UCB_SET_FIELDS:
                _UcbSetFields(port, ptrUcbPacket); break;
            case UCB_READ_FIELDS:
                _UcbReadFields(port, ptrUcbPacket); break;
            case UCB_WRITE_FIELDS:
                _UcbWriteFields(port, ptrUcbPacket); break;
            case UCB_UNLOCK_EEPROM:
                _UcbUnlockEeprom(port, ptrUcbPacket); break;
            case UCB_LOCK_EEPROM:
                _UcbLockEeprom(port, ptrUcbPacket); break;
            case UCB_READ_EEPROM:
                _UcbReadEeprom(port, ptrUcbPacket); break;
            case UCB_WRITE_EEPROM:
                _UcbWriteEeprom(port, ptrUcbPacket); break;
            case UCB_WRITE_CAL:
                _UcbWriteCal(port, ptrUcbPacket); break;
            case UCB_USER_IN:
                {
                int result = HandleUserInputPacket(ptrUcbPacket);
                if(result != USER_PACKET_OK){
  		            _SetNak(port,ptrUcbPacket);
                }
                }
                HandleUcbTx(port, ptrUcbPacket);
                break;
#endif // BOOT_MODE
//			case UCB_READ_APP:break;
            default:
                _UcbError(port, ptrUcbPacket); break;
                break; /// default handler - unknown send NAK
		}
}
/* end HandleUcbPacket() */


/** ****************************************************************************
 * @name _WriteMagAlignParamsToMemory
 * @brief writes the magnetic alignment parameters to the EEPROM
 * Trace: [SDD_HANDLE_PKT <-- SRC_HANDLE_PACKET]
 * @retval N/A
 ******************************************************************************/
void WriteMagAlignParamsToMemory( uint16_t port,
                                  UcbPacketStruct    *ptrUcbPacket )
{
    // Array sizes are based on maximum number of fields to change
    uint16_t fieldId   [UCB_MAX_PAYLOAD_LENGTH / 4];
    fieldId[0] = 0x0009;   // X-Axis Hard-Iron
    fieldId[1] = 0x000A;   // Y-Axis Hard-Iron
    fieldId[2] = 0x000B;   // Soft-Iron Scale-Ratio
    fieldId[3] = 0x000E;   // Soft-Iron Angle

    uint16_t fieldData [UCB_MAX_PAYLOAD_LENGTH / 4];
    fieldData[0] = gConfiguration.hardIronBias[0];      // [G]
    fieldData[1] = gConfiguration.hardIronBias[1];      // [G]
    fieldData[2] = gConfiguration.softIronScaleRatio;   // [N/A]
    fieldData[3] = gConfiguration.softIronAngle;        // [deg]

    uint8_t numFields = 0x4;
    ptrUcbPacket->payloadLength = 1 + 4*numFields;
    ptrUcbPacket->payload[0]    = numFields;

    // first two are field ID, second two are data
    for( uint8_t fieldNum = 0; fieldNum < numFields; fieldNum++ ) {
        ptrUcbPacket->payload[(fieldNum*numFields)+1] = fieldId[fieldNum] >> 8;
        ptrUcbPacket->payload[(fieldNum*numFields)+2] = fieldId[fieldNum] & 0x00FF;
        ptrUcbPacket->payload[(fieldNum*numFields)+3] = fieldData[fieldNum] >> 8;
        ptrUcbPacket->payload[(fieldNum*numFields)+4] = fieldData[fieldNum] & 0x00FF;
    }

    _UcbWriteFields( port, ptrUcbPacket );
}

