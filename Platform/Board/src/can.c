/** ***************************************************************************
 * @file can.c 
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

#include "board.h"
#include "GlobalConstants.h"
#include "hwAPI.h"

void (*canTxCompleteCallback)(void) = NULL; 


CAN_HandleTypeDef     CanHandle;
CAN_TxHeaderTypeDef   TxHeader;
CAN_RxHeaderTypeDef   RxHeader;



/**
  * @brief  Configures the CAN.
  * @param  None
  * @retval None
  */
static void CAN_Config(int baudrate)
{

  // assuming CVAN clock is 40000000
  int prescaler = 8000000/baudrate;

  CAN_FilterTypeDef  sFilterConfig;

  /*##-1- Configure the CAN peripheral #######################################*/
  CanHandle.Instance                    = CAN1;
  CanHandle.Init.TimeTriggeredMode      = DISABLE;
  CanHandle.Init.AutoBusOff             = DISABLE;
  CanHandle.Init.AutoWakeUp             = DISABLE;
  CanHandle.Init.AutoRetransmission     = ENABLE;
  CanHandle.Init.ReceiveFifoLocked      = DISABLE;
  CanHandle.Init.TransmitFifoPriority   = DISABLE;
  CanHandle.Init.Mode                   = CAN_MODE_NORMAL;
  CanHandle.Init.SyncJumpWidth          = CAN_SJW_1TQ;
  CanHandle.Init.TimeSeg1               = CAN_BS1_4TQ;
  CanHandle.Init.TimeSeg2               = CAN_BS2_5TQ;
  CanHandle.Init.Prescaler              = prescaler;           // supposedly will come up at 250 KHz

  if (HAL_CAN_Init(&CanHandle) != HAL_OK)
  {
    while(1);
  }

  /*##-2- Configure the CAN Filter ###########################################*/
  sFilterConfig.FilterBank              = 0;
  sFilterConfig.FilterMode              = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale             = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh            = 0x0000;
  sFilterConfig.FilterIdLow             = 0x0000;
  sFilterConfig.FilterMaskIdHigh        = 0x0000;
  sFilterConfig.FilterMaskIdLow         = 0x0000;
  sFilterConfig.FilterFIFOAssignment    = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation        = ENABLE;
  sFilterConfig.SlaveStartFilterBank    = 14;

  if (HAL_CAN_ConfigFilter(&CanHandle, &sFilterConfig) != HAL_OK)
  {
    while(1);
  }

  /*##-3- Start the CAN peripheral ###########################################*/
  if (HAL_CAN_Start(&CanHandle) != HAL_OK)
  {
    while(1);
  }

  // Activate CAN RX notification
//  if (HAL_CAN_ActivateNotification(&CanHandle, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
//  {
//    while(1);
//  }

}


//HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *pHeader, uint8_t aData[], uint32_t *pTxMailbox);

BOOL CAN_Transmit(uint32_t id, uint8_t *data, int len) 
{
    HAL_StatusTypeDef status;
    uint32_t  mailboxNum;  
    
    // Configure Transmission process
    TxHeader.StdId                = id;
    TxHeader.ExtId                = 0;
    TxHeader.RTR                  = CAN_RTR_DATA;
    TxHeader.IDE                  = CAN_ID_STD;
    TxHeader.DLC                  = len;
    TxHeader.TransmitGlobalTime   = DISABLE;

    status = HAL_CAN_AddTxMessage(&CanHandle, &TxHeader, data, &mailboxNum);
    
    if(status != HAL_OK){
        return FALSE;
    }
    
    return TRUE;
}


/**
  * @brief CAN MSP Initialization
  *        This function configures the hardware resources used in this example:
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration
  *           - NVIC configuration for DMA interrupt request enable
  * @param hcan: CAN handle pointer
  * @retval None
  */
void CAN_Init(int baudrate)
{
    board_can_init();

    CAN1_CLK_ENABLE();
    
    CAN_Config(baudrate);
   
    HAL_NVIC_SetPriority(CAN1_TX_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
}

void HAL_CAN_TxComplete(CAN_HandleTypeDef *hcan)
{
    canTxCompleteCallback();
}


void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
    HAL_CAN_TxComplete(hcan);
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
{
    HAL_CAN_TxComplete(hcan);
}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
{
    HAL_CAN_TxComplete(hcan);
}


void CAN1_TX_IRQHandler(void)
{
    HAL_CAN_IRQHandler(&CanHandle);
}

  // configure CAN's hardware
void CAN_ConfigureCallback(void (*callback)(void))
{
   canTxCompleteCallback = callback;
}
