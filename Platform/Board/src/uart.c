/** ***************************************************************************
 * @file uart.c 
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
#include "hwAPI.h"
#include "uart.h"
#include "osapi.h"

typedef struct{
    uint8_t aTxBuffer[TXBUFFERSIZE];
    uint8_t aRxBuffer[RXBUFFERSIZE];
    uint32_t bytesInRxBuf;
    uint32_t rxReadPos   ;
    uint32_t rxWritePos  ;
    uint32_t bytesInTxBuf;
    uint32_t txReadPos   ;
    uint32_t txWritePos  ;
    uint32_t txBytesInTransfer;
    int      txBusy     ;
    int      rxFromUart  ; 
    int      rxFromTo    ; 
    int      rxFromDma   ; 
    int      txErr       ;
    int      txFromWrite ;    
    int      txFromDma   ;
    int      txNum       ;
    IRQn_Type  type      ;
}uartCtrl_t;

uartCtrl_t  uartCtrl[2];


USART_TypeDef      *uartInstances[NUM_UARTS] = {USART1, USART3};
UART_HandleTypeDef  uartHandles[NUM_UARTS];


void uartInitCallback(UART_HandleTypeDef *huart)
{

}

void uartDeInitCallback(UART_HandleTypeDef *huart)
{

}



void usart1_DMA_Init(UART_HandleTypeDef *huart)
{
  static DMA_HandleTypeDef hdma_tx;
  static DMA_HandleTypeDef hdma_rx;


  /* Enable DMA clock */
  USART1_DMA2_CLK_ENABLE();
  
  // Configure the DMA handler for Transmission process
  hdma_tx.Instance                 = USART1_TX_DMA_CHANNEL;
  hdma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
  hdma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
  hdma_tx.Init.MemInc              = DMA_MINC_ENABLE;
  hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
  hdma_tx.Init.Mode                = DMA_NORMAL;
  hdma_tx.Init.Priority            = DMA_PRIORITY_LOW;
  hdma_tx.Init.Request             = USART1_TX_DMA_REQUEST;
  hdma_tx.XferCpltCallback         = UART_DMATransmitCplt;

  HAL_DMA_Init(&hdma_tx);

  /* Associate the initialized DMA handle to the UART handle */
  __HAL_LINKDMA(huart, hdmatx, hdma_tx);

  // Configure the DMA handler for reception process */
  hdma_rx.Instance                 = USART1_RX_DMA_CHANNEL;
  hdma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
  hdma_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
  hdma_rx.Init.MemInc              = DMA_MINC_ENABLE;
  hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
//  hdma_rx.Init.Mode                = DMA_NORMAL;
  hdma_rx.Init.Mode                = DMA_CIRCULAR;
  hdma_rx.Init.Priority            = DMA_PRIORITY_HIGH;
  hdma_rx.Init.Request             = USART1_RX_DMA_REQUEST;

  HAL_DMA_Init(&hdma_rx);

  // Associate the initialized DMA handle to the the UART handle
  __HAL_LINKDMA(huart, hdmarx, hdma_rx);
    
  // NVIC configuration for DMA TX complete interrupt
  HAL_NVIC_SetPriority(USART1_DMA_TX_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ(USART1_DMA_TX_IRQn);
    
  // NVIC configuration for DMA RX complete interrupt
  HAL_NVIC_SetPriority(USART1_DMA_RX_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_DMA_RX_IRQn);
  
  /* NVIC for USART, to catch the TX complete */
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
}

void usart3_DMA_Init(UART_HandleTypeDef *huart)
{
  static DMA_HandleTypeDef hdma_tx;
  static DMA_HandleTypeDef hdma_rx;


  /* Enable DMA clock */
  USART3_DMA1_CLK_ENABLE();
  
  // Configure the DMA handler for Transmission process
  hdma_tx.Instance                 = USART3_TX_DMA_CHANNEL;
  hdma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
  hdma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
  hdma_tx.Init.MemInc              = DMA_MINC_ENABLE;
  hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
  hdma_tx.Init.Mode                = DMA_NORMAL;
  hdma_tx.Init.Priority            = DMA_PRIORITY_LOW;
  hdma_tx.Init.Request             = USART3_TX_DMA_REQUEST;
  hdma_tx.XferCpltCallback         = UART_DMATransmitCplt;

  HAL_DMA_Init(&hdma_tx);

  /* Associate the initialized DMA handle to the UART handle */
  __HAL_LINKDMA(huart, hdmatx, hdma_tx);

  // Configure the DMA handler for reception process */
  hdma_rx.Instance                 = USART3_RX_DMA_CHANNEL;
  hdma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
  hdma_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
  hdma_rx.Init.MemInc              = DMA_MINC_ENABLE;
  hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
//  hdma_rx.Init.Mode                = DMA_NORMAL;
  hdma_rx.Init.Mode                = DMA_CIRCULAR;
  hdma_rx.Init.Priority            = DMA_PRIORITY_HIGH;
  hdma_rx.Init.Request             = USART3_RX_DMA_REQUEST;

  HAL_DMA_Init(&hdma_rx);

  // Associate the initialized DMA handle to the the UART handle
  __HAL_LINKDMA(huart, hdmarx, hdma_rx);
    
  // NVIC configuration for DMA TX complete interrupt
  HAL_NVIC_SetPriority(USART3_DMA_TX_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ(USART3_DMA_TX_IRQn);
    
  // NVIC configuration for DMA RX complete interrupt
  HAL_NVIC_SetPriority(USART3_DMA_RX_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART3_DMA_RX_IRQn);
  
  /* NVIC for USART, to catch the TX complete */
  HAL_NVIC_SetPriority(USART3_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
}




//*****************************************************************************
// UART - init functions
//*****************************************************************************

static BOOL usart1_Init(int channel, int baudrate)
{
    memset(&uartCtrl[channel], 0, sizeof(uartCtrl_t));
    uartCtrl[channel].type = USART1_DMA_TX_IRQn;
    // Enable USART clock
    USART1_CLK_ENABLE();
    UART_HandleTypeDef *uart = &uartHandles[channel];

    HAL_UART_RegisterCallback(uart, HAL_UART_MSPINIT_CB_ID, uartInitCallback);
    HAL_UART_RegisterCallback(uart, HAL_UART_MSPDEINIT_CB_ID, uartDeInitCallback);
    
    
    uart->Instance           = uartInstances[channel];
    uart->Init.BaudRate      = baudrate;
    uart->Init.WordLength    = UART_WORDLENGTH_8B;
    uart->Init.StopBits      = UART_STOPBITS_1;
    uart->Init.Parity        = UART_PARITY_NONE;
    uart->Init.HwFlowCtl     = UART_HWCONTROL_NONE;
    uart->Init.Mode          = UART_MODE_TX_RX;

    if(HAL_UART_DeInit(uart) != HAL_OK) {
        return FALSE;
    }  
    if(HAL_UART_Init(uart) != HAL_OK) {
        return FALSE;
    }

    usart1_DMA_Init(uart);
    
    // configure receive in DMA circular mode
    HAL_UART_Receive_DMA(uart, uartCtrl[channel].aRxBuffer, RXBUFFERSIZE);
    
    return TRUE;
}

static BOOL usart3_Init(int channel, int baudrate)
{
    memset(&uartCtrl[channel], 0, sizeof(uartCtrl_t));
    uartCtrl[channel].type = USART3_DMA_TX_IRQn;
   
    // Enable USART clock
    USART3_CLK_ENABLE();

    UART_HandleTypeDef *uart = &uartHandles[channel];
    uart->Instance           = uartInstances[channel];
    uart->Init.BaudRate      = baudrate;
    uart->Init.WordLength    = UART_WORDLENGTH_8B;
    uart->Init.StopBits      = UART_STOPBITS_1;
    uart->Init.Parity        = UART_PARITY_NONE;
    uart->Init.HwFlowCtl     = UART_HWCONTROL_NONE;
    uart->Init.Mode          = UART_MODE_TX_RX;

    if(HAL_UART_DeInit(uart) != HAL_OK) {
        return FALSE;
    }  
    if(HAL_UART_Init(uart) != HAL_OK) {
        return FALSE;
    }

    usart3_DMA_Init(uart);
    
    // configure receive in DMA circular mode
    HAL_UART_Receive_DMA(uart, uartCtrl[channel].aRxBuffer, RXBUFFERSIZE);
    
    return TRUE;
}



int     BytesTransmitted[512];
int     ByteCntIdx = 0;
uint8_t *_txBufPtr[512];

static int  _firstTime = 1;

BOOL UART_Init(int channel, int baudrate)
{
    
    if(_firstTime){
        memset(BytesTransmitted, 0, sizeof(BytesTransmitted));
        ByteCntIdx = 0;
        _firstTime = 0;
    }
 
    board_usart_Init(channel);
    if(channel == 0){
        return usart1_Init(0, baudrate);
    }

    if(channel == 1){
        return usart3_Init(1, baudrate);
    }
    
    return FALSE;
    
}


//*****************************************************************************
// UART - read functions
//*****************************************************************************

int UART_Read(int channel, uint8_t *buffer, uint32_t size)
{
    int bytesReady, bytesToRead, res = 0;
    uartCtrl_t *ctrl = &uartCtrl[channel];
    
    if((bytesReady = ctrl->bytesInRxBuf) == 0){
        return 0;
    }
    
    while(size && bytesReady){
        bytesToRead = RXBUFFERSIZE - uartCtrl[channel].rxReadPos;
        if(bytesToRead > bytesReady){
            bytesToRead = bytesReady;
        }
        if(bytesToRead > size){
            bytesToRead = size;
        }
        memcpy(buffer, (uint8_t*)&ctrl->aRxBuffer[ctrl->rxReadPos], bytesToRead);
        buffer       += bytesToRead;
        ctrl->rxReadPos    += bytesToRead;
        ctrl->rxReadPos    %= RXBUFFERSIZE;
        size         -= bytesToRead;
        res          += bytesToRead;
        bytesReady   -= bytesToRead;
        ctrl->rxFromUart   += bytesToRead;
    }
    ENTER_CRITICAL();
    ctrl->bytesInRxBuf -= res; 
    EXIT_CRITICAL();
    return res;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
    uartCtrl_t *ctrl;
    
    if(UartHandle == &uartHandles[0]){
        ctrl = &uartCtrl[0];
    }else if(UartHandle == &uartHandles[1]){
        ctrl = &uartCtrl[1];
    }else{
        return;
    }
    ctrl->rxFromDma    += (RXBUFFERSIZE - ctrl->rxWritePos);
    ctrl->bytesInRxBuf += (RXBUFFERSIZE - ctrl->rxWritePos);
    if(ctrl->bytesInRxBuf > RXBUFFERSIZE){
        ctrl->rxReadPos   += (ctrl->bytesInRxBuf - RXBUFFERSIZE);
        ctrl->rxReadPos   %= RXBUFFERSIZE;
        ctrl->bytesInRxBuf = RXBUFFERSIZE;
    }
    ctrl->rxWritePos    = 0;
}

void HAL_UART_RxTimeoutCallback(UART_HandleTypeDef *UartHandle)
{
    uartCtrl_t *ctrl;
    uint32_t tmp = RXBUFFERSIZE - UartHandle->hdmarx->Instance->CNDTR;

    if(UartHandle == &uartHandles[0]){
        ctrl = &uartCtrl[0];
    }else if(UartHandle == &uartHandles[1]){
        ctrl = &uartCtrl[1];
    }else{
        return;
    }

    ctrl->rxFromTo      += (tmp - ctrl->rxWritePos);
    ctrl->bytesInRxBuf  += (tmp - ctrl->rxWritePos);
    if(ctrl->bytesInRxBuf > RXBUFFERSIZE){
        ctrl->rxReadPos   += (ctrl->bytesInRxBuf - RXBUFFERSIZE);
        ctrl->rxReadPos   %= RXBUFFERSIZE;
        ctrl->bytesInRxBuf = RXBUFFERSIZE;
    }
    ctrl->rxWritePos     = tmp;
}

//*****************************************************************************
// UART - write functions
//*****************************************************************************
static int _attemptedWrites = 0;
static int _attemptedSize   = 0;
static int _actualSize      = 0;
//static int _bytesDropped    = 0;
int UART_Write(int channel, uint8_t *buffer, uint32_t size)
{
    uartCtrl_t *ctrl         = &uartCtrl[channel];
    UART_HandleTypeDef *uart = &uartHandles[channel];   
    int bytesToWrite  = TXBUFFERSIZE - ctrl->bytesInTxBuf;
    _attemptedSize   += size;
    _attemptedWrites++;

    if(bytesToWrite > size){
        bytesToWrite = size;
    }

    _actualSize += bytesToWrite;

    for(int i = 0; i < bytesToWrite; i++){
        ctrl->aTxBuffer[ctrl->txWritePos++] = *buffer++;
        if(ctrl->txWritePos == TXBUFFERSIZE){
            ctrl->txWritePos = 0;
        }
    }
    
    ENTER_CRITICAL();
    ctrl->bytesInTxBuf += bytesToWrite; 
    ctrl->txFromWrite  += bytesToWrite;
    if(!ctrl->txBusy){
        // trigger TX complete DMA interrupt
        ctrl->txBusy = 1;
        uart->hdmatx->ForcedISR = 1;
        HAL_NVIC_SetPendingIRQ(ctrl->type);
    }
    EXIT_CRITICAL();
    
    return bytesToWrite;
}



/**
* @brief  Tx Transfer completed callback
* @param  UartHandle: UART handle. 
* @note   This example shows a simple way to report end of DMA Tx transfer, and 
*         you can add your own implementation. 
* @retval None
*/
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart)
{
    uint32_t maxBytesToTransfer;
    uartCtrl_t *ctrl;
    
    if(uart == &uartHandles[0]){
        ctrl = &uartCtrl[0];
    }else if(uart == &uartHandles[1]){
        ctrl = &uartCtrl[1];
    }else{
        return;
    }

    uint8_t *txPtr = &ctrl->aTxBuffer[ctrl->txReadPos];
    
    if(ctrl->bytesInTxBuf){
        ctrl->txBusy = 1;
        maxBytesToTransfer = TXBUFFERSIZE - ctrl->txReadPos;
        if(ctrl->bytesInTxBuf <= maxBytesToTransfer){
            maxBytesToTransfer = ctrl->bytesInTxBuf;
            ctrl->txReadPos += maxBytesToTransfer;
            ctrl->txReadPos %= TXBUFFERSIZE;
        }else {
            ctrl->txReadPos = 0;
        }
        ctrl->bytesInTxBuf -= maxBytesToTransfer;
        uart->gState = HAL_UART_STATE_READY;
        uart->hdmatx->ForcedISR = 0;
        BytesTransmitted[ByteCntIdx] = maxBytesToTransfer;
        _txBufPtr[ByteCntIdx]        = txPtr;
        ByteCntIdx = (ByteCntIdx +1) & 0x1FF;
        HAL_UART_Transmit_DMA(uart, txPtr, maxBytesToTransfer);
        ctrl->txFromDma += maxBytesToTransfer;
    }else {
        ctrl->txBusy = 0;
    }
}


/**
* @brief  UART error callbacks
* @param  UartHandle: UART handle
* @note   This example shows a simple way to report transfer error, and you can
*         add your own implementation.
* @retval None
*/
void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{

}

void USART1_IRQHandler(void)
{
  HAL_UART_IRQHandler(&uartHandles[0]);
}

void USART3_IRQHandler(void)
{
  HAL_UART_IRQHandler(&uartHandles[1]);
}

void USART1_DMA_RX_IRQHandler(void)
{
  HAL_DMA_IRQHandler(uartHandles[0].hdmarx);
}

void USART1_DMA_TX_IRQHandler(void)
{
  HAL_DMA_IRQHandler(uartHandles[0].hdmatx);
}

void USART3_DMA_RX_IRQHandler(void)
{
  HAL_DMA_IRQHandler(uartHandles[1].hdmarx);
}

void USART3_DMA_TX_IRQHandler(void)
{
  HAL_DMA_IRQHandler(uartHandles[1].hdmatx);
}


