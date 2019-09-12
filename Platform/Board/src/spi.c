/** ***************************************************************************
 * @file spi.c 
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
#ifndef ALT_SPI_DRIVER

/* Includes ------------------------------------------------------------------*/
#include "spi.h"
#include "board.h"
#include "hwAPI.h"

typedef struct{
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
    uint8_t aTxBuffer[SPI_TXBUFFERSIZE];
    uint8_t aRxBuffer[SPI_RXBUFFERSIZE];
}spiCtrl_t;

spiCtrl_t  spiCtrl[2];

static SPI_HandleTypeDef   spi2Handle;
static DMA_HandleTypeDef   hdma2_tx;
static DMA_HandleTypeDef   hdma2_rx;

static void SPI_DMAReceiveCplt(DMA_HandleTypeDef *hdma);
static void SPI_DMATransmitCplt(DMA_HandleTypeDef *hdma);
static void SPI_DMAReceiveHalfCplt(DMA_HandleTypeDef *hdma);
static void SPI_DMARxError(DMA_HandleTypeDef *hdma);
static void SPI_DMATxError(DMA_HandleTypeDef *hdma);


#ifndef USE_SPI

__weak int  SPI_ProcessCommand(uint8_t cmd)
{

}

__weak void  SPI_ProcessData(uint8_t* in)
{

}

#endif

BOOL spi2_init()
{
  static BOOL firstTime = TRUE;
  
  if(firstTime){
    memset(&spi2Handle, 0, sizeof(SPI_HandleTypeDef));
  }

  SPI2_CLK_ENABLE();
  SPI2_DMA_CLK_ENABLE();

  /* Set the SPI parameters */
  spi2Handle.Instance               = SPI2;
  spi2Handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  spi2Handle.Init.Direction         = SPI_DIRECTION_2LINES;
  spi2Handle.Init.CLKPhase          = SPI_PHASE_2EDGE;    // risig edge for capturing data
  spi2Handle.Init.CLKPolarity       = SPI_POLARITY_HIGH;  // idle high
  spi2Handle.Init.DataSize          = SPI_DATASIZE_8BIT;
  spi2Handle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
  spi2Handle.Init.TIMode            = SPI_TIMODE_DISABLE;
  spi2Handle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
  spi2Handle.Init.CRCPolynomial     = 7;
  spi2Handle.Init.CRCLength         = SPI_CRC_LENGTH_8BIT;
  spi2Handle.Init.NSS               = SPI_NSS_SOFT;
  spi2Handle.Init.NSSPMode          = SPI_NSS_PULSE_DISABLE;
  spi2Handle.Init.Mode              = SPI_MODE_SLAVE;

  if(HAL_SPI_Init(&spi2Handle) != HAL_OK)
  {
    /* Initialization Error */
      while(1);
  }

    /*##-3- Configure the DMA ##################################################*/
    /* Configure the DMA handler for Transmission process */
    hdma2_tx.Instance                 = SPI2_TX_DMA_CHANNEL;
    hdma2_tx.Init.Request             = SPI2_TX_DMA_REQUEST;
    hdma2_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma2_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma2_tx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma2_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma2_tx.Init.Mode                = DMA_NORMAL;
//    hdma2_tx.Init.Mode                = DMA_CIRCULAR;
    hdma2_tx.Init.Priority            = DMA_PRIORITY_HIGH;

    HAL_DMA_Init(&hdma2_tx);

    // Associate the initialized DMA handle to the the SPI handle
    __HAL_LINKDMA(&spi2Handle, hdmatx, hdma2_tx);

    // Configure the DMA handler for Reception process
    hdma2_rx.Instance                 = SPI2_RX_DMA_CHANNEL;
    hdma2_rx.Init.Request             = SPI2_RX_DMA_REQUEST;
    hdma2_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma2_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma2_rx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma2_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma2_rx.Init.Mode                = DMA_NORMAL;
//    hdma2_rx.Init.Mode                = DMA_CIRCULAR;
    hdma2_rx.Init.Priority            = DMA_PRIORITY_HIGH;

    HAL_DMA_Init(&hdma2_rx);

    __HAL_LINKDMA(&spi2Handle, hdmarx, hdma2_rx);
    
    HAL_NVIC_SetPriority(SPI2_DMA_TX_IRQn, 1, 1);
    HAL_NVIC_EnableIRQ(SPI2_DMA_TX_IRQn);
    
    HAL_NVIC_SetPriority(SPI2_DMA_RX_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(SPI2_DMA_RX_IRQn);

    return TRUE;
}

BOOL spi2_deinit()
{

    HAL_NVIC_DisableIRQ(SPI2_DMA_TX_IRQn);
    HAL_NVIC_DisableIRQ(SPI2_DMA_RX_IRQn);

   	HAL_SPI_DeInit(&spi2Handle);

	HAL_DMA_DeInit(&hdma2_tx);
	HAL_DMA_DeInit(&hdma2_rx);

	__HAL_RCC_SPI2_FORCE_RESET();
	__HAL_RCC_DMA1_FORCE_RESET();

  	SPI2_CLK_DISABLE();
  	SPI2_DMA_CLK_DISABLE();

    return TRUE;
}



void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  
  while(1){};

}

void SPI2_DMA_RX_IRQHandler()
{
  HAL_DMA_IRQHandler(&hdma2_rx);
}

void SPI2_DMA_TX_IRQHandler()
{
  HAL_DMA_IRQHandler(&hdma2_tx);
}

//#define   SPI_BUF_LEN 32
uint8_t  _spiRxBuf[SPI_RXBUFFERSIZE];
uint8_t  _spiTxStartBuf[10] = {0,0,0,0,0,0,0,0};


int      _state = 0;
BOOL     _txStartComplete = FALSE; 


/** ****************************************************************************
 * @name: spi2_transfer() API
 * @brief Description:
 *		Used for bidirectional transfer of data on the customer user COM bus.
 *      Commands extracted from DMA functions to improve performance.  resets
 *      the DMA to restart for the next transfer
 *
 * @param [in] in  - pointer to rx buffer,  length long
 * @param [in] out - pointer to tx buffer, length long
 * @param [in] len - number of bytes to send
 * @retval: always 0
 ******************************************************************************/
static int spi2_transfer( volatile uint8_t *in, volatile uint8_t *out, uint32_t  inLen, uint32_t outLen)
{
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
  volatile uint16_t tmp;
#pragma GCC diagnostic warning "-Wunused-but-set-variable"

  static BOOL firstTime = TRUE;
  
  if ((in == NULL) || (out == NULL) || (inLen == 0U) || outLen == 0)
  {
    return HAL_ERROR;
  }

  // Disable SPI peripheral
  __HAL_SPI_DISABLE(&spi2Handle);
  // Disable Rx/Tx DMA Request
  CLEAR_BIT(spi2Handle.Instance->CR2, (SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN | SPI_CR2_LDMATX | SPI_CR2_LDMARX) );
  // Set RX Fifo threshold according the reception data length: 8bit
  SET_BIT(spi2Handle.Instance->CR2, SPI_RXFIFO_THRESHOLD);
  // Flush receive FIFO 
  for(int i = 0; i < 4; i++){
      tmp = spi2Handle.Instance->DR;
  }
  // read status
  tmp = spi2Handle.Instance->SR;
  if (firstTime)
  {
    firstTime = 0;
    // Set the SPI Rx DMA transfer callbacks
    spi2Handle.hdmarx->XferCpltCallback      = SPI_DMAReceiveCplt;
    spi2Handle.hdmarx->XferHalfCpltCallback  = SPI_DMAReceiveHalfCplt;
    spi2Handle.hdmarx->XferErrorCallback     = SPI_DMARxError;
    spi2Handle.hdmarx->XferAbortCallback     = NULL;
    // Set the SPI Tx DMA transfer callbacks
    spi2Handle.hdmatx->XferCpltCallback      = SPI_DMATransmitCplt;
    spi2Handle.hdmatx->XferErrorCallback     = SPI_DMATxError;
    spi2Handle.hdmatx->XferAbortCallback     = NULL;
  }


  spi2Handle.hdmarx->State = HAL_DMA_STATE_READY;
  spi2Handle.hdmatx->State = HAL_DMA_STATE_READY;

  // Configure the source, destination address and the data length
  // Enable DMA
  HAL_DMA_Start(spi2Handle.hdmarx, (uint32_t)&spi2Handle.Instance->DR, (uint32_t)in, inLen);
  HAL_DMA_Start(spi2Handle.hdmatx, (uint32_t)out, (uint32_t)&spi2Handle.Instance->DR, outLen);

  _txStartComplete = FALSE;
  // Enable the transfer complete and eror interrupt
  __HAL_DMA_DISABLE_IT(spi2Handle.hdmarx, (DMA_IT_HT | DMA_IT_TC | DMA_IT_TE));
  __HAL_DMA_ENABLE_IT(spi2Handle.hdmarx, (DMA_IT_TC | DMA_IT_TE | DMA_IT_HT));
  __HAL_DMA_DISABLE_IT(spi2Handle.hdmatx, (DMA_IT_TC | DMA_IT_TE));
  __HAL_DMA_ENABLE_IT(spi2Handle.hdmatx, (DMA_IT_TC | DMA_IT_TE));

  // Enable SPI peripheral
  __HAL_SPI_ENABLE(&spi2Handle);

  // Enable the SPI Error Interrupt Bit
  __HAL_SPI_ENABLE_IT(&spi2Handle, (SPI_IT_ERR));

  // Enable DMA Requests
 SET_BIT(spi2Handle.Instance->CR2, SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);
  
  // Enable interrupt from NSS pin 
 HAL_NVIC_EnableIRQ((IRQn_Type)(EXTI15_10_IRQn));

 return HAL_OK;
}



uint8_t testData[] = {
    0x12,
    0x34,
    0x56,
    0x78
};

static void SPI_DMAReceiveCplt(DMA_HandleTypeDef *hdma)
{
  SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;
  /* DMA Normal Mode */
  if ((hdma->Instance->CCR & DMA_CCR_CIRC) != DMA_CCR_CIRC)
  {
    // Disable Rx DMA Request
    __HAL_DMA_DISABLE_IT(spi2Handle.hdmarx, DMA_IT_HT | DMA_IT_TC | DMA_IT_TE);
    CLEAR_BIT(hspi->Instance->CR2, SPI_CR2_RXDMAEN);
	if(_state != 3){
    SPI_ProcessData(_spiRxBuf);
  }
  }
}

static void SPI_DMATransmitCplt(DMA_HandleTypeDef *hdma)
{
//  SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;
  /* DMA Normal Mode */
  if ((hdma->Instance->CCR & DMA_CCR_CIRC) != DMA_CCR_CIRC)
  {
   _txStartComplete = TRUE;
    // Disable Tx DMA Interrupt
    __HAL_DMA_DISABLE_IT(spi2Handle.hdmatx, DMA_IT_TC | DMA_IT_TE);
  }
}


static void SPI_DMAReceiveHalfCplt(DMA_HandleTypeDef *hdma)
{
  	if(_state == 3){
		// in boot mode
		return;
	}

  if ((hdma->Instance->CCR & DMA_CCR_CIRC) != DMA_CCR_CIRC)
  {
    if(_state == 1){
        // copy requested data to transmit buffer in case of read request
        // do nothing in case of write request
        SPI_ProcessCommand(_spiRxBuf[0]);
    }
    _state  = 2;
  }
}



static void SPI_DMARxError(DMA_HandleTypeDef *hdma)
{
  SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;
  // Disable DMA transfer on SPI side
  CLEAR_BIT(hspi->Instance->CR2, SPI_CR2_RXDMAEN);
  // call error handler here
  while(1){};
}

static void SPI_DMATxError(DMA_HandleTypeDef *hdma)
{
  SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;
  // Disable DMA transfer on SPI side
  CLEAR_BIT(hspi->Instance->CR2, SPI_CR2_TXDMAEN);
  // call error handler here
  while(1){};
}


// prepare for new SPI transaction
static void _spiRestart()
{
      _state  = 1;
      HAL_DMA_Abort(&hdma2_tx);
      // activate spi interface to receive 2 first bytes - command
      spi2_transfer(_spiRxBuf, _spiTxStartBuf, 2, 2);   
}

// prepare for new SPI transaction
static void _spiRestartBoot()
{
    // here we do not expetct any DMA interrupts -  
	_state  = 3;
    HAL_DMA_Abort(&hdma2_tx);
    HAL_DMA_Abort(&hdma2_rx);
    // activate spi interface to receive N bytes and transmit 4 bytes
    spi2_transfer(_spiRxBuf, _spiTxStartBuf, 512, 4);   
}


void SPI2_NSS_INTERRUPT_IRQHandler()
{
  /* EXTI line interrupt detected */
  if(__HAL_GPIO_EXTI_GET_IT(SPI2_NSS_PIN) != RESET){
     CLEAR_BIT(spi2Handle.Instance->CR2, SPI_CR2_RXDMAEN);
     __HAL_GPIO_EXTI_CLEAR_IT(SPI2_NSS_PIN);
      if(_state == 2){
         CLEAR_BIT(spi2Handle.Instance->CR2, SPI_CR2_TXDMAEN);
         _spiRestart();
      }
#ifdef BOOT_MODE
      	// in case of boot sequence
	  	if(_state == 3){
         	CLEAR_BIT(spi2Handle.Instance->CR2, SPI_CR2_TXDMAEN);
			uint32_t tmp = SPI_RXBUFFERSIZE - hdma2_rx.Instance->CNDTR;
    	 	SPI_ProcessBootData(_spiRxBuf, tmp, _spiTxStartBuf);
         	_spiRestartBoot();
	  	}		
#endif				
  }
}

// initialization of SPI interface - GPIO and peripheral
static BOOL _spiInit()
{
    board_user_spi_Init();
    return spi2_init();
}


// prepare for first SPI transaction
static void _spiBegin()
{
    _spiRestart();
}

//******************************************************************
//   SPI API functions
//******************************************************************

void SPI_ActivateInterface()
{
    _spiInit();
    _spiBegin();
}

void SPI_DeactivateInterface()
{
	spi2_deinit();
}

void SPI_ActivateInterfaceForBootLoading(uint16_t statusWord)
{
    
	_spiTxStartBuf[0] = (statusWord >> 8) & 0xff;
	_spiTxStartBuf[1] = statusWord & 0xff;
	_spiTxStartBuf[2] = (statusWord >> 8) & 0xff;
	_spiTxStartBuf[3] = statusWord & 0xff;
    _spiInit();
	_spiRestartBoot();
}


int SPI_PrepareForDataTransmit(uint8_t *out, int outLen)
{
  
  if (out == NULL || outLen == 0)
  {
    return HAL_ERROR;
  }

  // Configure the source, destination address and the data length
  // Enable DMA
  //CLEAR_BIT(spi2Handle.Instance->CR2, (SPI_CR2_TXDMAEN) );

  spi2Handle.hdmatx->State = HAL_DMA_STATE_READY;
  __HAL_UNLOCK(spi2Handle.hdmatx);

  HAL_DMA_Start(spi2Handle.hdmatx, (uint32_t)out, (uint32_t)&spi2Handle.Instance->DR, outLen);

  _txStartComplete = FALSE;

  // Enable DMA Requests
 //SET_BIT(spi2Handle.Instance->CR2, SPI_CR2_TXDMAEN);
  
  // Enable interrupt from NSS pin - Here?
// HAL_NVIC_EnableIRQ((IRQn_Type)(EXTI15_10_IRQn));

 return HAL_OK;
}

#endif
