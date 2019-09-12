/** ***************************************************************************
 * @file board.c 
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
#include "platformAPI.h"
#include "eepromAPI.h"

static BOOL testMode   = FALSE;
static BOOL testStatus = TRUE;

static void board_usart1_Init()
{
  
  GPIO_InitTypeDef  GPIO_InitStruct;
  
  USART1_TX_GPIO_CLK_ENABLE();
  USART1_RX_GPIO_CLK_ENABLE();


  //UART TX pin configuration
  GPIO_InitStruct.Pin       = USART1_TX_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = USART1_TX_AF;

  HAL_GPIO_Init(USART1_TX_GPIO_PORT, &GPIO_InitStruct);

  // UART RX pin configuration
  GPIO_InitStruct.Pin       = USART1_RX_PIN;
  GPIO_InitStruct.Alternate = USART1_RX_AF;

  HAL_GPIO_Init(USART1_RX_GPIO_PORT, &GPIO_InitStruct);
}

static void board_usart1_DeInit()
{
  HAL_GPIO_DeInit(USART1_TX_GPIO_PORT, USART1_TX_PIN);
  HAL_GPIO_DeInit(USART1_RX_GPIO_PORT, USART1_RX_PIN);

}

static void board_usart3_Init()
{
  
  GPIO_InitTypeDef  GPIO_InitStruct;
  
  USART3_TX_GPIO_CLK_ENABLE();
  USART3_RX_GPIO_CLK_ENABLE();


  //UART TX pin configuration
  GPIO_InitStruct.Pin       = USART3_TX_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = USART3_TX_AF;

  HAL_GPIO_Init(USART3_TX_GPIO_PORT, &GPIO_InitStruct);

  // UART RX pin configuration
  GPIO_InitStruct.Pin       = USART3_RX_PIN;
  GPIO_InitStruct.Alternate = USART3_RX_AF;

  HAL_GPIO_Init(USART3_RX_GPIO_PORT, &GPIO_InitStruct);
}

static void board_usart3_DeInit()
{
  HAL_GPIO_DeInit(USART3_TX_GPIO_PORT, USART3_TX_PIN);
  HAL_GPIO_DeInit(USART3_RX_GPIO_PORT, USART3_RX_PIN);

}

void board_usart_Init(int channel)
{
    if(channel == 0){
        board_usart1_Init();
    }
    if(channel == 1){
        board_usart3_Init();
    }

}

void board_usart_DeInit(int channel)
{
    if(channel == 0){
        board_usart1_DeInit();
    }
    if(channel == 1){
        board_usart3_DeInit();
    }

}

void board_SpiBitBang_Init()
{
    GPIO_InitTypeDef  GPIO_InitStruct;
    
    SPI2_PORT_CLK_ENABLE();
    
    GPIO_InitStruct.Pin   = SPI_MISO_PINS;
    GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(SPI_MISO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin   = (SPI_MOSI_PIN | SPI_SCK_PIN);
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(SPI_MISO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin   = SPI_NSS_PINS;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(SPI_NSS_PORT, &GPIO_InitStruct);
}


void board_user_spi_Init(int channel)
{
    GPIO_InitTypeDef  GPIO_InitStruct;
  
    SPI2_PORT_CLK_ENABLE();

    GPIO_InitStruct.Pin       = SPI2_SCK_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = SPI2_SCK_AF;

    HAL_GPIO_Init(SPI2_PORT, &GPIO_InitStruct);

    // SPI MISO pin configuration
    GPIO_InitStruct.Pin       = SPI2_MISO_PIN;
    GPIO_InitStruct.Alternate = SPI2_MISO_AF;
    HAL_GPIO_Init(SPI2_PORT, &GPIO_InitStruct);

    // SPI MOSI pin configuration
    GPIO_InitStruct.Pin       = SPI2_MOSI_PIN;
    GPIO_InitStruct.Alternate = SPI2_MOSI_AF;
    HAL_GPIO_Init(SPI2_PORT, &GPIO_InitStruct);

  // Enable GPIOB clock
  __HAL_RCC_GPIOB_CLK_ENABLE();

  // Configure NSS pin for interrupt
  GPIO_InitStruct.Pin   = GPIO_PIN_12;
  GPIO_InitStruct.Pull  = GPIO_PULLUP;
  GPIO_InitStruct.Mode  = GPIO_MODE_INPUT | GPIO_MODE_IT_RISING; 
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // Configure the DRDY pin
  GPIO_InitStruct.Pin   = DATA_READY_PIN;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


  NVIC_SetPriority((IRQn_Type)(EXTI15_10_IRQn), 0x03);
//  HAL_NVIC_EnableIRQ((IRQn_Type)(EXTI15_10_IRQn));

}


void board_configure_sync_pin()
{
  GPIO_InitTypeDef  GPIO_InitStruct;
  // Configure the 1PPS pin
  // Route to Timer 15
  
  PPS_CLK_ENABLE();

  GPIO_InitStruct.Pin       = PPS_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    
  HAL_GPIO_Init(PPS_PORT, &GPIO_InitStruct);

}


void HW_IO3_On()
{
  HAL_GPIO_WritePin(IO3_PORT, IO3_PIN, GPIO_PIN_SET); 
}

void HW_IO3_Off()
{
  HAL_GPIO_WritePin(IO3_PORT, IO3_PIN, GPIO_PIN_RESET); 
}

void HW_IO2_On()
{
  HAL_GPIO_WritePin(IO2_PORT, IO2_PIN, GPIO_PIN_SET); 
}

void HW_IO2_Off()
{
  HAL_GPIO_WritePin(IO2_PORT, IO2_PIN, GPIO_PIN_RESET); 
}


void HW_IO3_Toggle()
{
  HAL_GPIO_TogglePin(IO3_PORT, IO3_PIN);
}

void HW_IO2_Toggle()
{
  HAL_GPIO_TogglePin(IO2_PORT, IO2_PIN);
}


void HW_DRDY_Off()
{
  HAL_GPIO_WritePin(GPIOB, DATA_READY_PIN, GPIO_PIN_SET); 
}


void HW_DRDY_On()
{
  HAL_GPIO_WritePin(GPIOB, DATA_READY_PIN, GPIO_PIN_RESET); 
}


uint8_t HW_ReadConfig()
{
  uint8_t hw = HAL_GPIO_ReadPort(GPIOA, HWCONFIG_PINS);
  return hw >> 1;
}

/**
* @brief  System Clock Configuration
*         The system Clock is configured as follows :
*            System Clock source            = PLL (MSI)
*            SYSCLK(Hz)                     = 80000000
*            HCLK(Hz)                       = 80000000
*            AHB Prescaler                  = 1
*            APB1 Prescaler                 = 1
*            APB2 Prescaler                 = 1
*            MSI Frequency(Hz)              = 4000000
*            PLL_M                          = 1
*            PLL_N                          = 40
*            PLL_R                          = 2
*            PLL_P                          = 7
*            PLL_Q                          = 4
*            Flash Latency(WS)              = 4
* @param  None
* @retval None
*/
void SystemClock_Config(void)
{
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    

    // HSE is enabled after System reset, activate PLL with HSE as source
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM       = 5;
    RCC_OscInitStruct.PLL.PLLN       = 32;
    RCC_OscInitStruct.PLL.PLLR       = 2;
    RCC_OscInitStruct.PLL.PLLP       = 7;
    RCC_OscInitStruct.PLL.PLLQ       = 4;


    if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        while(1);
    }
    
    // Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
    // clocks dividers
    RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;  
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  
    if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
    {
        while(1);
    }
}


void IO3_Init()
{
    GPIO_InitTypeDef  GPIO_InitStruct;
    
    IO3_CLK_ENABLE();
    
    GPIO_InitStruct.Pin   = IO3_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    
    HAL_GPIO_Init(IO3_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(IO3_PORT, IO3_PIN, GPIO_PIN_RESET); 
}

void IO2_Init()
{
    GPIO_InitTypeDef  GPIO_InitStruct;
    
    IO2_CLK_ENABLE();
    
    GPIO_InitStruct.Pin   = IO2_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    
    HAL_GPIO_Init(IO2_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(IO2_PORT, IO2_PIN, GPIO_PIN_RESET); 
}

void HWConfig_Init()
{
    GPIO_InitTypeDef  GPIO_InitStruct;
    
    HWCONFIG_CLK_ENABLE();
    
    GPIO_InitStruct.Pin   = HWCONFIG_PINS;
    GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    
    HAL_GPIO_Init(HWCONFIG_PORT, &GPIO_InitStruct);
}

// PPS  <>  DRDY
// NSS  <>  SCK
// MISO <>  MOSI

int board_DetermineInterfaceType()
{
	volatile int tmp = 0;
    GPIO_InitTypeDef  GPIO_InitStruct;
    int intf = 0;	// UART by default
	uint32_t modeA, pullA;
    uint32_t modeB, pullB;

    // Enable GPIO Clocks
    GPIOB_CLK_ENABLE();
    GPIOA_CLK_ENABLE();
    
    // Configure the DRDY as input
    GPIO_InitStruct.Pin   = DATA_READY_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    for(int i = 0; i < 10000; i++){tmp++;}
    
    tmp = HAL_GPIO_ReadPin(GPIOB, DATA_READY_PIN);

    if(tmp == 0){
        // enforced UART mode
    	fUART = TRUE;
		return 0;
    }

	modeA = HAL_GPIO_GetModeAll(GPIOA);
	modeB = HAL_GPIO_GetModeAll(GPIOB);
	pullA = HAL_GPIO_GetPuPdAll(GPIOA);
	pullB = HAL_GPIO_GetPuPdAll(GPIOB);

	HAL_GPIO_SetModeAll(GPIOA, 0x2B000000);	// set all inputs except jtag
    for(int i = 0; i < 1000; i++){tmp++;}	// give time to propagate

   // Configure the SPI interface pins as inputs with pullups
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Pin   = PPS_PIN;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    GPIO_InitStruct.Pin   = SPI2_NSS_PIN;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    GPIO_InitStruct.Pin   = SPI2_MOSI_PIN;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	// delay before checking
    for(int i = 0; i < 10000; i++){tmp++;}
	// check that all inputs are high - not driven by external signals low
    for(int i = 0; i < 1000; i++){
		tmp = 1;
        tmp &= HAL_GPIO_ReadPin(GPIOB, PPS_PIN);
        tmp &= HAL_GPIO_ReadPin(GPIOB, SPI2_NSS_PIN);
        tmp &= HAL_GPIO_ReadPin(GPIOB, SPI2_MOSI_PIN);
		if(tmp == 0){
			// some pins are driven low - assuming SPI interface connected
			HAL_GPIO_SetModeAll(GPIOB, modeB);	// restore pins
			HAL_GPIO_SetPuPdAll(GPIOB, pullB);
			HAL_GPIO_SetModeAll(GPIOA, modeA);	// restore pins
			HAL_GPIO_SetPuPdAll(GPIOA, pullA);
        	return 1;                           // SPI interface detected 
        }
    }
 
    HAL_GPIO_SetPuPdAll(GPIOA, 0xA6AAAAAA);	// set all inputs except jtag to pulldown mode
	// configure SPI pins in pulldown mode
    GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
    GPIO_InitStruct.Pin   = PPS_PIN;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    GPIO_InitStruct.Pin   = SPI2_NSS_PIN;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    GPIO_InitStruct.Pin   = SPI2_MOSI_PIN;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    GPIO_InitStruct.Pin   = DATA_READY_PIN;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	// delay before checking
    for(int i = 0; i < 10000; i++){tmp++;}
	// check that all inputs are low - not driven by external signals high

    for(int i = 0; i < 1000 && !intf; i++){
		tmp = 0;
        tmp += HAL_GPIO_ReadPin(GPIOB, PPS_PIN);
        tmp += HAL_GPIO_ReadPin(GPIOB, SPI2_NSS_PIN);
        tmp += HAL_GPIO_ReadPin(GPIOB, SPI2_MOSI_PIN);
		if(tmp != 0){
			// some pins are driven high - assuming SPI interface connected
			intf = 1;
			break;
        }
    }

	// restore pin configuration
	HAL_GPIO_SetModeAll(GPIOB, modeB);
	HAL_GPIO_SetPuPdAll(GPIOB, pullB);
	HAL_GPIO_SetModeAll(GPIOA, modeA);
	HAL_GPIO_SetPuPdAll(GPIOA, pullA);
	
	return intf;	 
} 

void HW_Init()
{
#ifdef USE_SPI
	volatile int rrr;
#endif
    HAL_Init();
    // Configure the system clock to 80 MHz
    SystemClock_Config();
    // Configure IO3
    IO3_Init();
    IO2_Init();
#ifdef USE_SPI
    rrr = board_DetermineInterfaceType();
	if(rrr != 0){
		fSPI = 1;
	}
#endif
  if(!fUART && !fSPI){
    testMode = TRUE; 
  }
}


void board_can_init()
{
  GPIO_InitTypeDef   GPIO_InitStruct;

  CAN_GPIO_CLK_ENABLE();
  GPIO_InitStruct.Pin       = CAN_TX_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Alternate = CAN_TX_AF;
  HAL_GPIO_Init(CAN_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin       = CAN_RX_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Alternate = CAN_RX_AF;

  HAL_GPIO_Init(CAN_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin       = CAN_CTL_TERM_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;

  HAL_GPIO_Init(CAN_CTL_PORT, &GPIO_InitStruct);
  HAL_GPIO_WritePin(CAN_CTL_PORT, CAN_CTL_TERM_PIN, GPIO_PIN_RESET); 

  GPIO_InitStruct.Pin       = CAN_CTL_AB_PIN;
  HAL_GPIO_Init(CAN_CTL_PORT, &GPIO_InitStruct);
  HAL_GPIO_WritePin(CAN_CTL_PORT, CAN_CTL_AB_PIN, GPIO_PIN_RESET); 

}

/** ****************************************************************************
 * @name SystemReset - API
 * @brief system reset
 * Trace: [SDD_HANDLE_PKT <-- SRC_HANDLE_PACKET]
 * @retval N/A
 ******************************************************************************/
void HW_SystemReset(void)
{
    *((uint32_t *)0xE000ED0C) = 0x05fa0004; 
}

 
static BOOL drdyStat  = 0;
static BOOL sckStat   = 1;
static BOOL mosiStat  = 0;
static BOOL testFirstTime = 1;

void HW_SetTestMode(BOOL fOn)
{
	testMode = fOn;
}

BOOL HW_IsTestMode()
{
	return testMode;
}


BOOL HW_IsTestOK()
{
	if(testMode){
		return testStatus;
	}
	return TRUE;
}

#ifndef BT383
// PPS  <>  DRDY
// NSS  <>  SCK
// MISO <>  MOSI
void board_PerformPinTest()
{
	if(!testMode || fSPI){
		return;
	}

	if(testFirstTime){
 		GPIO_InitTypeDef   GPIO_InitStruct;
        HAL_NVIC_DisableIRQ((IRQn_Type)(EXTI15_10_IRQn));
		testFirstTime = 0;
    	// Configure output pins
    	GPIO_InitStruct.Pin   = DATA_READY_PIN;
    	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    	GPIO_InitStruct.Pull  = GPIO_NOPULL;
    	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    	GPIO_InitStruct.Pin   = SPI2_SCK_PIN;
    	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    	GPIO_InitStruct.Pin   = SPI2_MOSI_PIN;
    	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    	// Configure input pins
    	GPIO_InitStruct.Pin   = PPS_PIN;
    	GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
    	GPIO_InitStruct.Pull  = GPIO_NOPULL;
    	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    	GPIO_InitStruct.Pin   = SPI2_NSS_PIN;
    	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    	GPIO_InitStruct.Pin   = SPI2_MISO_PIN;
    	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  	    HAL_GPIO_WritePin(GPIOB, DATA_READY_PIN, drdyStat); 
  	    HAL_GPIO_WritePin(GPIOB, SPI2_SCK_PIN, sckStat); 
 	    HAL_GPIO_WritePin(GPIOB, SPI2_MOSI_PIN, mosiStat);
		return; 
	}

	if(HAL_GPIO_ReadPin(GPIOB, PPS_PIN) != drdyStat){
		testStatus = FALSE;
	}

	if(HAL_GPIO_ReadPin(GPIOB, SPI2_NSS_PIN) != sckStat){
		testStatus = FALSE;
	}

	if(HAL_GPIO_ReadPin(GPIOB, SPI2_MISO_PIN) != mosiStat){
		testStatus = FALSE;
	}

	drdyStat ^= 1;
	sckStat  ^= 1;
	mosiStat ^= 1;

    HAL_GPIO_WritePin(GPIOB, DATA_READY_PIN, drdyStat); 
    HAL_GPIO_WritePin(GPIOB, SPI2_SCK_PIN, sckStat); 
    HAL_GPIO_WritePin(GPIOB, SPI2_MOSI_PIN, mosiStat);

    platformUpdateInterfaceTestStatus(testStatus);

} 
#else
// MOSI -> NSS
// DRDY -> PPS 
void board_PerformPinTest()
{
	if(!testMode || fSPI){
		return;
	}

	if(testFirstTime){
 		GPIO_InitTypeDef   GPIO_InitStruct;
        HAL_NVIC_DisableIRQ((IRQn_Type)(EXTI15_10_IRQn));
		testFirstTime = 0;
    	// Configure output pins
    	GPIO_InitStruct.Pin   = DATA_READY_PIN;
    	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    	GPIO_InitStruct.Pull  = GPIO_NOPULL;
    	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    	GPIO_InitStruct.Pin   = SPI2_MOSI_PIN;
    	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    	// Configure input pins
    	GPIO_InitStruct.Pin   = PPS_PIN;
    	GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
    	GPIO_InitStruct.Pull  = GPIO_NOPULL;
    	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    	GPIO_InitStruct.Pin   = SPI2_NSS_PIN;
    	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  	    HAL_GPIO_WritePin(GPIOB, DATA_READY_PIN, drdyStat); 
 	    HAL_GPIO_WritePin(GPIOB, SPI2_MOSI_PIN,  mosiStat);
		return; 
	}
	
	if(HAL_GPIO_ReadPin(GPIOB, PPS_PIN) != drdyStat){
		testStatus = FALSE;
	}
	
	if(HAL_GPIO_ReadPin(GPIOB, SPI2_NSS_PIN) != mosiStat){
		testStatus = FALSE;
	}

	drdyStat ^= 1;
	mosiStat ^= 1;

    HAL_GPIO_WritePin(GPIOB, DATA_READY_PIN, drdyStat); 
    HAL_GPIO_WritePin(GPIOB, SPI2_MOSI_PIN, mosiStat);

    platformUpdateInterfaceTestStatus(testStatus);
}

#endif


void HW_DisableAllInterrupts(void)
{
    for(int i=WWDG_IRQn; i <= CRS_IRQn; i++)
    {
        NVIC->ICER[i >> 0x05] =(unsigned int )0x01 << (i & (unsigned char)0x1F);
    }
}


