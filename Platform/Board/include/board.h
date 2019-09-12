/** ***************************************************************************
 * @file board.h 
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BOARD_H
#define __BOARD_H
#ifdef __cplusplus

 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_uart.h"
#include "stm32l431xx.h"
#include "hwAPI.h"

#define NUM_UARTS 2
#define NUM_SPI   1

//*****************************************************************************   
// Bitbang SPI definitions 
//*****************************************************************************   

#define IO3_PIN                            GPIO_PIN_11
#define IO3_PORT                           GPIOA
#define IO2_PIN                            GPIO_PIN_12
#define IO2_PORT                           GPIOA
#define IO3_CLK_ENABLE()                   __HAL_RCC_GPIOA_CLK_ENABLE()  
#define IO2_CLK_ENABLE()                   __HAL_RCC_GPIOA_CLK_ENABLE()  
#define GPIOA_CLK_ENABLE()                 __HAL_RCC_GPIOA_CLK_ENABLE()  
#define GPIOA_CLK_DISABLE()                __HAL_RCC_GPIOA_CLK_DISABLE()  
#define GPIOB_CLK_ENABLE()                 __HAL_RCC_GPIOB_CLK_ENABLE()  

#define SPI_MISO1_PIN                      GPIO_PIN_3
#define SPI_MISO2_PIN                      GPIO_PIN_4
#define SPI_MISO3_PIN                      GPIO_PIN_6
#define SPI_MISO_PINS                      (GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_6)
#define SPI_MISO_PORT                      GPIOA

#define SPI_MOSI_PIN                       GPIO_PIN_7
#define SPI_SCK_PIN                        GPIO_PIN_5
#define SPI_MOSI_PORT                      GPIOA
#define SPI_SCK_PORT                       GPIOA


#define SPI_NSS1_PIN                       GPIO_PIN_0
#define SPI_NSS2_PIN                       GPIO_PIN_1
#define SPI_NSS3_PIN                       GPIO_PIN_2
#define SPI_NSS_PINS                       GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2
#define SPI_NSS_PORT                       GPIOB

#define SPI_CLK_ENABLE()                  do{__HAL_RCC_GPIOA_CLK_ENABLE(); \
                                             __HAL_RCC_GPIOB_CLK_ENABLE();} while(0)  

#define HWCONFIG_PINS                       (GPIO_PIN_1 | GPIO_PIN_2)
#define HWCONFIG_PORT                       GPIOA
#define HWCONFIG_CLK_ENABLE()               __HAL_RCC_GPIOA_CLK_ENABLE()  

#define PPS_PIN                            GPIO_PIN_3
#define PPS_PORT                           GPIOB
#define PPS_CLK_ENABLE()                   __HAL_RCC_GPIOB_CLK_ENABLE()  

//******************************************************************************
// Definition for USART1 (DEBUG USART) resources
//******************************************************************************
#define USART1_CLK_ENABLE()              __HAL_RCC_USART1_CLK_ENABLE()
#define USART1_DMA2_CLK_ENABLE()         __HAL_RCC_DMA2_CLK_ENABLE()
#define USART1_RX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
#define USART1_TX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()

#define USART1_FORCE_RESET()             __HAL_RCC_USART1_FORCE_RESET()
#define USART1_RELEASE_RESET()           __HAL_RCC_USART1_RELEASE_RESET()

/* Definition for USART1 Pins */
#define USART1_TX_PIN                    GPIO_PIN_9
#define USART1_TX_GPIO_PORT              GPIOA
#define USART1_TX_AF                     GPIO_AF7_USART1
#define USART1_RX_PIN                    GPIO_PIN_10
#define USART1_RX_GPIO_PORT              GPIOA
#define USART1_RX_AF                     GPIO_AF7_USART1

/* Definition for USART1's DMA */
#define USART1_TX_DMA_CHANNEL             DMA2_Channel6     // DMA2
#define USART1_RX_DMA_CHANNEL             DMA2_Channel7     // DMA2

/* Definition for USART1's DMA Request */
#define USART1_TX_DMA_REQUEST             DMA_REQUEST_2     // DMA1
#define USART1_RX_DMA_REQUEST             DMA_REQUEST_2     // DMA1

/* Definition for USART1's NVIC */
#define USART1_DMA_TX_IRQn                DMA2_Channel6_IRQn
#define USART1_DMA_RX_IRQn                DMA2_Channel7_IRQn
#define USART1_DMA_TX_IRQHandler          DMA2_Channel6_IRQHandler
#define USART1_DMA_RX_IRQHandler          DMA2_Channel7_IRQHandler

//******************************************************************************
// Definition for USART3 (USER USART) resources
//******************************************************************************
#define USART3_CLK_ENABLE()              __HAL_RCC_USART3_CLK_ENABLE()
#define USART3_DMA1_CLK_ENABLE()         __HAL_RCC_DMA1_CLK_ENABLE()
#define USART3_RX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()
#define USART3_TX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()

#define USART3_FORCE_RESET()             __HAL_RCC_USART3_FORCE_RESET()
#define USART3_RELEASE_RESET()           __HAL_RCC_USART3_RELEASE_RESET()

/* Definition for USART1 Pins */
#define USART3_TX_PIN                    GPIO_PIN_10
#define USART3_TX_GPIO_PORT              GPIOB
#define USART3_TX_AF                     GPIO_AF7_USART3
#define USART3_RX_PIN                    GPIO_PIN_11
#define USART3_RX_GPIO_PORT              GPIOB
#define USART3_RX_AF                     GPIO_AF7_USART3

/* Definition for USART1's DMA */
#define USART3_TX_DMA_CHANNEL             DMA1_Channel2     // DMA1
#define USART3_RX_DMA_CHANNEL             DMA1_Channel3     // DMA1

/* Definition for USART1's DMA Request */
#define USART3_TX_DMA_REQUEST             DMA_REQUEST_2     // DMA1
#define USART3_RX_DMA_REQUEST             DMA_REQUEST_2     // DMA1

/* Definition for USART1's NVIC */
#define USART3_DMA_TX_IRQn                DMA1_Channel2_IRQn
#define USART3_DMA_RX_IRQn                DMA1_Channel3_IRQn
#define USART3_DMA_TX_IRQHandler          DMA1_Channel2_IRQHandler
#define USART3_DMA_RX_IRQHandler          DMA1_Channel3_IRQHandler

// Definition for TIM resources
// capture timer
#define TIM2_CLK_ENABLE()                 __HAL_RCC_TIM2_CLK_ENABLE()
#define TIM2_IRQn                         TIM2_IRQn
#define TIM2_IRQHandler                   TIM2_IRQHandler


#define TIM7_CLK_ENABLE()                 __HAL_RCC_TIM7_CLK_ENABLE()
#define TIM6_CLK_ENABLE()                 __HAL_RCC_TIM6_CLK_ENABLE()


// Definition for SPI2 resources
#define SPI2_PORT_CLK_ENABLE()            __HAL_RCC_GPIOB_CLK_ENABLE()
#define SPI2_PORT                         GPIOB
#define SPI2_NSS_PIN                      GPIO_PIN_12
#define SPI2_SCK_PIN                      GPIO_PIN_13
#define SPI2_MISO_PIN                     GPIO_PIN_14
#define SPI2_MOSI_PIN                     GPIO_PIN_15
#define DATA_READY_PIN                    GPIO_PIN_5    
#define DATA_READY_CLK_ENABLE()           __HAL_RCC_GPIOB_CLK_ENABLE()
#define SPI2_SCK_AF                       GPIO_AF5_SPI2
#define SPI2_MISO_AF                      GPIO_AF5_SPI2
#define SPI2_MOSI_AF                      GPIO_AF5_SPI2

#define SPI2_CLK_ENABLE()                __HAL_RCC_SPI2_CLK_ENABLE()
#define SPI2_CLK_DISABLE()               __HAL_RCC_SPI2_CLK_DISABLE()

#define SPI2_TX_DMA_CHANNEL               DMA1_Channel5           
#define SPI2_TX_DMA_REQUEST               DMA_REQUEST_1
#define SPI2_RX_DMA_CHANNEL               DMA1_Channel4
#define SPI2_RX_DMA_REQUEST               DMA_REQUEST_1

#define SPI2_DMA_TX_IRQn                  DMA1_Channel5_IRQn
#define SPI2_DMA_RX_IRQn                  DMA1_Channel4_IRQn
#define SPI2_DMA_TX_IRQHandler            DMA1_Channel5_IRQHandler
#define SPI2_DMA_RX_IRQHandler            DMA1_Channel4_IRQHandler


#define SPI2_NSS_IRQn                     EXTI15_10_IRQn
#define SPI2_NSS_INTERRUPT_IRQHandler     EXTI15_10_IRQHandler


#define SPI2_DMA_CLK_ENABLE()             __HAL_RCC_DMA1_CLK_ENABLE()
#define SPI2_DMA_CLK_DISABLE()            __HAL_RCC_DMA1_CLK_DISABLE()


// definitions for CAN port

#define     CAN_TX_PIN                   GPIO_PIN_12
#define     CAN_PORT                     GPIOA
#define     CAN_TX_AF                    GPIO_AF9_CAN1
#define     CAN_RX_PIN                   GPIO_PIN_11
#define     CAN_RX_AF                    GPIO_AF9_CAN1
#define     CAN_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOA_CLK_ENABLE()  
#define     CAN1_CLK_ENABLE()            __HAL_RCC_CAN1_CLK_ENABLE()  


#define     CAN_CTL_CLK_ENABLE()        __HAL_RCC_GPIOB_CLK_ENABLE()
#define     CAN_CTL_PORT               GPIOB
#define     CAN_CTL_TERM_PIN           GPIO_PIN_12
#define     CAN_CTL_AB_PIN             GPIO_PIN_3


void    board_usart_Init(int channel);
void    board_usart_DeInit(int channel);
void    board_SpiBitBang_Init();
void    board_configure_sync_pin();
void board_user_spi_Init();
void    board_can_init();
void    SystemClock_Config(void);
int     board_DetermineInterfaceType();
void    board_PerformPinTest();

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_H */
