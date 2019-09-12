/** ***************************************************************************
 * @file timer.c 
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


/* Includes ------------------------------------------------------------------*/
#include "timer.h"
#include "GlobalConstants.h"
#include "board.h"
#include "osapi.h"
#include "hwAPI.h"


TIM_HandleTypeDef      TimHandle;
TIM_HandleTypeDef      Tim2Handle;
TIM_HandleTypeDef      TimdHandle;
int NewTick = 0;
int imuCounter = 0;
BOOL extSync         = FALSE;
BOOL dacqScheduled   = FALSE; 
uint64_t systemTmrHi = 0LL;
uint32_t prevTstamp  = 0;
uint64_t solutionTstamp = 0;
    
void DelayMs(int ms)
{
    HAL_Delay(ms); 
}

int TIMER_IsDacqOverrun()
{
  return NewTick;
}

uint64_t platformGetCurrTimeStamp()
{
    uint32_t tmp = TIM2->CNT;
	if(tmp < prevTstamp){
		systemTmrHi += 0x0000000100000000LL;
	}
	
	prevTstamp = tmp;

    return systemTmrHi | tmp;
}

uint64_t platformGetSolutionTstamp()
{
    return solutionTstamp;
}


int TIMER_WaitForNewDacqTick()
{
     imuCounter += 5;            // miliseconds considering 200 Hz tick
     board_PerformPinTest();
     while(NewTick == 0){};
     NewTick = 0;
     solutionTstamp = platformGetCurrTimeStamp(); // Maitenance of system us timer
     return 0;
}


int InitDacqTimer(int freq)
{
    int res;

    TIM7_CLK_ENABLE();

    TimHandle.Instance               = TIM7;
    TimHandle.Init.Period            = (uint32_t)(SystemCoreClock / (freq*8));
    TimHandle.Init.Prescaler         = 7;
    TimHandle.Init.ClockDivision     = 0;
    TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
    TimHandle.Init.RepetitionCounter = 0;
    
    HAL_NVIC_SetPriority(TIM7_IRQn, 3, 0);
    
    res = HAL_TIM_Base_Init(&TimHandle);
    return res;
}

int StartDacqTimer(int freq)
{
    int res;
    res = InitDacqTimer(freq);
    
    if(res != HAL_OK){
        return res;
    }
    
    res = HAL_TIM_Base_Start_IT(&TimHandle);
  
    if(res != HAL_OK){
        return res;
    }

    HAL_NVIC_EnableIRQ(TIM7_IRQn);

    return res;
}

int drdyTimerInit(int delay)
{
    int res;

    TIM6_CLK_ENABLE();

    TimdHandle.Instance               = TIM6;
    TimdHandle.Init.Period            = (uint32_t)(SystemCoreClock/delay);
    TimdHandle.Init.Prescaler         = 0;
    TimdHandle.Init.ClockDivision     = 0;
    TimdHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
    TimdHandle.Init.RepetitionCounter = 0;
    
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 5, 0);

    res = HAL_TIM_OnePulse_Init(&TimdHandle, TIM_OPMODE_SINGLE);

    return res;
}


int drdyTimerStart()
{
    int res;
    
    res = HAL_TIM_Base_Start_IT(&TimdHandle);

    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
  
    return res;
}


void TIM7_IRQHandler(void)
{
  OSEnterISR();
  HAL_TIM_IRQHandler(&TimHandle);
  OSExitISR();
}

uint64_t activeSyncEdgeTstamp = 0LL;
uint64_t activeDrdyEdgeTstamp = 0LL;
uint64_t sensorSamplingTstamp = 0LL;

int TIMER_GetSensToSyncDelay()
{
    return activeSyncEdgeTstamp - sensorSamplingTstamp;    //validate that not negative? put in buffer ???
}


int TIMER_GetSyncToDrdyDelay()
{
    return  platformGetCurrTimeStamp() - activeSyncEdgeTstamp;              // validate that not negative?
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim == &TimHandle && !extSync){
        sensorSamplingTstamp = platformGetCurrTimeStamp();
        NewTick = 1;
        dacqScheduled = TRUE;
    }
    if(htim == &TimdHandle){
        activeDrdyEdgeTstamp = platformGetCurrTimeStamp();
    }
}

void TIM6_DAC_IRQHandler(void)
{
  OSEnterISR();
  HAL_TIM_IRQHandler(&TimdHandle);
  OSExitISR();
}


int  oneKHzUpperLimit;
int  oneKHzLowerLimit;

void StartReferenceTimer(int precisionUs)
{

  int timerFreq   = SystemCoreClock;
  int usPrescaler = timerFreq/1000000;

  TIM2_CLK_ENABLE();
  
  Tim2Handle.Instance               = TIM2;
  Tim2Handle.Init.Period            = 0xFFFFFFFF;
  Tim2Handle.Init.Prescaler         = usPrescaler*precisionUs - 1;
  Tim2Handle.Init.ClockDivision     = 0;
  Tim2Handle.Init.CounterMode       = TIM_COUNTERMODE_UP;
  Tim2Handle.Init.RepetitionCounter = 0;

  if(HAL_TIM_IC_Init(&Tim2Handle) != HAL_OK)
  {
    /* Initialization Error - stay here so far */
    while(1);
  }

  /*##-3- Start the Input Capture in interrupt mode ##########################*/
  if(HAL_TIM_Base_Start(&Tim2Handle) != HAL_OK)
  {
    /* Initialization Error - stay here so far */
    while(1);
  }

}

uint32_t GetSensorSamplingTstamp()
{
    return sensorSamplingTstamp;
}


void ConfigureTimerForSyncCapture()
{
  TIM_IC_InitTypeDef   sICConfig;

  oneKHzUpperLimit =  (int)((double)SystemCoreClock*1.001/1000 + 0.5);   // +0.1% vs system clock
  oneKHzLowerLimit =  (int)((double)SystemCoreClock*0.999/1000 + 0.5);   // -0.1% vs system clock

  TIM2_CLK_ENABLE();
  
  Tim2Handle.Instance = TIM2;
  Tim2Handle.Init.Period            = 0xFFFFFFFF;
  Tim2Handle.Init.Prescaler         = 0;
  Tim2Handle.Init.ClockDivision     = 0;
  Tim2Handle.Init.CounterMode       = TIM_COUNTERMODE_UP;
  Tim2Handle.Init.RepetitionCounter = 0;

  if(HAL_TIM_IC_Init(&Tim2Handle) != HAL_OK)
  {
    /* Initialization Error - stay here so far */
    while(1);
  }

  /*##-2- Configure the Input Capture channel ################################*/ 
  /* Configure the Input Capture of channel 2 */
  sICConfig.ICPolarity  = TIM_ICPOLARITY_RISING;
  sICConfig.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sICConfig.ICPrescaler = TIM_ICPSC_DIV1;
  sICConfig.ICFilter    = 0;   
  if(HAL_TIM_IC_ConfigChannel(&Tim2Handle, &sICConfig, TIM_CHANNEL_2) != HAL_OK)
  {
    /* Initialization Error - stay here so far */
    while(1);
  }
  
  /*##-3- Start the Input Capture in interrupt mode ##########################*/
  if(HAL_TIM_IC_Start_IT(&Tim2Handle, TIM_CHANNEL_2) != HAL_OK)
  {
    /* Initialization Error - stay here so far */
    while(1);
  }

  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 1);
  /* Enable the TIM2 global Interrupt */
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
}


uint32_t  platformGetIMUCounter()
{
    return imuCounter;
}


/**
  * @brief  This function handles TIM interrupt request.
  * @param  None
  * @retval None
  */
void TIM2_IRQHandler(void)
{
  OSEnterISR();
  HAL_TIM_IRQHandler(&Tim2Handle);
  OSExitISR();
}


extern int getPacketRateDivider();

int capLog[32];
int logPtr = 0;
uint32_t ppsTstamp = 0; 

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    static int count = 0, refCounter = 5; // divide from 1000Hz to 200Hz
    static int restarted = 0;
    static uint32_t prevCap = 0;
    uint32_t curCap;
    static volatile int diff;

    if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2 ){
        curCap  = TIM2->CCR2;
        if(!extSync){
            diff    = curCap - prevCap;
            capLog[logPtr++] = diff;
            logPtr  &= 0x1f; 
            prevCap = curCap;
            if(diff < oneKHzUpperLimit && diff > oneKHzLowerLimit){
			// 10 consequtive matches
                count++;
                if(count >= 10 && dacqScheduled){
                    //switch right after last dacq interval was cheduled 
                    extSync = TRUE;
                    count       = 0;
                }else {
                    dacqScheduled = FALSE;
                }
            }else{
              count = 0;
            }
        }else{
           //sync achieved - start to sync system from external signal
           ++count;
           if(count >= refCounter){
               count       = 0;
                restarted  = 1;
                NewTick = 1;
                sensorSamplingTstamp = platformGetCurrTimeStamp();
                activeSyncEdgeTstamp = platformGetCurrTimeStamp();
           }
           if(count == 1 && restarted){
                restarted = 0;
           }
        }
    }
}


