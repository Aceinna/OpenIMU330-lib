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
#include "stdlib.h"

//#define DEBUG_EXT_SYNC 1

TIM_HandleTypeDef      TimHandle;
TIM_HandleTypeDef      Tim2Handle;
TIM_HandleTypeDef      TimdHandle;
int NewTick = 0;
int imuCounter = 0;
BOOL extSync         = FALSE;
BOOL dacqScheduled   = FALSE; 
uint64_t systemTmrHi = 0LL;
uint32_t prevTstamp  = 0;
uint32_t ticksPerDacq  = 0;
    
uint64_t solutionTstamp = 0;
int      syncFreq = 0;
#define  PPS_FILT_SIZE 8
uint32_t ppsFiltr[PPS_FILT_SIZE] = {0,0,0,0,0,0,0,0};
int      syncFltIdx;
BOOL     syncP, resync;
int    adjStep = 0;
int    adjVal  = 0;
int    adjSign = 1;
double   ticksPerUs;
double   ticksPerPps;
static int     lock1 = 0, lock2 = 0, lock3 = 0;

void calculateSyncPhaseShift();
void adjustDacqSyncPhase();




void DelayMs(int ms)
{
    HAL_Delay(ms); 
}


int TIMER_IsDacqOverrun()
{
  return NewTick;
}

// timestamp in us
uint64_t platformGetCurrTimeStamp()
{
   
    ENTER_CRITICAL();

    uint32_t tmp = TIM2->CNT;
	uint64_t tstamp;

	if(tmp < prevTstamp){
		systemTmrHi += 0x0000000100000000LL;
	}
	
	prevTstamp = tmp;
    tstamp     = systemTmrHi | tmp; 
    tstamp     = (uint64_t)((double)tstamp/ticksPerUs + 0.5); 

    EXIT_CRITICAL();
    
    return tstamp;
}

// timestamp in us
uint64_t platformGetCurrTimeStampFromIsr()
{
    uint32_t tmp = TIM2->CNT;
	uint64_t tstamp;

    if(tmp < prevTstamp){
		systemTmrHi += 0x0000000100000000LL;
	}
	
	prevTstamp = tmp;
    tstamp     = systemTmrHi | tmp; 
    
    return (uint64_t)((double)tstamp/ticksPerUs + 0.5);
}

uint64_t platformGetFullTimeStampFromIsr()
{
    uint32_t tmp = TIM2->CNT;
	uint64_t tstamp;

    if(tmp < prevTstamp){
		systemTmrHi += 0x0000000100000000LL;
	}
	
	prevTstamp = tmp;
    tstamp     = systemTmrHi | tmp; 
    
    return tstamp;
}



uint64_t platformGetSolutionTstamp()
{
    return solutionTstamp;
}

// resolution 1 uS
double   platformGetSolutionTstampAsDouble()
{
    return (double)solutionTstamp/ticksPerUs;
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

    ticksPerPps  = SystemCoreClock;
    ticksPerUs   = (double)SystemCoreClock/1000000;
    ticksPerDacq = (uint32_t)(SystemCoreClock / (freq*8));
    TIM7_CLK_ENABLE();

    TimHandle.Instance               = TIM7;
    TimHandle.Init.Period            =  ticksPerDacq;
    TimHandle.Init.Prescaler         = 7;
    TimHandle.Init.ClockDivision     = 0;
    TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
    TimHandle.Init.RepetitionCounter = 0;
    TimHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    
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
uint64_t ppsTstamp            = 0LL;
uint64_t dacqTickTimeStamp    = 0LL;
uint64_t dacqPpsTimeStamp     = 0LL;

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
        dacqTickTimeStamp    = platformGetFullTimeStampFromIsr();
        if(ppsTstamp){
            dacqPpsTimeStamp  = ppsTstamp;
            ppsTstamp         = 0;
            calculateSyncPhaseShift();
        }
        adjustDacqSyncPhase();
        sensorSamplingTstamp = platformGetCurrTimeStampFromIsr();
        NewTick = 1;
        dacqScheduled = TRUE;
    }
    if(htim == &TimdHandle){
        activeDrdyEdgeTstamp = platformGetCurrTimeStampFromIsr();
    }
}

void TIM6_DAC_IRQHandler(void)
{
  OSEnterISR();
  HAL_TIM_IRQHandler(&TimdHandle);
  OSExitISR();
}


int  upperLimit;
int  lowerLimit;

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


BOOL ConfigureTimerForSyncCapture(int freq)
{
  TIM_IC_InitTypeDef   sICConfig;

  syncFreq = freq;

  upperLimit =  (int)((double)SystemCoreClock*1.001/freq + 0.5);   // +0.1% vs system clock
  lowerLimit =  (int)((double)SystemCoreClock*0.999/freq + 0.5);   // -0.1% vs system clock

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

  return TRUE;

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

#ifdef DEBUG_EXT_SYNC
#define NUM_RECORDS 64
int signLog[NUM_RECORDS];
int adjLog[NUM_RECORDS];
int deltaLog[NUM_RECORDS];
int logPtr = 0;
#endif

//uint32_t syncPulseTimeStamp = 0, syncPulsePeriod = 0;
static int    refDelta = 200;

void adjustDacqSyncPhase()
{
    int      offset = 0;
    uint32_t ddd;
//    static int cnt = 0;

    // no valid sync pulse detected
    // keep ols settings

    if(!syncP){
        return;
    }

    ddd = ticksPerPps/1600 + 0.5 + adjStep;     // adjStep * 0.1 uS
    if(adjVal){
        // 1Us per tick
        offset = adjSign;
        adjVal -= 1;
        ddd += offset;
    }

    TIM7->ARR = ddd;
}

void calculateSyncPhaseShift()
{

    int delta = dacqTickTimeStamp - dacqPpsTimeStamp; 


    if(!lock1){
        if(delta < 16000){  // about 200 uS
            // try to properly position phase shift
            adjStep = 20;   // 400 uS per second
        }else{
            lock1    = 1;
            adjStep  = -10; // -200 uS per second
        }
    }else if(!lock2){
        if(delta > 8000){     // aboit 100 uS
                adjStep = -3; // -60 uS per second
        }else {
            lock2   = 1;
            adjStep = -1;
        }
    }else if(!lock3){
        if(delta > 4000){  // about 50 uS
            adjStep = -1;  // -60 uS per second
            adjVal  = 50;  // -60 uS per second
            adjSign = -1;  // about  
        }else{
            lock3   = 1;
            adjStep = -1;  // -60 uS per second
            adjVal  = 50;  // -60 uS per second
        }
    }else{
        adjStep = -1;
        adjVal  = 5;       // about 0.5 uS/second 
        if(delta > 2000){   // 25 uS
            adjSign = -1;    // about  
        }else{
            adjSign = 1;
        }
    }

#ifdef DEBUG_EXT_SYNC
    adjLog[logPtr]    = adjStep;
    deltaLog[logPtr]  = delta;
    signLog[logPtr]   = adjSign;
    logPtr  += 1;
    logPtr  &= 0x3f; 
#endif

}                





void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    static int count = 0, refCounter = 5; // divide from 1000Hz to 200Hz
    static int restarted = 0;
    static uint32_t prevCap = 0;
    uint32_t curCap;
    static volatile int diff;
    static int          delta1 = 0;
    static uint32_t     cnt = 0;
    static uint32_t     syncPeriod = 0, syncAvg = 0, synced = 0;
    static int          syncCnt = 0;

    if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2 ){
        curCap  = TIM2->CCR2;
        if(!synced){
            diff    = curCap - prevCap;
            prevCap = curCap;
            if(diff < upperLimit && diff > lowerLimit){
			    // 5 consequtive matches
                count++;
                if(count >= 5 && dacqScheduled){
                    //switch right after last dacq interval was cheduled 
                    synced = 1;
                    if(syncFreq == 1000){
                    extSync = TRUE;
                    }
                    count       = 0;
                }else {
                    dacqScheduled = FALSE;
                }
            }else{
              count = 0;
            }
        }else{
           //sync achieved - start to sync system from external signal
            if(syncFreq == 1000){
           ++count;
           if(count >= refCounter){
               count       = 0;
                restarted  = 1;
                NewTick = 1;
                    sensorSamplingTstamp = platformGetCurrTimeStampFromIsr();
                    activeSyncEdgeTstamp = platformGetCurrTimeStampFromIsr();
           }
           if(count == 1 && restarted){
                restarted = 0;
           }
            }
            if(syncFreq == 1){
                while(1){
                    // delta between two periods of sync signal 
                    if(!prevCap){
                        prevCap = curCap; 
                        break;
                    }
                    delta1  = curCap - prevCap;
                    prevCap = curCap; 

                    if (delta1 < lowerLimit || delta1 > upperLimit){
                        // disregard pulse and use previous numbers; 
                        resync   = 1;
                        syncP    = TRUE; 
                        refDelta = 0;
                        break;
                    }
                    ppsTstamp            = platformGetFullTimeStampFromIsr();
                    syncAvg             -= ppsFiltr[syncFltIdx];
                    ppsFiltr[syncFltIdx] = delta1;
                    syncFltIdx++;
                    syncFltIdx          &= 0x7;
                    syncAvg             += delta1;
                    if(syncCnt < PPS_FILT_SIZE){
                        syncCnt ++;
                        syncPeriod = delta1;
                    }else {
                        syncPeriod = syncAvg >> 3;    // divide by 8
                    }
                    ticksPerPps = syncPeriod;
                    ticksPerDacq = syncPeriod/200;
                    ticksPerUs  = (double)syncPeriod/1000000;
                    syncPeriod >>=3;                  // divide by 8 to scale to reload period for tim7 
                    syncP  = TRUE; 
                    resync = 1;
//                    calculateSyncPhaseShift();
//                syncPulseTimeStamp = cap;
                    cnt ++;
                    break;
                }
            }
        }
    }
}


