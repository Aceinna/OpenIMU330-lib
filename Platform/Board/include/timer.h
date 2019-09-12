/** ***************************************************************************
 * @file timer.h 
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

#ifndef __TIMER_H
#define __TIMER_H

#include <stdint.h>

int   dacqTimerInit(int period);
int   dacqTimerStart();
void  timerElapsedCallback();
void  ConfigureTimerFor1PPSCapture();
int   ConfigureLinkedTimers(int freq);
void  ConfigureReferenceTimer();
void  StartLinkedTimers();
int   GetSensToPpsDelay();
int   GetPpsToDrdyDelay();
int   WaitForNewTick();
int   TimeNow();
int   IsOverrun();
uint32_t GetSensorsSamplingTstamp();

#endif
