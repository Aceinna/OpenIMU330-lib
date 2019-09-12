/** ***************************************************************************
 * @file can.h 
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

#ifndef __CAN_H
#define __CAN_H
#include <stdint.h>
#include "GlobalConstants.h"


#define CAN_ERROR 			 1
#define CAN_NO_ERROR         0

#define USER_CAN_IDE         1
#define USER_CAN_RTR         0

// descriptor state
typedef enum {
  DESC_IDLE                  =   0,        // ready for being used
  DESC_OCCUPIED              =   1,        // unavailable
  DESC_PENDING               =   2         // in queue
} DESC_STATE;


#endif
