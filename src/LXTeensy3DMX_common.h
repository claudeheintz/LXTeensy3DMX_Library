/* LXTeensy3DMX_utility.h
   Copyright 2017-2020 by Claude Heintz Design
   All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of LXTeensyDMX nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef LXTeensy3DMX_H_UTIL
#define LXTeensy3DMX_H_UTIL

#include "kinetis.h"
#include "core_pins.h"

#define DMX_MIN_SLOTS 24
#define DMX_MAX_SLOTS 512
#define DMX_MAX_FRAME 513
#define RDM_MAX_FRAME 513

#define DIRECTION_PIN_NOT_USED 255

 //***** states indicate current position in DMX stream
#define DMX_READ_STATE_IDLE 0
#define DMX_READ_STATE_RECEIVING 1

#define DMX_TASK_RECEIVE		0   
#define DMX_TASK_SEND			1
#define DMX_TASK_SEND_RDM		2
#define DMX_TASK_SET_SEND		3
#define DMX_TASK_SET_SEND_RDM	4

#define RDM_NO_DISCOVERY		0
#define RDM_PARTIAL_DISCOVERY	1
#define RDM_DID_DISCOVER		2


#define RDM_DIRECTION_INPUT		0
#define RDM_DIRECTION_OUTPUT	1

#define RX_SIGNAL_NORMAL		0
#define RX_SIGNAL_INVERTED		1   
 
 #endif // ifndef LXTeensy3DMX_H_UTIL
