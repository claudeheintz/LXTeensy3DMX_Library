/* LXTeensy3DMX1.h
   Copyright 2016-2020 by Claude Heintz Design
   All rights reserved.
   For license see LXTeensyDMX.h or http://www.claudeheintzdesign.com/lx/opensource.html
-----------------------------------------------------------------------------------
*/

#ifndef LXTeensy3DMX1_H
#define LXTeensy3DMX1_H

#include "LXTeensy3DMX.h"


class LXTeensyDMX1 : public LXTeensyDMX {

  public:
  
	LXTeensyDMX1  ( void );
   ~LXTeensyDMX1 ( void );
   
};

extern LXTeensyDMX1 Teensy3DMX1;


// isr function
void lx_uart1_status_isr(void);


#endif //LXTeensy3DMX1_H