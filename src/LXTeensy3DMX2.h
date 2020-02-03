/* LXTeensy3DMX1.h
   Copyright 2016-2020 by Claude Heintz Design
   All rights reserved.
   For license see LXTeensyDMX.h or http://www.claudeheintzdesign.com/lx/opensource.html
-----------------------------------------------------------------------------------
*/

#ifndef LXTeensy3DMX2_H
#define LXTeensy3DMX2_H

#include "LXTeensy3DMX.h"


class LXTeensyDMX2 : public LXTeensyDMX {

  public:
  
	LXTeensyDMX2  ( void );
   ~LXTeensyDMX2 ( void );
   
};

extern LXTeensyDMX2 Teensy3DMX2;


// isr function
void lx_uart2_status_isr(void);


#endif //LXTeensy3DMX2_H