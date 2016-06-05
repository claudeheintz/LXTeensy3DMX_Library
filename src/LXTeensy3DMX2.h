/* LXTeensy3DMX2.h
   Copyright 2016 by Claude Heintz Design
   All rights reserved.
   For license see LXTeensyDMX.h or http://www.claudeheintzdesign.com/lx/opensource.html
-----------------------------------------------------------------------------------
*/

#ifndef LXTeensy3DMX2_H
#define LXTeensy3DMX2_H

#include "LXTeensy3DMX.h"

typedef void (*LXRecvCallback)(int);


class LXTeensyDMX2 : public LXTeensyDMX {

  public:
  
	LXTeensyDMX2  ( void );
   ~LXTeensyDMX2 ( void );
   
   void startOutput( void );
   void startInput( void );
   void stop( void );
   
   void setMaxSlots (int slot);
	void setDataReceivedCallback(LXRecvCallback callback);
};

extern LXTeensyDMX2 Teensy3DMX2;


// functions
void serial_three_set_baud(uint32_t bit_rate);
void serial_three_begin(uint32_t bit_rate);
void serial_three_format(uint32_t format);
void serial_three_end(void);
void lx_uart2_status_isr(void);


#endif //LXTeensy3DMX2_H