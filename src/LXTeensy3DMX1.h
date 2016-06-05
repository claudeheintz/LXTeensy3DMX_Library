/* LXTeensy3DMX1.h
   Copyright 2016 by Claude Heintz Design
   All rights reserved.
   For license see LXTeensyDMX.h or http://www.claudeheintzdesign.com/lx/opensource.html
-----------------------------------------------------------------------------------
*/

#ifndef LXTeensy3DMX1_H
#define LXTeensy3DMX1_H

#include "LXTeensy3DMX.h"

typedef void (*LXRecvCallback)(int);


class LXTeensyDMX1 : public LXTeensyDMX {

  public:
  
	LXTeensyDMX1  ( void );
   ~LXTeensyDMX1 ( void );
   
   void startOutput( void );
   void startInput( void );
   void stop( void );
   
   void setMaxSlots (int slot);
	void setDataReceivedCallback(LXRecvCallback callback);
};

extern LXTeensyDMX1 Teensy3DMX1;


// functions
void serial_two_set_baud(uint32_t bit_rate);
void serial_two_begin(uint32_t bit_rate);
void serial_two_format(uint32_t format);
void serial_two_end(void);
void lx_uart1_status_isr(void);


#endif //LXTeensy3DMX1_H