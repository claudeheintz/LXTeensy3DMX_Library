/**************************************************************************/
/*!
    @file     LXTeensy3DMX1.cpp
    @author   Claude Heintz
    @license  See LXTeensy3DMX.h or http://www.claudeheintzdesign.com/lx/opensource.html
    @copyright 2016-2020 by Claude Heintz

    DMX Driver for Arduino using USART.

    @section  HISTORY

    v1.0 - First release
    v2.0 - abstracted UART Hardware
*/
/**************************************************************************/

#include "LXTeensy3DMX1.h"
#include <inttypes.h>
#include <stdlib.h>


/*********************************************************************
 * Global Variables
*/

uart_hardware_t UART1_hardware = {1, &KINETISK_UART1, &CORE_PIN9_CONFIG, &CORE_PIN10_CONFIG, SIM_SCGC4_UART1, IRQ_UART1_STATUS};
uart_hardware_t* UART1_Hardware = &UART1_hardware;

LXTeensyDMX1 Teensy3DMX1;

/*********************************************************************
 * UART Serial Functions
 * derived from Teensyduino see LXTeensy3DMX.h
*/

void lx_uart1_status_isr(void)
{
  Teensy3DMX1.uartISR();
}


//*****************************************************************************
// ************************  LXTeensyDMX1 member functions  ********************

LXTeensyDMX1::LXTeensyDMX1 ( void ) {
    _uart_hardware = UART1_Hardware;
    _isr_func = &lx_uart1_status_isr;
    
	_direction_pin = DIRECTION_PIN_NOT_USED;	//optional
	_slots = DMX_MAX_SLOTS;
	_interrupt_status = ISR_DISABLED;
	_receive_callback = NULL;
	_rdm_receive_callback = NULL;
	
	//zero buffer including _dmxData[0] which is start code
    for (int n=0; n<DMX_MAX_SLOTS+1; n++) {
    	_dmxData[n] = 0;
    }
}

LXTeensyDMX1::~LXTeensyDMX1 ( void ) {
    stop();
}
