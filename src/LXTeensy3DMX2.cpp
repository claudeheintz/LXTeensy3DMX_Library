/**************************************************************************/
/*!
    @file     LXTeensy3DMX2.cpp
    @author   Claude Heintz
    @license  See LXTeensy3DMX.h or http://www.claudeheintzdesign.com/lx/opensource.html
    @copyright 2016-2020 by Claude Heintz

    DMX Driver for Arduino using USART.

    @section  HISTORY

    v1.0 - First release
    v2.0 - abstracted UART Hardware
*/
/**************************************************************************/

#include "LXTeensy3DMX2.h"
#include <inttypes.h>
#include <stdlib.h>


/*********************************************************************
 * Global Variables
*/

uart_hardware_t UART2_hardware = {2, &KINETISK_UART2, &CORE_PIN7_CONFIG, &CORE_PIN8_CONFIG, SIM_SCGC4_UART2, IRQ_UART2_STATUS};
uart_hardware_t* UART2_Hardware = &UART2_hardware;

LXTeensyDMX2 Teensy3DMX2;

void lx_uart2_status_isr(void)
{
  Teensy3DMX2.uartISR();
}


//*****************************************************************************
// ************************  LXTeensyDMX2 member functions  ********************

LXTeensyDMX2::LXTeensyDMX2 ( void ) {
    _uart_hardware = UART2_Hardware;
    _isr_func = &lx_uart2_status_isr;
    
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

LXTeensyDMX2::~LXTeensyDMX2 ( void ) {
    stop();
}
