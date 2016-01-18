/**************************************************************************/
/*!
    @file     LXTeensy32DMX.cpp
    @author   Claude Heintz
    @license  BSD (see LXTeensy32DMX.h)
    @copyright 2016 by Claude Heintz

    DMX Driver for Arduino using USART.

    @section  HISTORY

    v1.0 - First release
*/
/**************************************************************************/

#include "LXTeensy3DMX.h"
#include <inttypes.h>
#include <stdlib.h>

LXTeensyDMX Teensy3DMX;

//**************************************************************************************
//*** defines and functions adapted from from Teensyduino Core Library
//    (license at the bottom of this file)

// functions
void serial_one_set_baud(uint32_t bit_rate);
void serial_one_begin(uint32_t bit_rate);
void serial_one_format(uint32_t format);
void serial_one_end(void);
void lx_uart0_status_isr(void);

//defines
#define IRQ_PRIORITY  64  // 0 = highest priority, 255 = lowest

#define SERIAL_8N1 0x00
#define SERIAL_8N2 0x04

// UART0 and UART1 are clocked by F_CPU, UART2 is clocked by F_BUS
// UART0 has 8 byte fifo --unused in this implementation--, UART1 and UART2 have 1 byte buffer

#define C2_TX_ENABLE   UART_C2_TE
#define C2_RX_ENABLE   UART_C2_RE | UART_C2_RIE
#define C2_TX_ACTIVE    C2_TX_ENABLE | UART_C2_TIE
#define C2_TX_COMPLETING  C2_TX_ENABLE | UART_C2_TCIE
#define C2_TX_INACTIVE    C2_TX_ENABLE

#if defined(KINETISK)
#define BAUD2DIV(baud)  (((F_CPU * 2) + ((baud) >> 1)) / (baud))
#define BAUD2DIV2(baud) (((F_CPU * 2) + ((baud) >> 1)) / (baud))
#define BAUD2DIV3(baud) (((F_BUS * 2) + ((baud) >> 1)) / (baud))
#elif defined(KINETISL)

#if F_CPU <= 2000000
#define BAUD2DIV(baud)  (((F_PLL / 16 ) + ((baud) >> 1)) / (baud))
#elif F_CPU <= 16000000
#define BAUD2DIV(baud)  (((F_PLL / (F_PLL / 1000000)) + ((baud) >> 1)) / (baud))
#else
#define BAUD2DIV(baud)  (((F_PLL / 2 / 16) + ((baud) >> 1)) / (baud))
#endif

#define BAUD2DIV2(baud) (((F_BUS / 16) + ((baud) >> 1)) / (baud))
#define BAUD2DIV3(baud) (((F_BUS / 16) + ((baud) >> 1)) / (baud))
#endif

void serial_one_set_baud(uint32_t bit_rate) {
  uint32_t divisor = BAUD2DIV(bit_rate);
#if defined(HAS_KINETISK_UART0)
  UART0_BDH = (divisor >> 13) & 0x1F;
  UART0_BDL = (divisor >> 5) & 0xFF;
  UART0_C4 = divisor & 0x1F;
#elif defined(HAS_KINETISL_UART0)
  UART0_BDH = (divisor >> 8) & 0x1F;
  UART0_BDL = divisor & 0xFF;
#endif
}

void serial_one_begin(uint32_t bit_rate, uint8_t c2reg)
{
  SIM_SCGC4 |= SIM_SCGC4_UART0; // turn on clock, TODO: use bitband
  CORE_PIN0_CONFIG = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_PFE | PORT_PCR_MUX(3);
  CORE_PIN1_CONFIG = PORT_PCR_DSE | PORT_PCR_SRE | PORT_PCR_MUX(3);
  serial_one_set_baud(bit_rate);
#if defined(HAS_KINETISK_UART0)
	UART0_C1 = 0;
	UART0_PFIFO = 0;
#elif defined(HAS_KINETISL_UART0)
  UART0_C1 = 0;
#endif
  attachInterruptVector(IRQ_UART0_STATUS, lx_uart0_status_isr);
  UART0_C2 = c2reg;
  NVIC_SET_PRIORITY(IRQ_UART0_STATUS, IRQ_PRIORITY);
  NVIC_ENABLE_IRQ(IRQ_UART0_STATUS);
}

void serial_one_format(uint32_t format)
{
  uint8_t c;

  c = UART0_C1;
  c = (c & ~0x13) | (format & 0x03);  // configure parity
  if (format & 0x04) c |= 0x10;   // 9 bits (might include parity)
  UART0_C1 = c;
  if ((format & 0x0F) == 0x04) UART0_C3 |= 0x40; // 8N2 is 9 bit with 9th bit always 1
  c = UART0_S2 & ~0x10;
  if (format & 0x10) c |= 0x10;   // rx invert
  UART0_S2 = c;
  c = UART0_C3 & ~0x10;
  if (format & 0x20) c |= 0x10;   // tx invert
  UART0_C3 = c;
#ifdef SERIAL_9BIT_SUPPORT
  c = UART0_C4 & 0x1F;
  if (format & 0x08) c |= 0x20;   // 9 bit mode with parity (requires 10 bits)
  UART0_C4 = c;
  use9Bits = format & 0x80;
#endif
}

void serial_one_end(void)
{
  if (!(SIM_SCGC4 & SIM_SCGC4_UART0)) return;
  NVIC_DISABLE_IRQ(IRQ_UART0_STATUS);
  UART0_C2 = 0;
  CORE_PIN0_CONFIG = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_MUX(1);
  CORE_PIN1_CONFIG = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_MUX(1);
}


// ***************** define registers, flags and interrupts  ****************



	//***** baud rate defines
    #define DMX_DATA_BAUD		250000
    #define DMX_BREAK_BAUD 	90000
    //99900

    //***** states indicate current position in DMX stream
    #define DMX_STATE_BREAK 0
    #define DMX_STATE_START 1
    #define DMX_STATE_DATA 2
    #define DMX_STATE_IDLE 3

	//***** status is if interrupts are enabled and IO is active
    #define ISR_DISABLED 			0
    #define ISR_OUTPUT_ENABLED 	1
    #define ISR_INPUT_ENABLED 	2

// **************************** global data (can be accessed in ISR)  ***************

uint8_t*  _shared_dmx_data;
uint8_t   _shared_dmx_state;
uint16_t  _shared_dmx_slot;
uint16_t  _shared_max_slots = DMX_MIN_SLOTS;
LXRecvCallback _shared_receive_callback = NULL;


//************************************************************************************
// ************************  LXTeensyDMXOutput member functions  ********************

LXTeensyDMX::LXTeensyDMX ( void ) {
	_direction_pin = DIRECTION_PIN_NOT_USED;	//optional
	_shared_max_slots = DMX_MAX_SLOTS;
	_interrupt_status = ISR_DISABLED;
	
	//zero buffer including _dmxData[0] which is start code
    for (int n=0; n<DMX_MAX_SLOTS+1; n++) {
    	_dmxData[n] = 0;
    }
}

LXTeensyDMX::~LXTeensyDMX ( void ) {
    stop();
    _shared_dmx_data = NULL;
    _shared_receive_callback = NULL;
}


void LXTeensyDMX::startOutput ( void ) {
	if ( _direction_pin != DIRECTION_PIN_NOT_USED ) {
		digitalWrite(_direction_pin, HIGH);
	}
	if ( _interrupt_status == ISR_INPUT_ENABLED ) {
		stop();
	}
	if ( _interrupt_status == ISR_DISABLED ) {	//prevent messing up sequence if already started...
		serial_one_begin(DMX_BREAK_BAUD, C2_TX_ENABLE);
  		serial_one_format(SERIAL_8N2);

		_shared_dmx_data = dmxData();
		_shared_dmx_slot = 0;              
		_shared_dmx_state = DMX_STATE_BREAK;
		_interrupt_status = ISR_OUTPUT_ENABLED;
			
		UART0_D = 0x0;						//byte to tx register
  		UART0_C2 &= ~UART_C2_TIE;		//disable tx empty interrupt
  		UART0_C2 |= UART_C2_TCIE;		//enable transmission complete interrupt (end of break...)
	}
}

void LXTeensyDMX::startInput ( void ) {
	if ( _direction_pin != DIRECTION_PIN_NOT_USED ) {
		digitalWrite(_direction_pin, LOW);
	}
	if ( _interrupt_status == ISR_OUTPUT_ENABLED ) {
		stop();
	}
	if ( _interrupt_status == ISR_DISABLED ) {	//prevent messing up sequence if already started...
		_shared_dmx_data = dmxData();
		_shared_dmx_slot = 0;              
		_shared_dmx_state = DMX_STATE_IDLE;
		
		
		serial_one_begin(DMX_DATA_BAUD, C2_RX_ENABLE);
		serial_one_format(SERIAL_8N2);

		_interrupt_status = ISR_INPUT_ENABLED;
	}
}

void LXTeensyDMX::stop ( void ) { 
	NVIC_DISABLE_IRQ(IRQ_UART0_STATUS);
	UART0_C2 = 0;
	CORE_PIN0_CONFIG = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_MUX(1);
	CORE_PIN1_CONFIG = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_MUX(1);
	_interrupt_status = ISR_DISABLED;
}

void LXTeensyDMX::setDirectionPin( uint8_t pin ) {
	_direction_pin = pin;
	pinMode(_direction_pin, OUTPUT);
}

void LXTeensyDMX::setMaxSlots (int slots) {
	if ( slots > DMX_MIN_SLOTS ) {
		_shared_max_slots = slots;
	} else {
		_shared_max_slots = DMX_MIN_SLOTS;
	}
}

uint8_t LXTeensyDMX::getSlot (int slot) {
	return _dmxData[slot];
}

void LXTeensyDMX::setSlot (int slot, uint8_t value) {
	_dmxData[slot] = value;
}

uint8_t* LXTeensyDMX::dmxData(void) {
	return &_dmxData[0];
}

void LXTeensyDMX::setDataReceivedCallback(LXRecvCallback callback) {
	_shared_receive_callback = callback;
}


void lx_uart0_status_isr(void)
{
  uint8_t c;
  c = UART0_C2;
  if ((c & UART_C2_TIE) && (UART0_S1 & UART_S1_TDRE)) {   // transmit empty
    if ( _shared_dmx_state == DMX_STATE_DATA ) {
      UART0_D = _shared_dmx_data[_shared_dmx_slot++];	//send next slot
      if ( _shared_dmx_slot > _shared_max_slots ) {
      	_shared_dmx_state = DMX_STATE_IDLE;
			UART0_C2 &= ~UART_C2_TIE;
			UART0_C2 |= UART_C2_TCIE;			//switch to wait for tx complete
		}
    } else if ( _shared_dmx_state == DMX_STATE_BREAK ) {
      UART0_C2 &= ~UART_C2_TIE;
      UART0_C2 |= UART_C2_TCIE;			//switch to wait for tx complete
      UART0_D = 0;							//send break
    } else if ( _shared_dmx_state == DMX_STATE_START ) {
      UART0_D = 0;							// start code
      _shared_dmx_state = DMX_STATE_DATA;
      _shared_dmx_slot = 1;
    }
    
    return;
  }   		// transmit empty
  
  if ((c & UART_C2_TCIE) && (UART0_S1 & UART_S1_TC)) {    // transmit complete
    
    if ( _shared_dmx_state == DMX_STATE_IDLE ) {
      serial_one_set_baud(90000);
      _shared_dmx_state = DMX_STATE_BREAK;
      UART0_C2 &= ~UART_C2_TCIE;
      UART0_C2 |= UART_C2_TIE;
    } else if ( _shared_dmx_state == DMX_STATE_BREAK ) {
      serial_one_set_baud(250000);
      _shared_dmx_state = DMX_STATE_START;
      UART0_C2 &= ~UART_C2_TCIE;
      UART0_C2 |= UART_C2_TIE;
    }
    
    return;
  }    		// transmit complete
  
  uint8_t incoming_byte = UART0_D;					// read buffer to clear interrupt flag
  
  if ( UART0_S1 & UART_S1_FE ) {									// framing error
  		_shared_dmx_state = DMX_STATE_BREAK;
		if ( _shared_dmx_slot > 0 ) {
			if ( _shared_receive_callback != NULL ) {
				_shared_receive_callback(_shared_dmx_slot);
			}
		}
		_shared_dmx_slot = 0;
		return;
  }			// framing error
  
  if ( UART0_S1 & UART_S1_RDRF ) {								// receive register full
  		switch ( _shared_dmx_state ) {

			case DMX_STATE_BREAK:
				if ( incoming_byte == 0 ) {						//start code == zero (DMX)
					_shared_dmx_data[_shared_dmx_slot++] = incoming_byte;
					_shared_dmx_state = DMX_STATE_DATA;
					_shared_dmx_slot = 1;
				} else {
					_shared_dmx_state = DMX_STATE_IDLE;
				}
				break;
			
			case DMX_STATE_DATA:
				_shared_dmx_data[_shared_dmx_slot++] = incoming_byte;
				if ( _shared_dmx_slot > DMX_MAX_SLOTS ) {
					_shared_dmx_state = DMX_STATE_IDLE;			// go to idle, wait for next break
				}
				break;
		}
	}		// receive register full
	
}			//uart0_status_isr()


/* Teensyduino Core Library
 * http://www.pjrc.com/teensy/
 * Copyright (c) 2013 PJRC.COM, LLC.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * 1. The above copyright notice and this permission notice shall be 
 * included in all copies or substantial portions of the Software.
 *
 * 2. If the Software is incorporated into a build system that allows 
 * selection among a list of target devices, then similar target
 * devices manufactured by PJRC.COM must be included in the list of
 * target devices and selectable in the same manner.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
