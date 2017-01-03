/**************************************************************************/
/*!
    @file     LXTeensy3DMX.cpp
    @author   Claude Heintz
    @license  See LXTeensy3DMX.h or http://www.claudeheintzdesign.com/lx/opensource.html
    @copyright 2016 by Claude Heintz
    
    DMX Driver for Arduino using USART.

    @section  HISTORY

    v1.0 - First release
*/
/**************************************************************************/

#include "LXTeensy3DMX.h"
#include <inttypes.h>
#include <stdlib.h>


/*********************************************************************
 * Global Variables
*/

LXTeensyDMX Teensy3DMX;

volatile uint8_t*  _shared_dmx_data;
volatile uint8_t   _shared_dmx_state;
volatile uint16_t  _shared_dmx_slot;
volatile uint16_t  _shared_max_slots = DMX_MIN_SLOTS;
LXRecvCallback _shared_receive_callback = NULL;

/*********************************************************************
 * UART Serial Functions
 * derived from Teensyduino see LXTeensy3DMX.h
*/

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
  SIM_SCGC4 |= SIM_SCGC4_UART0;
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
  NVIC_SET_PRIORITY(IRQ_UART0_STATUS, DMX_UART_IRQ_PRIORITY);
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
      serial_one_set_baud(DMX_BREAK_BAUD);
      _shared_dmx_state = DMX_STATE_BREAK;
      UART0_C2 &= ~UART_C2_TCIE;
      UART0_C2 |= UART_C2_TIE;
    } else if ( _shared_dmx_state == DMX_STATE_BREAK ) {
      serial_one_set_baud(DMX_DATA_BAUD);
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


//*****************************************************************************
// ************************  LXTeensyDMX member functions  ********************

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