/**************************************************************************/
/*!
    @file     LXTeensy3DMX.cpp
    @author   Claude Heintz
    @license  See LXTeensy3DMX.h or http://www.claudeheintzdesign.com/lx/opensource.html
    @copyright 2016-2020 by Claude Heintz
    
    DMX Driver for Arduino using USART.

    @section  HISTORY

    v1.0 - First release
    v2.0 - abstracted UART Hardware
    
*/
/**************************************************************************/

#include "LXTeensy3DMX.h"
#include <rdm/rdm_utility.h>
#include <inttypes.h>
#include <stdlib.h>


/*********************************************************************
 * Global&static Variables
*/

uart_hardware_t UART0_hardware = {0, &KINETISK_UART0, &CORE_PIN0_CONFIG, &CORE_PIN1_CONFIG, SIM_SCGC4_UART0, IRQ_UART0_STATUS};
uart_hardware_t* UART0_Hardware = &UART0_hardware;

LXTeensyDMX Teensy3DMX;

UID LXTeensyDMX::THIS_DEVICE_ID(0x6C, 0x78, 0x00, 0x00, 0x00, 0x03);

/*********************************************************************
 * UART Serial Functions
 * derived from Teensyduino see LXTeensy3DMX.h
*/


void hardware_uart_set_bits_in_bdh(KINETISK_UART_t* uart_reg_ptr, uint8_t bits) {		// for two stop bits pass UART_BDH_SBNS
	uint8_t bdl = uart_reg_ptr->BDL;
	uart_reg_ptr->BDH |= bits;		// Turn on 2 stop bits - was turned off by set baud
	uart_reg_ptr->BDL = bdl;		// Says BDH not acted on until BDL is written
}

void hardware_uart_set_baud(uint8_t uart_num, KINETISK_UART_t * uart_reg_ptr, uint32_t bit_rate) {

  uint32_t divisor ;
  if ( uart_num == 0 ) {
    divisor = BAUD2DIV(bit_rate);
  } else if ( uart_num == 1 ) {
    divisor = BAUD2DIV2(bit_rate);
  } else {
    divisor = BAUD2DIV3(bit_rate);
  }
  if (divisor < 32) divisor = 32;
#if defined(HAS_KINETISK_UART0)
  uart_reg_ptr->BDH = (divisor >> 13) & 0x1F;	//UART0_BDH
  uart_reg_ptr->BDL = (divisor >> 5) & 0xFF;	//UART0_BDL	
  uart_reg_ptr->C4 = divisor & 0x1F;			//UART0_C4
#elif defined(HAS_KINETISL_UART0)
  uart_reg_ptr->BDH  = (divisor >> 8) & 0x1F;
  uart_reg_ptr->BDL = divisor & 0xFF;
#endif
}

void hardware_uart_set_baud_2s(uint8_t uart_num, KINETISK_UART_t * uart_reg_ptr, uint32_t bit_rate) {

	hardware_uart_set_baud(uart_num, uart_reg_ptr, bit_rate);
	
	// restore 2 stop bit mode when changing baud from DMX to Break
	#if defined(__MK64FX512__) || defined(__MK66FX1M0__) || defined(KINETISL)
	// For T3.5/T3.6/TLC See about turning on 2 stop bit mode
		hardware_uart_set_bits_in_bdh(uart_reg_ptr, UART_BDH_BITS);
    #endif
}

void hardware_uart_begin(uart_hardware_t* uart_hardware, void isr_func(void), uint32_t bit_rate, uint8_t c2reg)
{
  SIM_SCGC4 |= uart_hardware->uart_clk_bit;
  *(uart_hardware->pconfig0) = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_PFE | PORT_PCR_MUX(3);
  *(uart_hardware->pconfig1) = PORT_PCR_DSE | PORT_PCR_SRE | PORT_PCR_MUX(3);
  hardware_uart_set_baud(uart_hardware->uart_num, uart_hardware->uart_reg_ptr, bit_rate);
#if defined(HAS_KINETISK_UART0)
	uart_hardware->uart_reg_ptr->C1 = 0;
	uart_hardware->uart_reg_ptr->PFIFO = 0;				//no fifo, need to flush RX & TX per manual??
#elif defined(HAS_KINETISL_UART0)
  uart_hardware->uart_reg_ptr->C1 = 0;
#endif
  attachInterruptVector(uart_hardware->status_num, isr_func);
  uart_hardware->uart_reg_ptr->C2 = c2reg;
  NVIC_SET_PRIORITY(uart_hardware->status_num, DMX_UART_IRQ_PRIORITY);
  NVIC_ENABLE_IRQ(uart_hardware->status_num);
}

void hardware_uart_format(KINETISK_UART_t * uart_reg_ptr, uint32_t format)
{
  uint8_t c;

  c = uart_reg_ptr->C1;
  c = (c & ~0x13) | (format & 0x03);  // configure parity
  if (format & 0x04) c |= 0x10;   // 9 bits (might include parity)
  uart_reg_ptr->C1 = c;
  if ((format & 0x0F) == 0x04) uart_reg_ptr->C3 |= 0x40; // 8N2 is 9 bit with 9th bit always 1
  c = uart_reg_ptr->S2 & ~0x10;
  if (format & 0x10) c |= 0x10;   // rx invert
  uart_reg_ptr->S2 = c;
  c = uart_reg_ptr->C3 & ~0x10;
  if (format & 0x20) c |= 0x10;   // tx invert
  uart_reg_ptr->C3 = c;
#ifdef SERIAL_9BIT_SUPPORT
#warning SERIAL_9BIT_SUPPORT
  c = uart_reg_ptr->C4 & 0x1F;
  if (format & 0x08) c |= 0x20;   // 9 bit mode with parity (requires 10 bits)
  uart_reg_ptr->C4 = c;(KINETISK_UART_t* uart_reg_ptr, uint32_t format)
  use9Bits = format & 0x80;
#endif
#if defined(__MK64FX512__) || defined(__MK66FX1M0__) || defined(KINETISL)
	// For T3.5/T3.6/TLC See about turning on 2 stop bit mode
	if ( format & 0x100) {							// 0x100 is 2 stop bits if this is compiled
		hardware_uart_set_bits_in_bdh(uart_reg_ptr, UART_BDH_BITS);
	}
#endif
}

void hardware_serial_end(uart_hardware_t* uart_hardware)
{
  if (!(SIM_SCGC4 & uart_hardware->uart_clk_bit)) return;
  
  NVIC_DISABLE_IRQ(uart_hardware->status_num);
  uart_hardware->uart_reg_ptr->C2 = 0;
  *(uart_hardware->pconfig0) = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_MUX(1);
  *(uart_hardware->pconfig1) = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_MUX(1);
}

void lx_uart0_status_isr(void)
{
  Teensy3DMX.uartISR();
}					//uart0_status_isr()


//*****************************************************************************
// ************************  LXTeensyDMX member functions  ********************

/*
LXTeensyDMX::LXTeensyDMX ( uart_hardware_t*  u ) {
	LXTeensyDMX( (uart_hardware_t* )NULL);
}*/

LXTeensyDMX::LXTeensyDMX (  ) {
    _uart_hardware = UART0_Hardware;
    _isr_func = &lx_uart0_status_isr;
    
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

LXTeensyDMX::~LXTeensyDMX ( void ) {
    stop();
}

void LXTeensyDMX::startOutput ( void ) {
	if ( _direction_pin != DIRECTION_PIN_NOT_USED ) {
		digitalWrite(_direction_pin, HIGH);
	}
	if ( _interrupt_status == ISR_INPUT_ENABLED ) {
		stop();
	}
	if ( _interrupt_status == ISR_DISABLED ) {	//prevent messing up sequence if already started...
		hardware_uart_begin(_uart_hardware, _isr_func, DMX_BREAK_BAUD, C2_TX_ENABLE);
  		hardware_uart_format(_uart_hardware->uart_reg_ptr, SERIAL_8N2);

		//_shared_dmx_data = dmxData();
		_rdm_task_mode = DMX_TASK_SEND;
		_next_slot = 0;              
		_dmx_state = DMX_STATE_BREAK;
		_interrupt_status = ISR_OUTPUT_ENABLED;
			
		_uart_hardware->uart_reg_ptr->D = 0x0;						//byte to tx register
  		_uart_hardware->uart_reg_ptr->C2 &= ~UART_C2_TIE;		//disable tx empty interrupt
  		_uart_hardware->uart_reg_ptr->C2 |= UART_C2_TCIE;		//enable transmission complete interrupt (end of break...)
	}
}

void LXTeensyDMX::startInput ( uint8_t invert_rx ) {
	if ( _direction_pin != DIRECTION_PIN_NOT_USED ) {
		digitalWrite(_direction_pin, LOW);
	}
	if ( _interrupt_status == ISR_OUTPUT_ENABLED ) {
		stop();
	}
	if ( _interrupt_status == ISR_DISABLED ) {	//prevent messing up sequence if already started..
		//_shared_dmx_data = dmxData();
		_rdm_task_mode = DMX_TASK_RECEIVE;
		_rdm_read_handled = 0;
		_current_slot = 0;              
		_dmx_state = DMX_STATE_IDLE;
		
		hardware_uart_begin(_uart_hardware, lx_uart0_status_isr, DMX_BREAK_BAUD, C2_RX_ENABLE);
		hardware_uart_format(_uart_hardware->uart_reg_ptr, SERIAL_8N2);

		_interrupt_status = ISR_INPUT_ENABLED;
		
		if ( invert_rx ) {
			_uart_hardware->uart_reg_ptr->S2 |= 0x10;   					// rx inverted signal
		}
	}
}

void LXTeensyDMX::startRDM( uint8_t pin, uint8_t direction, uint8_t invert_rx ) {
	pinMode(pin, OUTPUT);
	_direction_pin = pin;
	if ( direction ) {
		startOutput();							//enables transmit interrupt
		_uart_hardware->uart_reg_ptr->C2 |= C2_RX_ENABLE;				//enable receive interrupt
		
		if ( invert_rx ) {
			_uart_hardware->uart_reg_ptr->S2 |= 0x10;   					// rx inverted signal
		}
	} else {
		startInput(invert_rx);
	}
}

void LXTeensyDMX::stop ( void ) { 
	NVIC_DISABLE_IRQ(_uart_hardware->status_num);
  _uart_hardware->uart_reg_ptr->C2 = 0;
  *(_uart_hardware->pconfig0) = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_MUX(1);
  *(_uart_hardware->pconfig1) = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_MUX(1);
	_interrupt_status = ISR_DISABLED;
}

void LXTeensyDMX::setDirectionPin( uint8_t pin ) {
	_direction_pin = pin;
	pinMode(_direction_pin, OUTPUT);
}

int LXTeensyDMX::numberOfSlots ( void ) {
	return _slots;
}

void LXTeensyDMX::setMaxSlots (int slots) {
	if ( slots > DMX_MIN_SLOTS ) {
		_slots = slots;
	} else {
		_slots = DMX_MIN_SLOTS;
	}
}

uint8_t LXTeensyDMX::getSlot (int slot) {
	return _dmxData[slot];
}

void LXTeensyDMX::setSlot (int slot, uint8_t value) {
	_dmxData[slot] = value;
}

uint8_t* LXTeensyDMX::dmxData(void) {
	return _dmxData;
}

uint8_t* LXTeensyDMX::rdmData(void) {
	return _rdmData;
}

uint8_t* LXTeensyDMX::receivedData(void) {
	return _receivedData;
}

uint8_t* LXTeensyDMX::rdmPacket(void) {
	return _rdmPacket;
}

int LXTeensyDMX::nextReadSlot(void) {
	return _current_slot;
}

/*************************** sending ***********************************/

void LXTeensyDMX::transmitEmpty( void ) {
	if ( _rdm_task_mode == DMX_TASK_SEND_RDM ) {

		if ( _dmx_state == DMX_STATE_DATA ) {
		  _uart_hardware->uart_reg_ptr->D = _rdmPacket[_next_slot++];	//send next slot
		  if ( _next_slot >= _rdm_len ) {		//0 based index
			 _dmx_state = DMX_STATE_IDLE;
			 _uart_hardware->uart_reg_ptr->C2 &= ~UART_C2_TIE;
			 _uart_hardware->uart_reg_ptr->C2 |= UART_C2_TCIE;			//switch to wait for tx complete
		  }
		} else if ( _dmx_state == DMX_STATE_BREAK ) {
		  _uart_hardware->uart_reg_ptr->C2 &= ~UART_C2_TIE;
		  _uart_hardware->uart_reg_ptr->C2 |= UART_C2_TCIE;			//switch to wait for tx complete
		  _uart_hardware->uart_reg_ptr->D = 0;						//send break
		} else if ( _dmx_state == DMX_STATE_START ) {
		  _uart_hardware->uart_reg_ptr->D = _rdmPacket[0];			// start code
		  _dmx_state = DMX_STATE_DATA;
		  _next_slot = 1;
		}	
		
	} else if ( _rdm_task_mode ) {					//should be DMX_TASK_SEND even if not RDM
		
		if ( _dmx_state == DMX_STATE_DATA ) {
		  _uart_hardware->uart_reg_ptr->D = _dmxData[_next_slot++];	//send next slot
		  if ( _next_slot > _slots ) {		//slots are 1 based index so OK to use > , index[512] is slot 512
			 _dmx_state = DMX_STATE_IDLE;
			 _uart_hardware->uart_reg_ptr->C2 &= ~UART_C2_TIE;
			 _uart_hardware->uart_reg_ptr->C2 |= UART_C2_TCIE;			//switch to wait for tx complete
		  }
		} else if ( _dmx_state == DMX_STATE_BREAK ) {
		  _uart_hardware->uart_reg_ptr->C2 &= ~UART_C2_TIE;
		  _uart_hardware->uart_reg_ptr->C2 |= UART_C2_TCIE;			//switch to wait for tx complete
		  _uart_hardware->uart_reg_ptr->D = 0;						//send break
		} else if ( _dmx_state == DMX_STATE_START ) {
		  _uart_hardware->uart_reg_ptr->D = _dmxData[0];		// start code
		  _dmx_state = DMX_STATE_DATA;
		  _next_slot = 1;
		}
		
	}	// _rdm_task_mode
}
   

void LXTeensyDMX::transmitComplete( void ) {
	if ( _rdm_task_mode == DMX_TASK_SEND_RDM ) {

		if ( _dmx_state == DMX_STATE_IDLE ) {
		  _dmx_state = DMX_STATE_IDLE;				//sets baud to break next transmit complete interrupt
		  // Packet complete, change to receive

		  _uart_hardware->uart_reg_ptr->C2 &= ~UART_C2_TCIE;				//disable send interrupts
		  _uart_hardware->uart_reg_ptr->C2 &= ~UART_C2_TIE;
		  
		  digitalWrite(_direction_pin, LOW);		// call from interrupt only because receiving starts
		  _current_slot = 0;						// and these flags need to be set
		  _packet_length = DMX_MAX_FRAME;			// but no bytes read from fifo until next task loop
		  if ( _rdm_read_handled ) {
		     _dmx_read_state = DMX_READ_STATE_RECEIVING;
		  } else {
		     _dmx_read_state = DMX_READ_STATE_IDLE;		// if not after controller message, wait for a break
		  }										// signaling start of packet
		  _rdm_task_mode = DMX_TASK_RECEIVE;
		  
		} else if ( _dmx_state == DMX_STATE_BREAK ) {
		  hardware_uart_set_baud_2s(_uart_hardware->uart_num, _uart_hardware->uart_reg_ptr, DMX_DATA_BAUD);
		  _dmx_state = DMX_STATE_START;
		  _uart_hardware->uart_reg_ptr->C2 &= ~UART_C2_TCIE;
		  _uart_hardware->uart_reg_ptr->C2 |= UART_C2_TIE;
		}
		
	} else if ( _rdm_task_mode ) {					//should be DMX_TASK_SEND if sending and not using RDM

		if ( _dmx_state == DMX_STATE_IDLE ) {
		  hardware_uart_set_baud_2s(_uart_hardware->uart_num, _uart_hardware->uart_reg_ptr, DMX_BREAK_BAUD);
		  _dmx_state = DMX_STATE_BREAK;
		  _uart_hardware->uart_reg_ptr->C2 &= ~UART_C2_TCIE;
		  _uart_hardware->uart_reg_ptr->C2 |= UART_C2_TIE;
		  
		  // Packet complete, change _rdm_task_mode if flag indicates
		  if ( _rdm_task_mode == DMX_TASK_SET_SEND ) {
		  	_rdm_task_mode = DMX_TASK_SEND;
		  } else if ( _rdm_task_mode == DMX_TASK_SET_SEND_RDM ) {
		  	_rdm_task_mode = DMX_TASK_SEND_RDM;
		  }
		  
		} else if ( _dmx_state == DMX_STATE_BREAK ) {
		  hardware_uart_set_baud_2s(_uart_hardware->uart_num, _uart_hardware->uart_reg_ptr, DMX_DATA_BAUD);
		  _dmx_state = DMX_STATE_START;
		  _uart_hardware->uart_reg_ptr->C2 &= ~UART_C2_TCIE;
		  _uart_hardware->uart_reg_ptr->C2 |= UART_C2_TIE;
		}
		
	}
}


/*************************** receiving ***********************************/

uint8_t* LXTeensyDMX::printReceivedData( void ) {
	return _receivedData;
}

void LXTeensyDMX::packetComplete( void ) {
	if ( _receivedData[0] == 0 ) {				//zero start code is DMX
		if ( _rdm_read_handled == 0 ) {			// not handled by specific method
			_slots = _current_slot - 1;				//_current_slot represents next slot so subtract one
			for(int j=0; j<_current_slot; j++) {	//copy dmx values from read buffer
				_dmxData[j] = _receivedData[j];
			}
	
			if ( _receive_callback != NULL ) {
				_receive_callback(_slots);
			}
		}
	} else if ( _receivedData[0] == RDM_START_CODE ) {			//zero start code is RDM
		if ( _rdm_read_handled == 0 ) {					// not handled by specific method
			if ( validateRDMPacket(_receivedData) ) {	// evaluate checksum
				uint8_t plen = _receivedData[2] + 2;
				for(int j=0; j<plen; j++) {
					_rdmData[j] = _receivedData[j];
					if ( _receive_callback != NULL ) {
						_rdm_receive_callback(plen);
					}
				}
			}		// validRDM
		}			// ! _rdm_read_handled	
	}				// RDM_START_CODE
	resetFrame();
}

void LXTeensyDMX::resetFrame( void ) {		
	_dmx_read_state = DMX_READ_STATE_IDLE;						// insure wait for next break
}

void LXTeensyDMX::breakReceived( void ) {
	if ( _dmx_read_state == DMX_READ_STATE_RECEIVING ) {	// break has already been detected
		if ( _current_slot > 1 ) {						// break before end of maximum frame
			if ( _receivedData[0] == 0 ) {				// zero start code is DMX
				packetComplete();						// packet terminated with slots<512
			}
		}
	}
	_dmx_read_state = DMX_READ_STATE_RECEIVING;
	_current_slot = 0;
	_packet_length = DMX_MAX_FRAME;						// default to receive complete frame
}

void LXTeensyDMX::byteReceived(uint8_t c) {
	if ( _dmx_read_state == DMX_READ_STATE_RECEIVING ) {
		_receivedData[_current_slot] = c;
		if ( _current_slot == 2 ) {						//RDM length slot
			if ( _receivedData[0] == RDM_START_CODE ) {			//RDM start code
				if ( _rdm_read_handled == 0 ) {
					_packet_length = c + 2;				//add two bytes for checksum
				}
			} else if ( _receivedData[0] == 0xFE ) {	//RDM Discovery Response
				_packet_length = DMX_MAX_FRAME;
			} else if ( _receivedData[0] != 0 ) {		// if Not Null Start Code
				_dmx_read_state = DMX_STATE_IDLE;			//unrecognized, ignore packet
			}
		}
	
		_current_slot++;
		if ( _current_slot >= _packet_length ) {		//reached expected end of packet
			packetComplete();
		}
	}
}

void LXTeensyDMX::setDataReceivedCallback(LXRecvCallback callback) {
	_receive_callback = callback;
}


/************************************ RDM Methods **************************************/

void LXTeensyDMX::setRDMReceivedCallback(LXRecvCallback callback) {
	_rdm_receive_callback = callback;
}

uint8_t LXTeensyDMX::rdmTaskMode( void ) {		// applies to bidirectional RDM connection
	return _rdm_task_mode;
}

void LXTeensyDMX::setTaskSendDMX( void ) {		// only valid if connection started using startRDM()
	digitalWrite(_direction_pin, HIGH);
	 _rdm_task_mode = DMX_TASK_SEND;
}


void LXTeensyDMX::restoreTaskSendDMX( void ) {		// only valid if connection started using startRDM()
	digitalWrite(_direction_pin, HIGH);
	_dmx_state = DMX_STATE_IDLE;
	_rdm_task_mode = DMX_TASK_SET_SEND;
	 
	 //restore the interrupts
	_uart_hardware->uart_reg_ptr->C2 &= ~UART_C2_TIE;		//disable tx empty interrupt
	_uart_hardware->uart_reg_ptr->C2 |= UART_C2_TCIE;		//enable transmission complete interrupt (end of break...)
  							
	 do {
	 	delay(1);
	 } while ( _rdm_task_mode != DMX_TASK_SEND );	//set to send on interrupt pass after first DMX frame sent
}

void LXTeensyDMX::setTaskReceive( void ) {		// only valid if connection started using startRDM()
	_current_slot = 0;
	_packet_length = DMX_MAX_FRAME;
    _dmx_state = DMX_STATE_IDLE;
    _rdm_task_mode = DMX_TASK_RECEIVE;
    _rdm_read_handled = 0;
    
    
    digitalWrite(_direction_pin, LOW);
}

void LXTeensyDMX::sendRawRDMPacket( uint8_t len ) {		// only valid if connection started using startRDM()
	_rdm_len = len;
	// calculate checksum:  len should include 2 bytes for checksum at the end
	uint16_t checksum = rdmChecksum(_rdmPacket, _rdm_len-2);
	_rdmPacket[_rdm_len-2] = checksum >> 8;
	_rdmPacket[_rdm_len-1] = checksum & 0xFF;

	if ( _rdm_task_mode ) {						//already sending, flag to send RDM
		_rdm_task_mode = DMX_TASK_SET_SEND_RDM;
	} else {
		_rdm_task_mode = DMX_TASK_SEND_RDM;
		digitalWrite(_direction_pin, HIGH);
		_dmx_state = DMX_STATE_BREAK;
		 //set the interrupts			//will be disable when packet is complete
		_uart_hardware->uart_reg_ptr->C2 &= ~UART_C2_TIE;		//disable tx empty interrupt
		_uart_hardware->uart_reg_ptr->C2 |= UART_C2_TCIE;		//enable transmission complete interrupt (end of break...)
	}
	
	while ( _rdm_task_mode ) {	//wait for packet to be sent and listening to start
		delay(2);				//_rdm_task_mode is set to 0 (receive) after RDM packet is completely sent
	}
}

void  LXTeensyDMX::setupRDMControllerPacket(uint8_t* pdata, uint8_t msglen, uint8_t port, uint16_t subdevice) {
	pdata[RDM_IDX_START_CODE]		= RDM_START_CODE;
  	pdata[RDM_IDX_SUB_START_CODE]	= RDM_SUB_START_CODE;
  	pdata[RDM_IDX_PACKET_SIZE]		= msglen;
  	
  	// must set target UID outside this method
  	UID::copyFromUID(THIS_DEVICE_ID, pdata, RDM_IDX_SOURCE_UID);
  	
  	pdata[RDM_IDX_TRANSACTION_NUM]	= _transaction++;
  	pdata[RDM_IDX_PORT]				= port;
  	pdata[RDM_IDX_MSG_COUNT]		= 0x00;		//(always zero for controller msgs)
  	pdata[RDM_IDX_SUB_DEV_MSB] 		= subdevice >> 8;
  	pdata[RDM_IDX_SUB_DEV_LSB] 		= subdevice & 0xFF;
  	// total always 20 bytes
}

void  LXTeensyDMX::setupRDMMessageDataBlock(uint8_t* pdata, uint8_t cmdclass, uint16_t pid, uint8_t pdl) {
	pdata[RDM_IDX_CMD_CLASS] 		= cmdclass;
  	pdata[RDM_IDX_PID_MSB] 			= (pid >> 8) & 0xFF;
  	pdata[RDM_IDX_PID_LSB]			 = pid & 0xFF;
  	pdata[RDM_IDX_PARAM_DATA_LEN] 	= pdl;
  	// total always 4 bytes
}

uint8_t LXTeensyDMX::sendRDMDiscoveryPacket(UID lower, UID upper, UID* single) {
	uint8_t rv = RDM_NO_DISCOVERY;
	uint8_t j;
	
	//Build RDM packet
	setupRDMControllerPacket(_rdmPacket, RDM_DISC_UNIQUE_BRANCH_MSGL, RDM_PORT_ONE, RDM_ROOT_DEVICE);
	UID::copyFromUID(BROADCAST_ALL_DEVICES_ID, _rdmPacket, 3);
	setupRDMMessageDataBlock(_rdmPacket, RDM_DISCOVERY_COMMAND, RDM_DISC_UNIQUE_BRANCH, RDM_DISC_UNIQUE_BRANCH_PDL);
  	UID::copyFromUID(lower, _rdmPacket, 24);
  	UID::copyFromUID(upper, _rdmPacket, 30);
	
	_rdm_read_handled = 1;
	sendRawRDMPacket(RDM_DISC_UNIQUE_BRANCH_PKTL);
	delay(2);

	// any bytes read indicate response to discovery packet
	// check if a single, complete, uncorrupted packet has been received
	// otherwise, refine discovery search
	
	if ( _current_slot ) {
		rv = RDM_PARTIAL_DISCOVERY;
		
		// find preamble separator
		for(j=0; j<8; j++) {
			if ( _receivedData[j] == RDM_DISC_PREAMBLE_SEPARATOR ) {
				break;
			}
		}
		// 0-7 bytes preamble
		if ( j < 8 ) {
			if ( _current_slot == j + 17 ) { //preamble separator plus 16 byte payload
				uint8_t bindex = j + 1;
				
				//calculate checksum of 12 slots representing UID
				uint16_t checksum = rdmChecksum(&_receivedData[bindex], 12);
				
				//convert dual bytes to payload of single bytes
				uint8_t payload[8];
				for (j=0; j<8; j++) {
					payload[j] = _receivedData[bindex] & _receivedData[bindex+1];
					bindex += 2;
				}

				if ( testRDMChecksum( checksum, payload, 6 ) ) {
					//copy UID into uldata
					rv = RDM_DID_DISCOVER;
					*single = payload;
				}
			}
		}			// j<8
		
		_rdm_read_handled = 0;
		resetFrame();
	} else {
		_rdm_read_handled = 0;
	}

	restoreTaskSendDMX();
	return rv;
}

uint8_t LXTeensyDMX::sendRDMDiscoveryMute(UID target, uint8_t cmd) {
	uint8_t rv = 0;

	//Build RDM packet
	// total packet length 0 parameter is 24 (+cksum =26 for sendRawRDMPacket) 
	setupRDMControllerPacket(_rdmPacket, RDM_PKT_BASE_MSG_LEN, RDM_PORT_ONE, RDM_ROOT_DEVICE);
	UID::copyFromUID(target, _rdmPacket, 3);
	setupRDMMessageDataBlock(_rdmPacket, RDM_DISCOVERY_COMMAND, cmd, 0x00);

	_rdm_read_handled = 1;
	sendRawRDMPacket(RDM_PKT_BASE_TOTAL_LEN);
	delay(3);
	
	if ( _current_slot >= (RDM_PKT_BASE_TOTAL_LEN+2) ) {				//expected pdl 2 or 8
		if ( validateRDMPacket(_receivedData) ) {
			if ( _receivedData[RDM_IDX_RESPONSE_TYPE] == RDM_RESPONSE_TYPE_ACK ) {
				if ( _receivedData[RDM_IDX_CMD_CLASS] == RDM_DISC_COMMAND_RESPONSE ) {
					if ( THIS_DEVICE_ID == UID(&_receivedData[RDM_IDX_DESTINATION_UID]) ) {
						rv = 1;
					}
				}
			}
		}
		
		_rdm_read_handled = 0;
		resetFrame();
	} else {
		_rdm_read_handled = 0;
	}
	
	restoreTaskSendDMX();
	return rv;
}

uint8_t LXTeensyDMX::sendRDMControllerPacket( void ) {
	uint8_t rv = 0;
	_rdm_read_handled = 1;
	sendRawRDMPacket(_rdmPacket[2]+2);
	delay(3);
	
	if ( _current_slot > 0 ) {
		if ( validateRDMPacket(_receivedData) ) {
			uint8_t plen = _receivedData[2] + 2;
			for(int rv=0; rv<plen; rv++) {
				_rdmData[rv] = _receivedData[rv];
			}
			rv = 1;
		}
		_rdm_read_handled = 0;
		resetFrame();
	} else {
		_rdm_read_handled = 0;
	}
	
	restoreTaskSendDMX();
	return rv;
}

uint8_t LXTeensyDMX::sendRDMControllerPacket( uint8_t* bytes, uint8_t len ) {
	for (uint8_t j=0; j<len; j++) {
		_rdmPacket[j] = bytes[j];
	}
	return sendRDMControllerPacket();
}

uint8_t LXTeensyDMX::sendRDMGetCommand(UID target, uint16_t pid, uint8_t* info, uint8_t len) {
	uint8_t rv = 0;
	
	//Build RDM packet
	// total packet length 0 parameter is 24 (+cksum =26 for sendRawRDMPacket) 
	setupRDMControllerPacket(_rdmPacket, RDM_PKT_BASE_MSG_LEN, RDM_PORT_ONE, RDM_ROOT_DEVICE);
	UID::copyFromUID(target, _rdmPacket, 3);
	setupRDMMessageDataBlock(_rdmPacket, RDM_GET_COMMAND, pid, 0x00);
	
	if ( sendRDMControllerPacket() ) {
		if ( _rdmData[RDM_IDX_RESPONSE_TYPE] == RDM_RESPONSE_TYPE_ACK ) {
			if ( _rdmData[RDM_IDX_CMD_CLASS] == RDM_GET_COMMAND_RESPONSE ) {
				if ( THIS_DEVICE_ID == UID(&_rdmData[RDM_IDX_DESTINATION_UID]) ) {
					rv = 1;
					for(int j=0; j<len; j++) {
						info[j] = _rdmData[24+j];
					}
				}
			}
		}
	}
	
	return rv;
}

uint8_t LXTeensyDMX::sendRDMSetCommand(UID target, uint16_t pid, uint8_t* info, uint8_t len) {
	uint8_t rv = 0;
	
	//Build RDM packet
	// total packet length 1 byte parameter is 25 (+cksum =27 for sendRawRDMPacket) 
	setupRDMControllerPacket(_rdmPacket, RDM_PKT_BASE_MSG_LEN+len, RDM_PORT_ONE, RDM_ROOT_DEVICE);
	UID::copyFromUID(target, _rdmPacket, 3);
	setupRDMMessageDataBlock(_rdmPacket, RDM_SET_COMMAND, pid, len);
	for(int j=0; j<len; j++) {
		_rdmPacket[24+j] = info[j];
	}
	
	if ( sendRDMControllerPacket() ) {
		if ( _rdmData[RDM_IDX_RESPONSE_TYPE] == RDM_RESPONSE_TYPE_ACK ) {
			if ( _rdmData[RDM_IDX_CMD_CLASS] == RDM_SET_COMMAND_RESPONSE ) {
				if ( THIS_DEVICE_ID == UID(&_rdmData[RDM_IDX_DESTINATION_UID]) ) {
					rv = 1;
				}
			}
		}
	}
	
	return rv;
}

// ***** protected member function ******

void LXTeensyDMX::uartISR( void ) {
  uint8_t incoming_byte = _uart_hardware->uart_reg_ptr->D;					// read buffer to clear interrupt flag (should go here or below?)
  
  if ( _uart_hardware->uart_reg_ptr->S1 & UART_S1_FE ) {					// framing error
		breakReceived(); 
		return;										// do not call byteReceived if framing error
  }	
  
  /* experimental break detect for Teensy 3.6
  if ( _uart_hardware->uart_reg_ptr->S2 & UART_S2_LBKDIF ) {					// break detect
        _uart_hardware->uart_reg_ptr->S2 |= UART_S2_LBKDIF;                     // clear breakflag
		breakReceived(); 
		return;										// do not call byteReceived if break detected
  }
  */

  if ( _uart_hardware->uart_reg_ptr->S1 & UART_S1_RDRF ) {					// receive register full
		byteReceived(incoming_byte);
  }			// receive register full
  

  
	

	
// ********************** send portion of isr

  uint8_t c = _uart_hardware->uart_reg_ptr->C2;

  if ((c & UART_C2_TIE) && (_uart_hardware->uart_reg_ptr->S1 & UART_S1_TDRE)) {   // transmit empty
	transmitEmpty();
	return;
  }   		// transmit empty

  if ((c & UART_C2_TCIE) && (_uart_hardware->uart_reg_ptr->S1 & UART_S1_TC)) {    // transmit complete
	transmitComplete();
	return;
  }				// transmit complete
	  
	

}