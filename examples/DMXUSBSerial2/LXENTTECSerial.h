/* LXENTTECSerial.h
   Copyright 2016 by Claude Heintz Design
   see http://www.claudeheintzdesign.com/lx/opensource.html
*/

#ifndef LXENTTECSerial_H
#define LXENTTECSerial_H

#include <Arduino.h>
#include <inttypes.h>

#define DMX_MIN 25
#define DMX_MAX 513

#define ENTTEC_LABEL_NONE         0
#define ENTTEC_LABEL_GET_INFO     3
#define ENTTEC_LABEL_RECEIVED_DMX 5
#define ENTTEC_LABEL_SEND_DMX     6
#define ENTTEC_LABEL_SEND_DMX_ULTRA    100
#define ENTTEC_LABEL_SEND_DMX_ULTRA2   101
#define ENTTEC_LABEL_RECEIVE_DMX  8
#define ENTTEC_LABEL_GET_SERIAL   10

class LXENTTECSerial {

  public:
	LXENTTECSerial  ( void );
   ~LXENTTECSerial ( void );

/**
 * returns the number of DMX addresses
 * returns 0 if no packet has been read or the last attempt returned an error
 * a partial read of a corrupted packet will invalidate the buffer data
*/    
   int  numberOfSlots    ( void );

/**
 * sets the number of DMX addresses, up to 512
*/
   void setNumberOfSlots ( int n );
   
/**
 * returns the value stored in a slot
 * slots are numbered 1 to 512
*/
   uint8_t  getSlot      ( int slot );
   
/**
 * sets the value of a slot
 * slots are numbered 1 to 512
*/
   void     setSlot      ( int slot, uint8_t value );
   
/**
 * returns the start code, standard DMX start code is zero
*/
   uint8_t  startCode    ( void );
   
/**
 * sets the start code
*/
   void  setStartCode    ( uint8_t value );
   
/**
 * pointer to dmx data buffer for direct access
*/
   uint8_t* dmxData      ( void );


/**
 * returns the number of DMX addresses stored in the second buffer
 * returns 0 if no packet has been read or the last attempt returned an error
 * a partial read of a corrupted packet will invalidate the buffer data
*/  
   int  numberOfSlots2    ( void );
   
/**
 * sets the number of DMX addresses, up to 512, in the 2nd buffer
*/
   void setNumberOfSlots2 ( int n );
   
/**
 * returns the value of a slot stored in the 2nd buffer
 * slots are numbered 1 to 512
*/
   uint8_t  getSlot2      ( int slot );
   
/**
 * sets the value of a slot stored in the 2nd buffer
 * slots are numbered 1 to 512
*/
   void     setSlot2      ( int slot, uint8_t value );
   
/**
 * returns the start code for the 2nd universe, standard DMX start code is zero
*/
   uint8_t  startCode2    ( void );

/**
 * sets the start code for the 2nd universe
*/
   void  setStartCode2    ( uint8_t value );
   
/**
 * pointer to 2nd data buffer for direct access
*/
   uint8_t* dmxData2      ( void );
   
/**
 * readPacket() reads a packet into the buffer using built-in Serial
 * blocks until packet is read or fails
 * 
 * Packets with labels ENTTEC_LABEL_GET_INFO or ENTTEC_LABEL_GET_SERIAL
 * are handled by writing a reply directly to the serial conection
 * Packets with ENTTEC_LABEL_SEND_DMX labels have their dmx data read into the buffer
 * Packets with other labels have their data discarded, however their labels are retuned
 * 
 * Must call Serial.begin(<baud>) before calling this method
*/
   uint8_t readPacket     ( void );
   
/**
 * writes a DMX Received packet to Serial
 * sends the DMX data from the _dmx_data buffer 
 * Important:  must call Serial.begin(<baud>) before calling this method
*/
   void    writeDMXPacket ( void );
   
/**
 * writes a DMX Received packet to Serial
 * sends the DMX data from an external buffer
 * Must call Serial.begin(<baud>) before calling this method
*/
   void    writeDMXPacket ( uint8_t *buffer, int length );
   
/**
 * writes widget info packet
 * writes zeros for user data of length
 * Must call Serial.begin(<baud>) before calling this method
*/
   void    writeInfo      ( uint16_t length );
   
/**
 * writes serial number packet, serial number is all ones
*/
   void    writeSerialNumber ( uint32_t sn );
    
  private:
/**
 *	storage for dmx data
*/
  	uint8_t  _dmx_data[DMX_MAX];
/**
 *	storage for 2nd universe ofdmx data
*/
  	uint8_t  _dmx_data2[DMX_MAX];
/**
 *	number of dmx slots
*/
  	int      _dmx_slots;
/**
 *	number of dmx slots in 2nd universe
*/
  	int      _dmx_slots2;

/**
 *	Utility function for reading bytes from Serial
 * Blocks until byte is available on Serial connection!
*/
  	uint8_t  getNextByte( void );
};

#endif // ifndef LXENTTECSerial_H