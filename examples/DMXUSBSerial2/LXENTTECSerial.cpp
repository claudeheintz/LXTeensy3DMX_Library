/* LXENTTECSerial.cpp
   Copyright 2016 by Claude Heintz Design
   see http://www.claudeheintzdesign.com/lx/opensource.html
   
	The ENTTEC DMX USB Pro Widget API specifies the interface requirements for
	PC based application programs to use the DMX USB Pro Widget
	to send or receive DMX512 packets.  see www.enttec.com

	LXENTTECSerial implements exchanging DMX USB Pro Widget API packets with Arduino
	over the built-in Serial connection.  LXENTTECSerial does not implement the full
	ENTTEC DMX USB Pro Widget API specification.
*/

#include "LXENTTECSerial.h"

LXENTTECSerial::LXENTTECSerial ( void )
{
	//zero buffer including _dmx_data[0] which is start code
    for (int n=0; n<DMX_MAX; n++) {
    	_dmx_data[n] = 0;
    	_dmx_data2[n] = 0;
    }
    
    _dmx_slots = 0;
}

LXENTTECSerial::~LXENTTECSerial ( void )
{
    //no need for specific destructor
}

int  LXENTTECSerial::numberOfSlots ( void ) {
	return _dmx_slots;
}

void LXENTTECSerial::setNumberOfSlots ( int n ) {
	_dmx_slots = n;
}

uint8_t LXENTTECSerial::getSlot ( int slot ) {
	return _dmx_data[slot];
}

void LXENTTECSerial::setSlot ( int slot, uint8_t value ) {
	_dmx_data[slot] = value;
}

uint8_t LXENTTECSerial::startCode ( void ) {
	return _dmx_data[0];
}

void LXENTTECSerial::setStartCode ( uint8_t value ) {
	_dmx_data[0] = value;
}

uint8_t* LXENTTECSerial::dmxData( void ) {
	return &_dmx_data[0];
}

int  LXENTTECSerial::numberOfSlots2 ( void ) {
	return _dmx_slots2;
}

void LXENTTECSerial::setNumberOfSlots2 ( int n ) {
	_dmx_slots2 = n;
}

uint8_t LXENTTECSerial::getSlot2 ( int slot ) {
	return _dmx_data2[slot];
}

void LXENTTECSerial::setSlot2 ( int slot, uint8_t value ) {
	_dmx_data2[slot] = value;
}

uint8_t LXENTTECSerial::startCode2 ( void ) {
	return _dmx_data2[0];
}

void LXENTTECSerial::setStartCode2 ( uint8_t value ) {
	_dmx_data2[0] = value;
}

uint8_t* LXENTTECSerial::dmxData2( void ) {
	return &_dmx_data2[0];
}

uint8_t LXENTTECSerial::readPacket( void ) {
   uint8_t b;
   uint8_t label = ENTTEC_LABEL_NONE;
   int data_length;
   uint8_t userlsb = 0;
   uint8_t usermsb = 0;
   
   b = getNextByte();
   if ( b == 0x7E ) {               // byte must be packet start delimiter 0x7E
      label = getNextByte();        // 6 = Send DMX
      data_length = getNextByte();  // LSB
      b = getNextByte();            // MSB
	   data_length += (b << 8);
	   switch ( label ) {
	      case ENTTEC_LABEL_SEND_DMX:
	      case ENTTEC_LABEL_SEND_DMX_ULTRA:
	         for(int n=0; n<data_length; n++) {        // read data_length bytes
	            b = getNextByte();
	            if ( n < DMX_MAX ) {
			         _dmx_data[n] = b;                   // store bytes in buffer 
			      }
	         }
	         break;
	      case ENTTEC_LABEL_SEND_DMX_ULTRA2:
	         for(int n=0; n<data_length; n++) {        // read data_length bytes
	            b = getNextByte();
	            if ( n < DMX_MAX ) {
			         _dmx_data2[n] = b;                   // store bytes in buffer 
			      }
	         }
	         break;
	      case ENTTEC_LABEL_GET_INFO:
	         if ( data_length == 2 ) {					   // user data length is 2 bytes
	            userlsb = getNextByte();
	            usermsb = getNextByte();
	            break;
	         }
	      default:
	         for(int n=0; n<data_length; n++) {        // read data_length bytes
	            b = getNextByte();                     // discard
	         }
	   } //switch
	   

	   b = getNextByte();
	   if ( b == 0xE7 ) {						// good packet
			switch ( label ) {
				case ENTTEC_LABEL_SEND_DMX:
				case ENTTEC_LABEL_SEND_DMX_ULTRA:
					if ( data_length < DMX_MIN ) {
						data_length = DMX_MIN;
					} else if ( data_length > DMX_MAX ) {
						data_length = DMX_MAX;
					}
				   _dmx_slots = data_length-1; //data length includes start code
					break;
				case ENTTEC_LABEL_SEND_DMX_ULTRA2:
					if ( data_length < DMX_MIN ) {
						data_length = DMX_MIN;
					} else if ( data_length > DMX_MAX ) {
						data_length = DMX_MAX;
					}
				   _dmx_slots2 = data_length-1; //data length includes start code
					break;
				case ENTTEC_LABEL_GET_INFO:
				   this->writeInfo(userlsb+(usermsb<<8));
				   break;
				case ENTTEC_LABEL_GET_SERIAL:
				   this->writeSerialNumber(0xffffffff);
				   break;
			}
	   } else {							          // bad packet
	   	switch ( label ) {			
	      	case ENTTEC_LABEL_SEND_DMX:
				case ENTTEC_LABEL_SEND_DMX_ULTRA:
	         	_dmx_slots = 0;				 // buffer is not valid anymore
	         	break;
	      	case ENTTEC_LABEL_SEND_DMX_ULTRA2:
	          _dmx_slots2 = 0;					 // buffer is not valid anymore
	      }
	      label = ENTTEC_LABEL_NONE;
	   }
   }      // good start delimiter

	return label;
}

void LXENTTECSerial::writeDMXPacket( void ) {
   this->writeDMXPacket(_dmx_data, _dmx_slots+1);	//includes start code
}

void LXENTTECSerial::writeDMXPacket( uint8_t *buffer, int length ) {
	int total_length = length + 1;
	uint8_t header[5];
	header[0] = 0x7E;
	header[1] = ENTTEC_LABEL_RECEIVED_DMX;
	header[2] = total_length & 0xff;
	header[3] = total_length >> 8;
	header[4] = 0;						//status byte unused at present
	Serial.write(header,5);
	Serial.write(buffer, length);
	header[0] = 0xE7;
	Serial.write(header,1);
}

void LXENTTECSerial::writeInfo( uint16_t length ) {
	uint8_t header[9];
	header[0] = 0x7E;
	header[1] = ENTTEC_LABEL_GET_INFO;
	header[2] = (length+5) & 0xff;
	header[3] = (length+5) >> 8;
	header[4] = 44;   // protocol version lsb
	header[5] = 1;    // msb
	header[6] = 9;    // DMX break x10.67 usecs (~99usec on Teensy2++)
	header[7] = 1;    // MAB x10.67 usecs       (~13usec on Teensy2++)
	header[8] = 0;    // output speed packets/sec 0=max
	Serial.write(header,9);
	header[0] = 0x00;
	for(uint16_t n=0; n<length; n++) {
	   Serial.write(header, 1);				// does not save user data, write as zeros
	}
	header[0] = 0xE7;
	Serial.write(header,1);
}

void LXENTTECSerial::writeSerialNumber( uint32_t sn ) {
	uint8_t header[9];
	header[0] = 0x7E;
	header[1] = ENTTEC_LABEL_GET_SERIAL;
	header[2] = 4;
	header[3] = 0;
	header[4] = sn & 0xff;
	header[5] = ( sn >> 8 ) & 0xff;
	header[6] = ( sn >> 16 ) & 0xff;
	header[7] = ( sn >> 24 ) & 0xff;
	header[8] = 0xE7;
	Serial.write(header,9);
} 

uint8_t LXENTTECSerial::getNextByte( void ) {
  while ( true ) {
    if ( Serial.available()) {
      return Serial.read();
    }
  }
}
