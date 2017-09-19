/**************************************************************************/
/*!
    @file     TOD.h
    @author   Claude Heintz
    @license  BSD (see LXESP32DMX.h)
    @copyright 2017 by Claude Heintz

    RDM support for DMX Driver for ESP32
    
    Implements RDM Table of Devices as array of 6 byte (48bit) UIDs
    capable of storing 200 UIDs in static array

    @section  HISTORY

    v1.0 - First release
*/
/**************************************************************************/

#ifndef RDMTOD_h
#define RDMTOD_h

#include <stdint.h>
#include <WString.h>
#include <rdm/UID.h>

// A class to make it easier to handle RDM UIDs
// based on Arduino IPAddress class


#define STORAGE_SIZE 1200

class TOD {
private:
    uint8_t   storage[STORAGE_SIZE];
    uint16_t  next;

public:
    TOD( void );

	uint8_t addUID(UID uid);
	uint8_t add(UID uid);
	void    removeUIDAt(int index);
	
	uint8_t getUIDAt(int index, UID* uid);
	int     getNextUID(int index, UID* uid);
	
	void push (UID uid);
	uint8_t pop (UID* uid);
	
	uint8_t contains( UID uid );
	uint8_t count( void );
	void    reset( void );
	
	uint8_t* rawBytes( void );
	
	void    printTOD( void );
};


#endif	//RDMTOD
