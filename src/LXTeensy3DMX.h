/* LXTeensy3DMX.h
   Copyright 2016 by Claude Heintz Design
   All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of LXTeensyDMX nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Portions of this software were derived from Teensyduino Core Library
Copyright (c) 2013 PJRC.COM, LLC. http://www.pjrc.com/teensy/
See bottom of this file for license.
-----------------------------------------------------------------------------------

   The LXTeensyDMX library supports output and input of DMX using the UART(s)
   of a Teensy 3.2's microcontroller.
   
   This is the circuit for a simple unisolated DMX Shield
   that could be used with LXTeensyDMX.  It uses a line driver IC
   to convert the output from the Teensy to DMX:

 Teensy Pin
 |                         SN 75176 A or MAX 481CPA
 V                            _______________
       |                      | 1      Vcc 8 |------(+5v)
RX (0) |----------------------|              |                 DMX Output
       |                 +----| 2        B 7 |---------------- Pin 2
       |                 |    |              |
   (d) |----------------------| 3 DE     A 6 |---------------- Pin 3
       |                      |              |
TX (1) |----------------------| 4 DI   Gnd 5 |---+------------ Pin 1
       |                                         |
       |                                       (GND)

	Data Enable (DE) and Inverted Read Enable (!RE) can be wired to +5v for output
	or Gnd for input if direction switching is not needed.
*/

#ifndef LXTeensy3DMX_H
#define LXTeensy3DMX_H

#include "kinetis.h"
#include "core_pins.h"

#define DMX_MIN_SLOTS 24
#define DMX_MAX_SLOTS 512

#define DIRECTION_PIN_NOT_USED 255

typedef void (*LXRecvCallback)(int);

/*!   
@class LXTeensyDMX
@abstract
   LXTeensyDMX is a driver for sending or receiving DMX using a Teensy 3.1/3.2's
   UART0 RX pin 0, TX pin 1
   
   LXTeensyDMX output mode continuously sends DMX once its interrupts have been enabled using startOutput().
   Use setSlot() to set the level value for a particular DMX dimmer/address/channel.
   
   LXTeensyDMX input mode continuously receives DMX once its interrupts have been enabled using startInput()
   Use getSlot() to read the level value for a particular DMX dimmer/address/channel.
   
   LXTeensyDMX is used with a single instance called Teensy3DMX	.
*/

class LXTeensyDMX {

  public:
  
	LXTeensyDMX  ( void );
   ~LXTeensyDMX ( void );
    
   /*!
    * @brief starts interrupt that continuously sends DMX output
    * @discussion Sets up baud rate, bits and parity, 
    *             sets globals accessed in ISR, 
    *             enables transmission (TE) and tx interrupts (TIE/TCIE).
   */
   void startOutput( void );
   
   /*!
    * @brief starts interrupt that continuously reads DMX data
    * @discussion sets up baud rate, bits and parity, 
    *             sets globals accessed in ISR, 
    *             enables receive (RE) and rx interrupt (RIE)
   */
   void startInput( void );
   
   /*!
    * @brief disables tx, rx and interrupts.
   */
	void stop( void );
	
	/*!
	 * @brief optional utility sets the pin used to control driver chip's
	 *        DE (data enable) line, HIGH for output, LOW for input.     
    * @param pin to be automatically set for input/output direction
    */
   void setDirectionPin( uint8_t pin );
	
	/*!
	 * @brief Sets the number of slots (aka addresses or channels) sent per DMX frame.
	 * @discussion defaults to 512 or DMX_MAX_SLOTS and should be no less DMX_MIN_SLOTS slots.  
	 *             The DMX standard specifies min break to break time no less than 1024 usecs.  
	 *             At 44 usecs per slot ~= 24
	 * @param slot the highest slot number (~24 to 512)
	*/
	void setMaxSlots (int slot);
	
	/*!
    * @brief reads the value of a slot/address/channel
    * @discussion NOTE: Data is not double buffered.  
    *                   So a complete single frame is not guaranteed.  
    *                   The ISR continuously reads the next frame into the buffer
    * @return level (0-255)
   */
   uint8_t getSlot (int slot);
   
	/*!
	 * @brief Sets the output value of a slot
	 * @param slot number of the slot/address/channel (1-512)
	 * @param value level (0-255)
	*/
   void setSlot (int slot, uint8_t value);
   
   /*!
    * @brief provides direct access to data array
    * @return pointer to dmx array
   */
   uint8_t* dmxData(void);
   
      /*!
    * @brief Function called when DMX frame has been read
    * @discussion Sets a pointer to a function that is called
    *             on the break after a DMX frame has been received.  
    *             Whatever happens in this function should be quick!  
    *             Best used to set a flag that is polled outside of ISR for available data.
   */
   void setDataReceivedCallback(LXRecvCallback callback);
    
  protected:
   /*!
    * @brief Indicates mode ISR_OUTPUT_ENABLED or ISR_INPUT_ENABLED or ISR_DISABLED
   */
  	uint8_t  _interrupt_status;
  	
  	/*!
   * @brief pin used to control direction of output driver chip
   */
  	uint8_t _direction_pin;
  	
  	/*!
    * @brief Array of dmx data including start code
   */
	uint8_t  _dmxData[DMX_MAX_SLOTS+1];
};

extern LXTeensyDMX Teensy3DMX;


/**************************************************************************************/

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

/**************************************************************************************
 * defines and functions adapted from from Teensyduino Core Library
 * (see license below)
 * refer to .../cores/teensy3/kinetis.h, HardwareSerial.h, serial1.c, serial2.c, serial3.c
 * datasheet pg.131 / pg 1201
 * UART0 and UART1 are clocked by F_CPU, UART2 is clocked by F_BUS
 * UART0 has 8 byte fifo --unused in this implementation--, UART1 and UART2 have 1 byte buffer
*/

// Teensy 3.1/3.2
#if defined(KINETISK)
#define BAUD2DIV(baud)  (((F_CPU * 2) + ((baud) >> 1)) / (baud))
#define BAUD2DIV2(baud) (((F_CPU * 2) + ((baud) >> 1)) / (baud))
#define BAUD2DIV3(baud) (((F_BUS * 2) + ((baud) >> 1)) / (baud))

// Teensy LC
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

#define DMX_UART_IRQ_PRIORITY  64  // 0 = highest priority, 255 = lowest

#define SERIAL_8N1 0x00
#define SERIAL_8N2 0x04

#define C2_TX_ENABLE   UART_C2_TE
#define C2_RX_ENABLE   UART_C2_RE | UART_C2_RIE

// functions
void serial_one_set_baud(uint32_t bit_rate);
void serial_one_begin(uint32_t bit_rate);
void serial_one_format(uint32_t format);
void serial_one_end(void);
void lx_uart0_status_isr(void);

/* License for Teensyduino Core Library
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
 
 #endif // ifndef LXTeensy3DMX_H
