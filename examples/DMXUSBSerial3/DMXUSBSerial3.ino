/**************************************************************************/
/*!
    @file     DMXUSBSerial3.ino
    @author   Claude Heintz
    @license  BSD (see LXTeensy3DMX LICENSE)
    @copyright 2016 by Claude Heintz

    This sketch allows a Teensy 3.2 board to emulate parts of an ENTTEC DMX USB Pro's functions.
    Instead of using Teensy 3.2's USB Serial, this version uses TX3/RX3 and requires an external
    Serial to USB connection.  The circuit shown below uses an FTDI Friend USB/Serial Adapter.
    @section  HISTORY

    v1.00 - First release


Teensy 3.2           SN 75176 A or MAX 481CPA
pin                       _______________
 0 |--------------------- | 1      Vcc 8 |------ +5v ----  Vin/Vcc 
   |                      |              |
   |                 +----| 2        B 7 |---------------- Pin 2
   |                 |    |              |
 4 |----------------------| 3 DE     A 6 |---------------- Pin 3
   |                      |              |
 1 |----------------------| 4 DI   Gnd 5 |---+------------ Pin 1
   |                      _______________    |             DMX Output
   |                                         |
 5 |-------[ 330 ohm ]--[ G LED ]------------|
 6 |-------[ 330 ohm ]--[ B LED ]------------|
13 |-------[ 330 ohm ]--[ R LED ]-(built-in)-| 
   |                                         |               FTDI Adapter
Gnd|--------------------------------------- GND -------------| GND
   |                                                        -| CTS
Vin|---------------------------------------------------------| Vcc
 7 |---------------------------------------------------------| Tx
 8 |---------------------------------------------------------| Rx
	|                                                        -| DTR

Pins 7 & 8 are Serial3 Rx/Tx
Pins 0 & 1 are UART0 Rx/Tx which is used by LXTeensy3DMX.
The built-in LED is on pin 13 and is used to indicate error.
Optionally pin 13 can also be connected to red lead of a tri color LED.
The output/input indicator LEDs, green and blue are also optional.
    
*/
/**************************************************************************/

#include <LXTeensy3DMX.h>
#include "LXENTTECSerial3.h"

// ********************** defines **********************

// Pin 13 has an LED connected on most Arduino boards.
// Pin 13 has the LED on Teensy 3.0
#define RED_LED_PIN 13
#define GRN_LED_PIN 5
#define BLU_LED_PIN 6

#define RED_LED_BIT 1
#define GRN_LED_BIT 2
#define BLU_LED_BIT 4

#define RXTX_PIN 4

#define MODE_OUTPUT_DMX 0
#define MODE_INPUT_DMX 1

#define PACKET_LABEL_DMX 6
#define PACKET_LABEL_RECEIVE 8
#define PACKET_LABEL_GET_INFO 3
#define PACKET_LABEL_GET_SERIAL 10

// ********************** globals **********************

uint8_t mode = MODE_OUTPUT_DMX;
int got_dmx = 0;
uint8_t buffer[DMX_MAX_SLOTS+1];				// add 1 for start code
LXENTTECSerial eSerial = LXENTTECSerial();

// ***************** setup() runs once  ****************

void setup() {
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GRN_LED_PIN, OUTPUT);           
  pinMode(BLU_LED_PIN, OUTPUT);
  Teensy3DMX.setDirectionPin(RXTX_PIN); 
  Serial3.begin(250000);  // 250000 for Windows, 9600 for Mac
}

// ***************** utility functions  ****************

/*  setLED(uint8_t flags)
 *  sets color of RGB LED
 */

void setLED(uint8_t flags) {
  digitalWrite(RED_LED_PIN, 1 & flags);
  digitalWrite(GRN_LED_PIN, 1 & (flags>>1));
  digitalWrite(BLU_LED_PIN, 1 & (flags>>2));
}


// ***************** input/output functions *************

void gotDMXCallback(int slots) {
  got_dmx = slots;
}

void doOutputMode() {
  uint8_t label = eSerial.readPacket();
  if ( label == ENTTEC_LABEL_SEND_DMX ) {
    int s = eSerial.numberOfSlots() + 1;		//add start code
    for(int i=0; i<s; i++) {
      Teensy3DMX.setSlot(i,eSerial.getSlot(i));
    }
    Teensy3DMX.startOutput();  //ignored if already started
    setLED(GRN_LED_BIT);
  } else if ( label == ENTTEC_LABEL_RECEIVE_DMX ) {
	Teensy3DMX.setDataReceivedCallback(&gotDMXCallback);
	mode = MODE_INPUT_DMX;
	Teensy3DMX.startInput();
	setLED(BLU_LED_BIT);
  } else if ( label == ENTTEC_LABEL_NONE ) {
    setLED(RED_LED_BIT);
  }
}

void doInputMode() {
  if ( Serial3.available()) {  //writing anything to the USB switched to output mode
	Teensy3DMX.setDataReceivedCallback(0);
	mode = MODE_OUTPUT_DMX;
	setLED(0);
    return;
  }
  if ( got_dmx ) {
    int msg_size = got_dmx;
	 eSerial.writeDMXPacket(Teensy3DMX.dmxData(), msg_size);
    got_dmx = 0;
  }
  delay(50);         // wait to allow serial to keep up
}

// ***************** main loop  ****************

void loop() {
  if ( mode == MODE_OUTPUT_DMX ) {
    doOutputMode();
  } else {
    doInputMode();
  }
}        //main loop


