/**************************************************************************/
/*!
    @file     DMXUSBSerial.ino
    @author   Claude Heintz
    @license  see LXTeensy3DMX.h or http://www.claudeheintzdesign.com/lx/opensource.html
    @copyright 2016 by Claude Heintz

    This sketch allows a Teensy 3.2 board to emulate some functions
    of a DMXKing Ultra DMX interface including output of two universes
    of DMX data.
    
    @section  HISTORY

    v1.00 - First release
*/
/**************************************************************************/

#include <LXTeensy3DMX.h>
#include <LXTeensy3DMX1.h>
#include "LXENTTECSerial.h"

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
LXENTTECSerial eSerial = LXENTTECSerial();

// ***************** setup() runs once  ****************

void setup() {
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GRN_LED_PIN, OUTPUT);           
  pinMode(BLU_LED_PIN, OUTPUT);
  Teensy3DMX.setDirectionPin(RXTX_PIN); 
  Serial.begin(57600);//115200 doesn't matter because teensy changes to USB speed
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
  if (( label == ENTTEC_LABEL_SEND_DMX ) || ( label == ENTTEC_LABEL_SEND_DMX_ULTRA)) {
    int s = eSerial.numberOfSlots() + 1;		//add start code
    for(int i=0; i<s; i++) {
      Teensy3DMX.setSlot(i,eSerial.getSlot(i));
    }
    Teensy3DMX.startOutput();  //ignored if already started
    setLED(GRN_LED_BIT);
  } else if ( label == ENTTEC_LABEL_SEND_DMX_ULTRA2 ) {
    int s = eSerial.numberOfSlots2() + 1;		//add start code
    for(int i=0; i<s; i++) {
      Teensy3DMX1.setSlot(i,eSerial.getSlot2(i));
    }
    Teensy3DMX1.startOutput();  //ignored if already started
    setLED(BLU_LED_BIT);
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
  if ( Serial.available()) {  //writing anything to the USB switched to output mode
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


