/**************************************************************************/
/*!
    @file     DMXInputTest.ino
    @author   Claude Heintz
    @license  BSD (see LXTeensy3DMX LICENSE)
    @copyright 2016 by Claude Heintz

    Control brightness of LED on PWM_PIN with DMX address 1
    @section  HISTORY

    v1.00 - First release
*/
/**************************************************************************/
#include <LXTeensy3DMX.h>

#define PWM_PIN 6
int got_dmx = 0;

void setup() {
  pinMode(PWM_PIN, OUTPUT);
  Teensy3DMX.setDataReceivedCallback(&gotDMXCallback);
  Teensy3DMX.startInput();
  Serial.begin(115200);
}


// ***************** input callback function *************

void gotDMXCallback(int slots) {
  got_dmx = slots;
}

/************************************************************************

  The main loop checks to see if dmx input is available (got_dmx>0)
  And then reads the level of dimmer 1 to set PWM level of LED
  
*************************************************************************/

void loop() {
  if ( got_dmx ) {
    analogWrite(PWM_PIN,Teensy3DMX.getSlot(1));
    Serial.println("---");
    Serial.println(Teensy3DMX.getSlot(0));
    Serial.println(Teensy3DMX.getSlot(1));
    Serial.println(got_dmx);
    Serial.println("___");
  }
}
