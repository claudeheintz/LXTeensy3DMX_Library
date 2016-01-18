/**************************************************************************/
/*!
    @file     DMXFadeTest.ino
    @author   Claude Heintz
    @license  BSD (see LXTeensy3DMX LICENSE)
    @copyright 2016 by Claude Heintz

    Simple Fade test of Teensy3 DMX Driver
    @section  HISTORY

    v1.00 - First release6
*/
/**************************************************************************/
#include <LXTeensy3DMX.h>


uint8_t level = 0;

void setup() {
  Teensy3DMX.startOutput();
}

/************************************************************************

  The main loop fades the levels of addresses 7 and 8 to full
  
*************************************************************************/

void loop() {
 Teensy3DMX.setSlot(1,level);
 Teensy3DMX.setSlot(512,level);
 delay(50);
 level++;
}
