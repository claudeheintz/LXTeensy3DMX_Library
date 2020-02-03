/**************************************************************************/
/*!
    @file     DMXFadeTest.ino
    @author   Claude Heintz
    @license  BSD (see LXTeensy3DMX LICENSE)
    @copyright 2016 by Claude Heintz

    Simple Fade test of Teensy3 DMX Driver
    
    @section  HISTORY
    v1.00 - First release
    v1.1  - Added additional universes
*/
/**************************************************************************/
#include <LXTeensy3DMX.h>
#include <LXTeensy3DMX1.h>
#include <LXTeensy3DMX2.h>


uint8_t level = 0;

void setup() {
  Teensy3DMX.startOutput();   // uses pins 0 and 1		Universe 1
  Teensy3DMX1.startOutput();  // uses pins 9 and 10	Universe 2
  Teensy3DMX2.startOutput();  // uses pins 7 and 8		Universe 3
}

/************************************************************************

  The main loop fades the levels of addresses 1 and 512
  
*************************************************************************/

void loop() {
 Teensy3DMX.setSlot(1,level);
 Teensy3DMX.setSlot(512,level);
 
 Teensy3DMX1.setSlot(1,level);
 Teensy3DMX1.setSlot(512,level);
 
 Teensy3DMX2.setSlot(1,level);
 Teensy3DMX2.setSlot(512,level);
 
 delay(50);
 level++;
}
