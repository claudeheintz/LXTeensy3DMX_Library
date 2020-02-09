/**************************************************************************/
/*!
    @file     rdmTest.ino
    @author   Claude Heintz
    @license  BSD (see LXTeensy3DMX LICENSE)
    @copyright 2017 by Claude Heintz

        Test of LXTeensy3DMX RDM functions
        Changes output level of some DMX Addresses while building RDM
        table of devices.  Turns identify on and off for found RDM devices.

    @section  HISTORY

    v1.00 - First release
*/
/**************************************************************************/

#include <LXTeensy3DMX.h>
#include <rdm/rdm_utility.h>
#include <rdm/UID.h>
#include <rdm/TOD.h>

/* Teensy 4's pins are not 5v tolerant!
 * One method of converting signal from a 5v powered MAX485 to 3.3v
 * is to use a pullup resistor to 3.3v and a transistor
 * to ground on the rx pin.  This inverts the signal so HIGH
 * from the 485 turns on the transistor, pulling the rx pin LOW.
 * Conversely, LOW from the 485 turns the transistor off and
 * the pullup resistor to 3.3v makes the rx pin HIGH.
 * An opto-isolator can be used similarly.
 */
uint8_t rx_polarity = RX_SIGNAL_NORMAL; // or RX_SIGNAL_INVERTED

uint8_t testLevel = 0;
uint8_t loopDivider = 0;
uint8_t identifyFlag = 1;
uint8_t tableChangedFlag = 0;

TOD tableOfDevices;
TOD discoveryTree;

UID lower(0,0,0,0,0,0);
UID upper(0,0,0,0,0,0);
UID mid(0,0,0,0,0,0);
UID found(0,0,0,0,0,0);

#define DIRECTION_PIN 3
#define DISC_STATE_SEARCH 0
#define DISC_STATE_TBL_CK 1
uint8_t discovery_state = DISC_STATE_TBL_CK;
uint8_t discovery_tbl_ck_index = 0;


//***************** discovery functions

void checkDeviceFound(UID found) {
  if ( testMute(found) ) {
    tableOfDevices.add(found);
    tableChangedFlag = 1;
  }
}

uint8_t testMute(UID u) {
   // try three times to get response when sending a mute message
   if ( Teensy3DMX.sendRDMDiscoveryMute(u, RDM_DISC_MUTE) ) {
     return 1;
   }
   if ( Teensy3DMX.sendRDMDiscoveryMute(u, RDM_DISC_MUTE) ) {
     return 1;
   }
   if ( Teensy3DMX.sendRDMDiscoveryMute(u, RDM_DISC_MUTE) ) {
     return 1;
   }
   return 0;
}

uint8_t checkTable(uint8_t ck_index) {
  if ( ck_index == 0 ) {
    Teensy3DMX.sendRDMDiscoveryMute(BROADCAST_ALL_DEVICES_ID, RDM_DISC_UNMUTE);
  }

  if ( tableOfDevices.getUIDAt(ck_index, &found) )  {
    if ( testMute(found) ) {
      // device confirmed
      return ck_index += 6;
    }
    
    // device not found
    tableOfDevices.removeUIDAt(ck_index);
    tableChangedFlag = 1;
    return ck_index;
  }
  // index invalid
  return 0;
}

void identifyEach() {
  int i = 0;
  uint8_t notDone = 1;
  while ( notDone ) {
    i = tableOfDevices.getNextUID(i, &found);
    if ( i < 0 ) {
      notDone = 0;
    } else {
      //uint16_t data;  //for DMX address and identify device on/off
      uint8_t data[2];
      if ( Teensy3DMX.sendRDMGetCommand(found, RDM_DEVICE_START_ADDR, data, 2) ) {
        uint16_t addr = (data[0] << 8) | data[1];

        if ( addr == 0x0F ) {
          data[0] = 0x00;
          data[1] = 0x01;
          Teensy3DMX.sendRDMSetCommand(found, RDM_DEVICE_START_ADDR, (uint8_t*)data, 2);
        }
  
        data[0] = 0x01;
        Teensy3DMX.sendRDMSetCommand(found, RDM_IDENTIFY_DEVICE, (uint8_t*)data, 1);
        delay(2000);
        data[0] = 0x00;
        Teensy3DMX.sendRDMSetCommand(found, RDM_IDENTIFY_DEVICE, (uint8_t*)data, 1);
      }
    }
  }
}

//called when range responded, so divide into sub ranges push them on stack to be further checked
void pushActiveBranch(UID lower, UID upper) {
  if ( mid.becomeMidpoint(lower, upper) ) {
    discoveryTree.push(lower);
    discoveryTree.push(mid);
    discoveryTree.push(mid);
    discoveryTree.push(upper);
  } else {
    // No midpoint possible:  lower and upper are equal or a 1 apart
    checkDeviceFound(lower);
    checkDeviceFound(upper);
  }
}

void pushInitialBranch() {
  lower.setBytes(0);
  upper.setBytes(BROADCAST_ALL_DEVICES_ID);
  discoveryTree.push(lower);
  discoveryTree.push(upper);

  //ETC devices seem to only respond with wildcard or exact manufacturer ID
  lower.setBytes(0x657400000000);
  upper.setBytes(0x6574FFFFFFFF);
  discoveryTree.push(lower);
  discoveryTree.push(upper);
}

uint8_t checkNextRange() {
  if ( discoveryTree.pop(&upper) ) {
    if ( discoveryTree.pop(&lower) ) {
      if ( lower == upper ) {
        checkDeviceFound(lower);
      } else {        //not leaf so, check range lower->upper
        uint8_t result = Teensy3DMX.sendRDMDiscoveryPacket(lower, upper, &found);
        if ( result ) {
          //this range responded, so divide into sub ranges push them on stack to be further checked
          pushActiveBranch(lower, upper);
           
        } else if ( Teensy3DMX.sendRDMDiscoveryPacket(lower, upper, &found) ) {
            pushActiveBranch(lower, upper); //if discovery fails, try a second time
        }
      }         // end check range
      return 1; // UID ranges may be remaining to test
    }           // end valid pop
  }             // end valid pop  
  return 0;     // none left to pop
}



void testRDMDiscovery() {
  if ( discovery_state ) {
    // check the table of devices
    discovery_tbl_ck_index = checkTable(discovery_tbl_ck_index);
    if ( discovery_tbl_ck_index == 0 ) {
      // done with table check
      discovery_state = DISC_STATE_SEARCH;
      pushInitialBranch();

      if ( identifyFlag ) {   //once per cycle identify each device
        identifyEach();       //this is just to demonstrate GET device address
        identifyFlag = 0;     //and SET identify device
      }
      
      //if ( tableChangedFlag ) {   //if the table has changed...
        tableChangedFlag = 0;

        // if this were an Art-Net application, you would send an 
        // ArtTOD packet here, because the device table has changed.
        // for this test, we just print the list of devices
         Serial.println("_______________ Table Of Devices _______________");
         tableOfDevices.printTOD();
      //}
    }
  } else {    // search for devices in range popped from discoveryTree
    if ( checkNextRange() == 0 ) {
      // done with search
      discovery_tbl_ck_index = 0;
      discovery_state = DISC_STATE_TBL_CK;
    }
  }
}

/************************************************************************
	setup
*************************************************************************/
void setup() {
  Serial.begin(115200);
  Serial.print("setup... ");
  
  // pins used for testing library, unnecessary for this example
  pinMode(4, OUTPUT);  
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);          

  pinMode(DIRECTION_PIN, OUTPUT);
  Teensy3DMX.startRDM(DIRECTION_PIN, RDM_DIRECTION_OUTPUT, rx_polarity);
  Serial.println("setup complete");
}


/************************************************************************

  The main loop checks to see if the level of the designated slot has changed
  and prints the new level to the serial monitor.  If a PWM channel is assigned,
  it also sets the output level.
  
*************************************************************************/

void loop() {
  delay(2);
  testRDMDiscovery();
  
  Teensy3DMX.setSlot(7,testLevel);
  Teensy3DMX.setSlot(8,255);
  Teensy3DMX.setSlot(371,testLevel);
  Teensy3DMX.setSlot(22,255);
  loopDivider++;
  if ( loopDivider == 4 ) {
    testLevel++;
    loopDivider = 0;
  }
  if ( testLevel == 1 ) {
    delay(500);
    identifyFlag = 1;
  }
}
