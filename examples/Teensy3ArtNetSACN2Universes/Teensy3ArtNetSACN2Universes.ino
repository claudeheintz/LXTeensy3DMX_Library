/**************************************************************************/
/*!
    @file     Teensy3ArtNetSACN2Universes.ino
    @author   Claude Heintz
    @license  BSD (see LXDMXEthernet.h or http://www.claudeheintzdesign.com/lx/opensource.html
    @copyright 2017 by Claude Heintz All Rights Reserved

    Example using LXEDMXEthernet_Library for 2 universes of DMX output using
    Teensy3.2 with a wiznet based shield to receve Art-Net or sACN e1.31
    
    Art-Net(TM) Designed by and Copyright Artistic Licence Holdings Ltd.
    sACN E 1.31 is a public standard published by the PLASA technical standards program
    
    Note:  This example requires the LXTeensy3DMX_Library for DMX serial output
           https://github.com/claudeheintz/LXTeensy3DMX_Library
           first universe pin 1
           second universe pin 8

           This example requires LXDMXEthernet_library for Art-Net/sACN interface
           https://github.com/claudeheintz/LXDMXEthernet_library
           
    @section  HISTORY

    v1.00 - First release January 2017
*/
/**************************************************************************/

#include <LXTeensy3DMX.h>
#include <rdm/UID.h>
#include <rdm/TOD.h>
#include <rdm/rdm_utility.h>
#include <LXTeensy3DMX2.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <LXDMXEthernet.h>
#include <LXArtNet.h>
#include <LXSACN.h>

// to use unicast sACN, uncomment the following:
#define use_multicast 1

byte mac[] = { 0x00, 0x08, 0xDC, 0x4C, 0x29, 0x7E }; //00:08:DC... Wiznet

uint8_t artnetBuffer[ARTNET_BUFFER_MAX];
uint8_t sACNBuffer[SACN_BUFFER_MAX];

// dmx protocol interface for parsing packets (created in setup)
LXArtNet* artNetInterface;
LXArtNet* artNetInterfaceUniverse2;
LXSACN* sACNInterface;
LXSACN* sACNInterfaceUniverse2;

// EthernetUDP instances (one for Art-Net, one for sACN) to let us send and receive packets over UDP
EthernetUDP aUDP;
EthernetUDP sUDP;
// Multicast uses 2 separate EthernetUDP objects
#if defined ( use_multicast )
EthernetUDP tUDP;
#endif

// RDM defines
#define DISC_STATE_SEARCH 0
#define DISC_STATE_TBL_CK 1

// RDM globals
uint8_t rdm_enabled = 1;
uint8_t discovery_state = DISC_STATE_TBL_CK;
uint8_t discovery_tbl_ck_index = 0;
uint8_t tableChangedFlag = 0;
TOD tableOfDevices;
TOD discoveryTree;

UID lower(0,0,0,0,0,0);
UID upper(0,0,0,0,0,0);
UID mid(0,0,0,0,0,0);
UID found(0,0,0,0,0,0);

// globals for main loop
int packetSize;
uint8_t read_result_artnet1;
uint8_t read_result_artnet2;
uint8_t read_result_sacn1;
uint8_t read_result_sacn2;

// ***** Pins *****
//                  GND  -
//                U1_IN  0   (rx1)
//               U1_OUT  1   (tx1)
#define W5500_RESET_PIN  2
#define          U1_DIR  3
//               U2_OUT  8   (tx3)
//                   CS 10
//                 MOSI 11
//                 MISO 12
//                  SCK 13
#define          U1_LED 14
#define          U2_LED 15
//                 3.3V --
//                5V In --

uint8_t led_state = 0;
uint8_t led_state2 = 0;

void blinkLED() {
  //Serial.println("LED");
  if ( led_state ) {
    digitalWrite(U1_LED, HIGH);
    led_state = 0;
  } else {
    digitalWrite(U1_LED, LOW);
    led_state = 1;
  }
}

void blinkLED2() {
  //Serial.println("LED2");
  if ( led_state2 ) {
    digitalWrite(U2_LED, HIGH);
    led_state2 = 0;
  } else {
    digitalWrite(U2_LED, LOW);
    led_state2 = 1;
  }
}

void artTodRequestReceived(uint8_t* type) {
  if ( type[0] ) {
    tableOfDevices.reset();
  }
  artNetInterface->send_art_tod(&aUDP, tableOfDevices.rawBytes(), tableOfDevices.count());
}

void artRDMReceived(uint8_t* pdata) {
  uint8_t plen = pdata[1] + 2;
  uint8_t j;

  uint8_t* pkt = Teensy3DMX.rdmPacket();
  for (j=0; j<plen; j++) {
    pkt[j+1] = pdata[j];
  }
  pkt[0] = 0xCC;

  if ( Teensy3DMX.sendRDMControllerPacket() ) {
    artNetInterface->send_art_rdm(&aUDP, Teensy3DMX.rdmData(), aUDP.remoteIP());
  }
  
}

/************************************************************************/

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

void checkDeviceFound(UID found) {
  if ( testMute(found) ) {
    tableOfDevices.add(found);
    tableChangedFlag = 1;
  }
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

void updateRDMDiscovery() {
  if ( discovery_state ) {
    // check the table of devices
    discovery_tbl_ck_index = checkTable(discovery_tbl_ck_index);
    
    if ( discovery_tbl_ck_index == 0 ) {
      // done with table check
      discovery_state = DISC_STATE_SEARCH;
      pushInitialBranch();
   
      if ( tableChangedFlag ) {   //if the table has changed...
        tableChangedFlag = 0;

        artNetInterface->send_art_tod(&aUDP, tableOfDevices.rawBytes(), tableOfDevices.count());
        // if this were an Art-Net application, you would send an 
        // ArtTOD packet here, because the device table has changed.
        // for this test, we just print the list of devices
        Serial.println("_______________ Table Of Devices _______________");
        tableOfDevices.printTOD();
      }
    } //end table check ended
  } else {    // search for devices in range popped from discoveryTree

    if ( checkNextRange() == 0 ) {
      // done with search
      discovery_tbl_ck_index = 0;
      discovery_state = DISC_STATE_TBL_CK;
    }
  }           //end search
}


/************************************************************************

  Setup initializes Ethernet and UDP
  
  It also creates the network protocol object,
  either an instance of LXWiFiArtNet or LXWiFiSACN.
  
  It then starts listening on the appropriate UDP port.

*************************************************************************/

void setup() {
  // reset procedure for w5500
  pinMode(W5500_RESET_PIN, OUTPUT);
  digitalWrite(W5500_RESET_PIN, LOW);
  delay(500);
  digitalWrite(W5500_RESET_PIN, HIGH);
  //Serial.begin(115200);
  //while(!Serial);

  // The following starts ethernet using DHCP
  Ethernet.begin(mac);

  // Initialize Interfaces
  uint8_t interface1univ = 0;	//zero based
  uint8_t interface2univ = 1;
  sACNInterface = new LXSACN(sACNBuffer);
  sACNInterface->enableHTP();
  sACNInterface->setUniverse(interface1univ+1);	         // for different universe, change this line and the multicast address below
  sACNInterfaceUniverse2 = new LXSACN(sACNBuffer);
  sACNInterfaceUniverse2->enableHTP();
  sACNInterfaceUniverse2->setUniverse(interface2univ+1);

  artNetInterface = new LXArtNet(Ethernet.localIP(), Ethernet.subnetMask(), artnetBuffer);
  artNetInterface->enableHTP();
  artNetInterface->setSubnetUniverse(0, interface1univ);  //for different subnet/universe, change this line
  if ( rdm_enabled ) {
    artNetInterface->setArtTodRequestCallback(&artTodRequestReceived);
    artNetInterface->setArtRDMCallback(&artRDMReceived);
  }
  
  artNetInterfaceUniverse2 = new LXArtNet(Ethernet.localIP(), Ethernet.subnetMask(), artnetBuffer);
  artNetInterfaceUniverse2->enableHTP();
  
  artNetInterfaceUniverse2->setSubnetUniverse(0, interface2univ);
  
  // set reply fields for second port
  // (each initialize of LXArtNet object sets this shared buffer to default)
  uint8_t* pollReply = artNetInterface->replyData();
  pollReply[26] = 'c';
  pollReply[173] = 2;    // number of ports
  pollReply[175] = 128;  //  can output from network (port2)
  pollReply[183] = 128;  //  good output... change if error  (port2)
  pollReply[191] = interface2univ;    //  universe  (port2)
  pollReply[212] = 2;    //  dhcp
  strcpy((char*)&pollReply[26], "Teensy3DMX");
  strcpy((char*)&pollReply[44], "Teensy3DMX");

// multicast requires modificaton of Ethernet library.  see LXDMXEthernet.h
#if defined ( use_multicast )
  sUDP.beginMulticast(IPAddress(239,255,0,1), sACNInterface->dmxPort());
  tUDP.beginMulticast(IPAddress(239,255,0,2), sACNInterface->dmxPort());
#else
  sUDP.begin(sACNInterface->dmxPort());
#endif
  aUDP.begin(artNetInterface->dmxPort());

  //announce presence via Art-Net Poll Reply
  artNetInterface->send_art_poll_reply(&aUDP);

  pinMode(U1_LED, OUTPUT);
  pinMode(U2_LED, OUTPUT);
  Teensy3DMX.setDirectionPin(U1_DIR);
  
  if ( rdm_enabled ) {
  	Teensy3DMX.startRDM(U1_DIR, RDM_DIRECTION_OUTPUT, RX_SIGNAL_INVERTED);
  } else {
    Teensy3DMX.startOutput();
  }
  Teensy3DMX2.startOutput();
} //setup

void copyDMX1ToOutput(void) {
  uint8_t a, s;
  uint16_t a_slots = artNetInterface->numberOfSlots();
  uint16_t s_slots = sACNInterface->numberOfSlots();
  for (int i=1; i <=DMX_UNIVERSE_SIZE; i++) {
    if ( i <= a_slots ) {
      a = artNetInterface->getHTPSlot(i);
    } else {
      a = 0;
    }
    if ( i <= s_slots ) {
      s = sACNInterface->getHTPSlot(i);
    } else {
      s = 0;
    }
    if ( a > s ) {
        Teensy3DMX.setSlot(i , a);
      } else {
        Teensy3DMX.setSlot(i , s);
      }
   }
}

void copyDMX2ToOutput(void) {
  uint8_t a, s;
  uint16_t a_slots = artNetInterfaceUniverse2->numberOfSlots();
  uint16_t s_slots = sACNInterfaceUniverse2->numberOfSlots();
  for (int i=1; i <=DMX_UNIVERSE_SIZE; i++) {
    if ( i <= a_slots ) {
      a = artNetInterfaceUniverse2->getHTPSlot(i);
    } else {
      a = 0;
    }
    if ( i <= s_slots ) {
      s = sACNInterfaceUniverse2->getHTPSlot(i);
    } else {
      s = 0;
    }
    if ( a > s ) {
        Teensy3DMX2.setSlot(i , a);
      } else {
        Teensy3DMX2.setSlot(i , s);
      }
   }
}

/************************************************************************

  The main loop checks for and reads packets from WiFi UDP socket
  connection.  readDMXPacketContents() returns true when a DMX packet is received.
  If the first universe does not match, try the second
  (assuming sender unicasts both universes to same IP address)
  In the case of multicast sACN, try to read a packet from both addresses

*************************************************************************/

void loop() {
  read_result_artnet1 = RESULT_NONE;
  read_result_artnet2 = RESULT_NONE;
  read_result_sacn1 = RESULT_NONE;
  read_result_sacn2 = RESULT_NONE;
  
  packetSize = aUDP.parsePacket();
  if ( packetSize ) {
  	packetSize = aUDP.read(artnetBuffer, ARTNET_BUFFER_MAX);
	  read_result_artnet1 = artNetInterface->readDMXPacketContents(&aUDP, packetSize);
	  if ( read_result_artnet1 == RESULT_NONE ) {				// if not good_dmx or otherwise handled, try 2nd universe
      read_result_artnet2 = artNetInterfaceUniverse2->readDMXPacketContents(&aUDP, packetSize);
	  }
  }
  
#if defined ( use_multicast )
  packetSize = sUDP.parsePacket();
  if ( packetSize ) {
    packetSize = sUDP.read(sACNBuffer, SACN_BUFFER_MAX);
  	read_result_sacn1 = sACNInterface->readDMXPacketContents(&sUDP, packetSize);
  }
  // read packet from 2nd multicast address
  packetSize = tUDP.parsePacket();
  if ( packetSize ) {
    packetSize = tUDP.read(sACNBuffer, SACN_BUFFER_MAX);
    read_result_sacn2 = sACNInterfaceUniverse2->readDMXPacketContents(&tUDP, packetSize);
  }
#else
  //assume sender unicasts both universes to same IP address
  packetSize = sUDP.parsePacket();
  if ( packetSize ) {
    packetSize = sUDP.read(sACNBuffer, SACN_BUFFER_MAX);
    read_result_sacn1 = sACNInterface->readDMXPacketContents(&sUDP, packetSize);
    if ( read_result_sacn1 == RESULT_NONE ) {       // if not good_dmx or otherwise handled, try 2nd universe
      read_result_sacn2 = sACNInterfaceUniverse2->readDMXPacketContents(&sUDP, packetSize);
    }   // was not dmx first universe
  }     // no packetSize
#endif

  if ((read_result_artnet1 == RESULT_DMX_RECEIVED) || (read_result_sacn1 == RESULT_DMX_RECEIVED)) {
    copyDMX1ToOutput();
    blinkLED();
  } else if ( rdm_enabled ) {
  	updateRDMDiscovery();
  }
  if ((read_result_artnet2 == RESULT_DMX_RECEIVED) || (read_result_sacn2 == RESULT_DMX_RECEIVED)) {
    copyDMX2ToOutput();
    blinkLED2();
  }
  
  uint8_t dhcpr = Ethernet.maintain();
  if (( dhcpr == 4 ) || (dhcpr == 2)) {	//renew/rebind success, update ArtPollReply
  	artNetInterface->setLocalIP(Ethernet.localIP(), Ethernet.subnetMask());
  	artNetInterfaceUniverse2->setLocalIP(Ethernet.localIP(), Ethernet.subnetMask());
  }
} //loop()
