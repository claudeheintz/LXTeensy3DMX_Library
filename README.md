# LXTeensy3DMX
DMX Driver for Teensy 3.x using Teensyduino

   LXTeensyDMX is a driver for sending or receiving DMX using a Teensy 3.x's UART0 RX pin 0, TX pin 1
   
   LXTeensyDMX output mode continuously sends DMX once its interrupts have been enabled using startOutput().
   Use setSlot() to set the level value for a particular DMX dimmer/address/channel.
   
   LXTeensyDMX input mode continuously receives DMX once its interrupts have been enabled using startInput()
   Use getSlot() to read the level value for a particular DMX dimmer/address/channel.
   
   LXTeensyDMX rdm mode allows both sending and receiving. RDM mode can continuously send DMX, the same as when using output mode.  But, RDM mode can also pause regular DMX (zero start code) to send an RDM message and wait for a reply.  RDM mode can also be used to listen for both regular and RDM packets, responding to the RDM messages by sending replies.
   
   LXTeensyDMX is used with a single instance called Teensy3DMX	
   
   
   Additional instances using UART1 and UART2 are available by using Teensy3DMX1 and Teensy3DMX2.
   
   LXTeensyDMX has been tested with Teensy 3.2 and 3.6
   
   
   Support for Teensy4 is a separate project on GiHub:
   https://github.com/claudeheintz/LXTeensy4DMX_Library