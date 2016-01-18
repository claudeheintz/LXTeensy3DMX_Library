# LXTeensy3DMX
DMX Driver for Teensy 3.1/3.2 using Teensyduino

   LXTeensyDMX is a driver for sending or receiving DMX using a Teensy 3.1/3.2's UART0 RX pin 0, TX pin 1
   
   LXTeensyDMX output mode continuously sends DMX once its interrupts have been enabled using startOutput().
   Use setSlot() to set the level value for a particular DMX dimmer/address/channel.
   
   LXTeensyDMX input mode continuously receives DMX once its interrupts have been enabled using startInput()
   Use getSlot() to read the level value for a particular DMX dimmer/address/channel.
   
   LXTeensyDMX is used with a single instance called Teensy3DMX	