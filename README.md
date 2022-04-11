# Arduino-Drone-YMC32-C
Modified Version of Joop Brokking's YMC32 Autonomous Quadcopter.  Huge thank you to Joop Brokking for his hard work, great videos and ongoing inspiration.

I added an 24LC256 IC2 EEPROM to save the calibration parameters, debug data and waypoints, modified PID parameters, etc..

I enhance the telemetry system to use two-way communications via serial port.  The new telemetry interface allows me to change the PID tuning parameters using user supplied floating point values, send debug data, etc.

I added some noise reduction capacitors and delayed boot capacitor.

I added a switch, so I change the boot mode without having to move the jumper.

The telemetry receiver was moved from and Arduino UNO to an MRK1010.  The MRK1010 allows me to have a WiFi interface, so I can use a Laptop or Iphone while interacting with the YMC32.  The webpage.html is javascript and is compressed and base64 encoded to save space on the MKR1010.  I create a YouTube video (I'm no Joop that's for sure, but it gets the job done) a while back to show you how to create the file for the Arduino.  The Link is:
https://www.youtube.com/watch?v=oKCXiYc311A&t=149s - skip to about 1:18.



