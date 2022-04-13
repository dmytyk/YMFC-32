# Arduino-Drone-YMC32-C
Modified Version of Joop Brokking's YMC32 Autonomous Quadcopter.  Huge thank you to Joop Brokking for his hard work, great videos (seriously great videos) and ongoing inspiration.

I added an 24LC256 IC2 EEPROM to save the calibration parameters, debug data, waypoints, modified PID parameters, etc..

I added some noise reduction capacitors and delayed boot capacitor, this ensures all IC2 devices are booted before the STM32.

I added a switch, so I change the boot mode without having to move the jumper, my fingers are too big!

I enhance the telemetry system to use two-way communications via serial port.  The new telemetry interface allows me to change the PID tuning parameters with user supplied floating point values, change system variables and send debug data etc. 
The telemetry receiver was moved from and Arduino UNO to an MRK1010.  The MRK1010 allows me to have a WiFi interface, so I can use a Laptop or Iphone while interacting with the YMC32.  The webpage.html is javascript and is compressed and base64 encoded to save space on the MKR1010.  I create a YouTube video (I'm no Joop that's for sure, but it gets the job done) a while back to show you how to create the file for the Arduino.  The Link is:
https://www.youtube.com/watch?v=oKCXiYc311A&t=149s - Fast forward to around 1 minute and 18 seconds.

Added the RDC (Remote Drop Command) feature.  This option allows me to control an external servo, when triggered via the RDC Button, the YMC32-C will open its storage door and release its payload.  For summer the payload is several parachute guys for the grandchildren and 4th of July it's ground snaps!

