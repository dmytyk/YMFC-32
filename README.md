# Arduino-Drone-YMFC-32-C

Modified Version of Joop Brokking's YMFC32 Autonomous Quadcopter.  Huge thank you to Joop Brokking for his hard work, great videos (seriously great videos) and ongoing inspiration.

![YMFC-32-C](/Images/YMFC32-C.jpg)

I enhance the telemetry system to use two-way communications via the serial port.  The new telemetry interface allows me to change the PID tuning parameters with user supplied floating point values, change system variables and send debug data etc.
The telemetry receiver was moved from and Arduino UNO to an MRK1010.  The MRK1010 allows me to have a WiFi interface, so I can use a Laptop or Iphone while interacting with the YMFC32.  The webpage.html is javascript and is compressed and base64 encoded to save space on the MKR1010.  I create a YouTube video (I'm no Joop that's for sure, but it gets the job done) a while back to show you how to create the file for the Arduino.  The Link is:
[T002 Laser Tank Base64 Web Page](https://www.youtube.com/watch?v=oKCXiYc311A&t=149s) Fast forward to around 1 minute and 18 seconds, one of my first videos you can thank me later for skipping.

![Radio & Telemetry Combo](/Images/Radio_Telemetry_Combo.jpg)

![Telemetry Closeup](/Images/Telemetry_Closeup.PNG)

![Telemetry Ground Station](/Images/Telemetry_Ground_Station.jpg)

I added an 24LC256 IC2 EEPROM to save the calibration parameters (I don't have to reload them every time I change the code), debug data, waypoints, modified PID parameters, etc.

![EEPROM Storage](/Images/24LC256.jpg)

Added the RDC (Remote Drop Command) feature.  This option allows me to control an external servo, when triggered via the RDC Button, the YMFC32-C will open its storage door and release its payload.  For summer the payload is several parachute guys for the grandchildren and 4th of July it's ground snaps!

![RDC (Remote Drop Control)](/Images/RDC.jpg)

I also made a few minor hardware changes:

 - I added some noise reduction capacitors and delayed boot capacitor, the boot capacitor ensures all IC2 devices are booted before the STM32 starts to run.
 - I added a switch, so I change the boot mode without having to move the jumper, my fingers are too big!

![YMFC-32-C Schematic](/Images/YMFC32-C_Schematic.png)

