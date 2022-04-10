void check_gps(void) {
  data = 0;
  loop_counter = 0;
  Serial1.begin(9600);
  delay(250);

  while (loop_counter < 1000) {                                                           //Stay in this loop until the data variable data holds a q.
    if (loop_counter < 1000)loop_counter ++;
    delayMicroseconds(4000);                                                              //Wait for 4000us to simulate a 250Hz loop.
    if (loop_counter == 1) {
      Serial.println("");
      Serial.println("====================================================================");
      Serial.println("Checking gps data @ 9600bps");
      Serial.println("====================================================================");
    }
    if (loop_counter > 1 && loop_counter < 500)while (Serial1.available())Serial.print((char)Serial1.read());
    if (loop_counter == 500) {
      Serial.println("");
      Serial.println("====================================================================");
      Serial.println("Checking gps data @ 57600bps");
      Serial.println("====================================================================");
      delay(200);
      
      //Disable GPGSV messages
      uint8_t Disable_GPGSV[11] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x03, 0x00, 0xFD, 0x15};
      Serial1.write(Disable_GPGSV, 11);
      delay(350);
      //Set the refresh rate to 5Hz
      uint8_t Set_to_5Hz[14] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A};
      Serial1.write(Set_to_5Hz, 14);
      delay(350);
      //Set the baud rate to 57.6kbps
      uint8_t Set_to_57kbps[28] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00,
                                   0x00, 0xE1, 0x00, 0x00, 0x07, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE2, 0xE1
                                  };
      Serial1.write(Set_to_57kbps, 28);
      delay(200);

      Serial1.begin(57600);
      delay(200);


      while (Serial1.available())Serial1.read();
    }
    if (loop_counter > 500 && loop_counter < 1000)while (Serial1.available())Serial.print((char)Serial1.read());

  }
  loop_counter = 0;                                                                       //Reset the loop counter.
  print_intro();                                                                          //Print the intro to the serial monitor.
}


