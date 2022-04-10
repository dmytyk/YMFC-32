void check_compass(void) {
  loop_counter = 0;                                                                       //Reset the loop counter.
  battery_voltage = analogRead(4);                                                        //Set battery voltage.

  //Check if the compass is responding.
  HWire.beginTransmission(compass_address);                     //Start communication with the compass.
  error = HWire.endTransmission();                              //End the transmission and register the exit status.
  if (error != 0) {                                             //Print a message when the compass did not respond.
    Serial.println("Compass not responding");
  }
  else {
    HWire.beginTransmission(compass_address); //open communication with HMC5883
    HWire.write(0x00);
    HWire.write(0x10);
    HWire.write(0x60);
    HWire.write(0x00);
    HWire.endTransmission();
    delay(250);
    HWire.beginTransmission(compass_address);
    HWire.write(0x00);
    HWire.write(0x11);
    HWire.write(0x60);
    HWire.write(0x01);
    HWire.endTransmission();
    Serial.print("Positive bias test: ");
    delay(10);
    read_data();
    Serial.print("x: ");
    Serial.print(compass_x);
    Serial.print("  y: ");
    Serial.print(compass_y);
    Serial.print("  z: ");
    Serial.println(compass_z);

    HWire.beginTransmission(compass_address); //open communication with HMC5883
    HWire.write(0x00);
    HWire.write(0x12);
    HWire.write(0x60);
    HWire.write(0x01);
    HWire.endTransmission();
    Serial.print("Negative bias test: ");
    delay(10);
    read_data();
    Serial.print("x: ");
    Serial.print(compass_x);
    Serial.print("  y: ");
    Serial.print(compass_y);
    Serial.print("  z: ");
    Serial.println(compass_z);

    HWire.beginTransmission(compass_address); //open communication with HMC5883
    HWire.write(0x00);
    HWire.write(0x10);
    HWire.write(0x60);
    HWire.write(0x00);
    HWire.endTransmission();
    delay(2000);
  }


  while (data != 'q') {                                                                   //Stay in this loop until the data variable data holds a q.
    delayMicroseconds(3700);                                                              //Wait for 4000us to simulate a 250Hz loop.
    if (Serial.available() > 0) {                                                         //If serial data is available.
      data = Serial.read();                                                               //Read the incomming byte.
      delay(100);                                                                         //Wait for any other bytes to come in.
      while (Serial.available() > 0)loop_counter = Serial.read();                         //Empty the Serial buffer.
    }

    //Tell the HMC5883 where to begin reading data
    HWire.beginTransmission(compass_address);
    HWire.write(0x03); //select register 3, X MSB register
    HWire.endTransmission();

    read_data();
    loop_counter++;
    if (loop_counter == 125) {
      Serial.print("X-axis:");
      Serial.print (compass_x);
      Serial.print(" Z-axis:");
      Serial.print(compass_z);
      Serial.print(" Y-axis:");
      Serial.println(compass_y);
      loop_counter = 0;
    }
  }
  loop_counter = 0;                                                                       //Reset the loop counter.
  print_intro();                                                                          //Print the intro to the serial monitor.
}

void read_data() {
  HWire.requestFrom(compass_address, 6);
  compass_x = (HWire.read() << 8) | HWire.read();
  compass_z = (HWire.read() << 8) | HWire.read();
  compass_y = (HWire.read() << 8) | HWire.read();
}

