void check_barometer(void) {
  loop_counter = 0;

  //Check if the MS5611 is responding.
  HWire.beginTransmission(MS5611_address);                        //Start communication with the MS5611.
  error = HWire.endTransmission();                                //End the transmission and register the exit status.
  if (error != 0) {                                               //If the exit status is not 0 an error occurred.
    Serial.print("MS5611 is not responding on address: ");        //Print the error on the screen.
    Serial.println(MS5611_address, HEX);
    data = 'q';                                                   //Set the data variable to 'q' so it automatically exits the loop.
  }
  else {                                                          //If the MS5611 is responding normal
    Serial.print("MS5611 found on address: ");                    //Print the conformation on the screen.
    Serial.println(MS5611_address, HEX);

    //For calculating the pressure the 6 calibration values need to be polled from the MS5611.
    //These 2 byte values are stored in the memory location 0xA2 and up.
    for (start = 1; start <= 6; start++) {
      HWire.beginTransmission(MS5611_address);                    //Start communication with the MPU-6050.
      HWire.write(0xA0 + start * 2);                              //Send the address that we want to read.
      HWire.endTransmission();                                    //End the transmission.

      HWire.requestFrom(MS5611_address, 2);                       //Request 2 bytes from the MS5611.
      C[start] = HWire.read() << 8 | HWire.read();                //Add the low and high byte to the C[x] calibration variable.
    }
    //Print the 6 calibration values on the screen.
    Serial.print("C1 = ");
    Serial.println(C[1]);
    Serial.print("C2 = ");
    Serial.println(C[2]);
    Serial.print("C3 = ");
    Serial.println(C[3]);
    Serial.print("C4 = ");
    Serial.println(C[4]);
    Serial.print("C5 = ");
    Serial.println(C[5]);
    Serial.print("C6 = ");
    Serial.println(C[6]);

    OFF_C2 = C[2] * pow(2, 16);                                   //This value is pre-calculated to offload the main program loop.
    SENS_C1 = C[1] * pow(2, 15);                                  //This value is pre-calculated to offload the main program loop.

    start = 0;
  }

  while (data != 'q') {                                           //Stay in this loop until the data variable data holds a q.
    loop_timer = micros() + 4000;                                 //Set the loop_timer variable to the current micros() value + 4000.
    if (Serial.available() > 0) {                                 //If serial data is available.
      data = Serial.read();                                       //Read the incomming byte.
      delay(100);                                                 //Wait for any other bytes to come in.
      while (Serial.available() > 0)loop_counter = Serial.read(); //Empty the Serial buffer.
    }

    barometer_counter ++;                                         //Increment the barometer_counter variable for the next step.

    if (barometer_counter == 1) {
      if (temperature_counter == 0) {
        //Get temperature data from MS-5611
        HWire.beginTransmission(MS5611_address);
        HWire.write(0x00);
        HWire.endTransmission();
        HWire.requestFrom(MS5611_address, 3);
        raw_temperature = HWire.read() << 16 | HWire.read() << 8 | HWire.read();
      }
      else {
        //Get pressure data from MS-5611
        HWire.beginTransmission(MS5611_address);
        HWire.write(0x00);
        HWire.endTransmission();
        HWire.requestFrom(MS5611_address, 3);
        raw_pressure = HWire.read() << 16 | HWire.read() << 8 | HWire.read();
      }

      temperature_counter ++;
      if (temperature_counter > 9) {
        temperature_counter = 0;
        //Request temperature data
        HWire.beginTransmission(MS5611_address);
        HWire.write(0x58);
        HWire.endTransmission();
      }
      else {
        //Request pressure data
        HWire.beginTransmission(MS5611_address);
        HWire.write(0x48);
        HWire.endTransmission();
      }
    }
    if (barometer_counter == 2) {
      //Calculate pressure as explained in the datasheet of the MS-5611.
      dT = C[5];
      dT <<= 8;
      dT *= -1;
      dT += raw_temperature;

      OFF = OFF_C2 + ((int64_t)dT * (int64_t)C[4]) / pow(2, 7);

      SENS = SENS_C1 + ((int64_t)dT * (int64_t)C[3]) / pow(2, 8);

      P = ((raw_pressure * SENS) / pow(2, 21) - OFF) / pow(2, 15);

      if (actual_pressure == 0) {
        actual_pressure = P;
        actual_pressure_fast = P;
        actual_pressure_slow = P;
      }

      actual_pressure_fast = actual_pressure_fast * (float)0.92 + P * (float)0.08;
      actual_pressure_slow = actual_pressure_slow * (float)0.99 + P * (float)0.01;
      actual_pressure_diff = actual_pressure_slow - actual_pressure_fast;
      if (actual_pressure_diff > 8)actual_pressure_diff = 8;
      if (actual_pressure_diff < -8)actual_pressure_diff = -8;
      if (actual_pressure_diff > 1 || actual_pressure_diff < -1)actual_pressure_slow -= actual_pressure_diff / 6.0;
      actual_pressure = actual_pressure_slow;
      if (start < 200){
        start++;
        actual_pressure = 0;
      }
      else Serial.println(actual_pressure,0);
    }
    if (barometer_counter == 3) {
      barometer_counter = 0;
    }
    while (loop_timer > micros());
  }
  loop_counter = 0;                                                                     //Reset the loop counter variable to 0.
  start = 0;
  print_intro();                                                                        //Print the intro to the serial monitor.
}
