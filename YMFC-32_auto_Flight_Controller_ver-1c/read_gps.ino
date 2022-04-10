///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//In this part the GPS module is setup and read.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void gps_setup(void) {

  Serial1.begin(9600);
  delay(350);

  //Disable GPGSV messages by using the ublox protocol.
  uint8_t Disable_GPGSV[11] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x03, 0x00, 0xFD, 0x15};
  Serial1.write(Disable_GPGSV, 11);
  delay(350);   //A small delay is added to give the GPS some time to respond @ 9600bps.
  //Set the refresh rate to 5Hz by using the ublox protocol.
  uint8_t Set_to_5Hz[14] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A};
  Serial1.write(Set_to_5Hz, 14);
  delay(350);   //A small delay is added to give the GPS some time to respond @ 9600bps.
  //Set the baud rate to 57.6kbps by using the ublox protocol.
  uint8_t Set_to_57kbps[28] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00,
                               0x00, 0xE1, 0x00, 0x00, 0x07, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE2, 0xE1
                              };
  Serial1.write(Set_to_57kbps, 28);
  delay(350);

  Serial1.begin(57600);
  delay(350);
}

void read_gps(void) {
//  int index;
  while (Serial1.available() && new_line_found == 0) {                                                   //Stay in this loop as long as there is serial information from the GPS available.
    char read_serial_byte = Serial1.read();                                                              //Load a new serial byte in the read_serial_byte variable.
    if (read_serial_byte == '$') {                                                                       //If the new byte equals a $ character.
      for (message_counter = 0; message_counter <= 99; message_counter ++) {                             //Clear the old data from the incomming buffer array.
        incomming_message[message_counter] = '-';                                                        //Write a - at every position.
      }
      message_counter = 0;                                                                               //Reset the message_counter variable because we want to start writing at the begin of the array.
    }
    else if (message_counter <= 99)message_counter ++;                                                   //If the received byte does not equal a $ character, increase the message_counter variable.
    incomming_message[message_counter] = read_serial_byte;                                               //Write the new received byte to the new position in the incomming_message array.
    if (read_serial_byte == '*') { 
      new_line_found = 1;                                                     //Every NMEA line end with a *. If this character is detected the new_line_found variable is set to 1.
//      incomming_message[message_counter + 1] = 0; 
//      Serial.print("GPS Line: ");
//      for(index = 0; index < 100; index++) {
//        read_serial_byte = incomming_message[index];
//        if(read_serial_byte != 0) {
//          Serial.print(read_serial_byte);
//        } else {
//          index = 100;
//        }  
//      }
//      Serial.println();
    }
  }

  //If the software has detected a new NMEA line it will check if it's a valid line that can be used.
  if (new_line_found == 1) {                                                                             //If a new NMEA line is found.
    new_line_found = 0;                                                                                  //Reset the new_line_found variable for the next line.
    if (incomming_message[4] == 'L' && incomming_message[5] == 'L' && incomming_message[7] == ',') {     //When there is no GPS fix or latitude/longitude information available.
//      Serial.println("GPS No Fix: ");
      digitalWrite(STM32_board_LED, !digitalRead(STM32_board_LED));                                      //Change the LED on the STM32 to indicate GPS reception.
      //Set some variables to 0 if no valid information is found by the GPS module. This is needed for GPS lost when flying.
      l_lat_gps = 0;
      l_lon_gps = 0;
      lat_gps_previous = 0;
      lon_gps_previous = 0;
      number_used_sats = 0;
    }
    //If the line starts with GA and if there is a GPS fix we can scan the line for the latitude, longitude and number of satellites.
    if (incomming_message[4] == 'G' && incomming_message[5] == 'A' && (incomming_message[44] == '1' || incomming_message[44] == '2')) {
//      Serial.println("GPS Data: ");
      lat_gps_actual = ((int)incomming_message[19] - 48) *  (long)10000000;                              //Filter the minutes for the GGA line multiplied by 10.
      lat_gps_actual += ((int)incomming_message[20] - 48) * (long)1000000;                               //Filter the minutes for the GGA line multiplied by 10.
      lat_gps_actual += ((int)incomming_message[22] - 48) * (long)100000;                                //Filter the minutes for the GGA line multiplied by 10.
      lat_gps_actual += ((int)incomming_message[23] - 48) * (long)10000;                                 //Filter the minutes for the GGA line multiplied by 10.
      lat_gps_actual += ((int)incomming_message[24] - 48) * (long)1000;                                  //Filter the minutes for the GGA line multiplied by 10.
      lat_gps_actual += ((int)incomming_message[25] - 48) * (long)100;                                   //Filter the minutes for the GGA line multiplied by 10.
      lat_gps_actual += ((int)incomming_message[26] - 48) * (long)10;                                    //Filter the minutes for the GGA line multiplied by 10.
      lat_gps_actual /= (long)6;                                                                         //To convert the minutes to degrees we need to divide the minutes by 6.
      lat_gps_actual += ((int)incomming_message[17] - 48) *  (long)100000000;                            //Add the degrees multiplied by 10.
      lat_gps_actual += ((int)incomming_message[18] - 48) *  (long)10000000;                             //Add the degrees multiplied by 10.
      lat_gps_actual /= 10;                                                                              //Divide everything by 10.

      lon_gps_actual = ((int)incomming_message[33] - 48) *  (long)10000000;                              //Filter the minutes for the GGA line multiplied by 10.
      lon_gps_actual += ((int)incomming_message[34] - 48) * (long)1000000;                               //Filter the minutes for the GGA line multiplied by 10.
      lon_gps_actual += ((int)incomming_message[36] - 48) * (long)100000;                                //Filter the minutes for the GGA line multiplied by 10.
      lon_gps_actual += ((int)incomming_message[37] - 48) * (long)10000;                                 //Filter the minutes for the GGA line multiplied by 10.
      lon_gps_actual += ((int)incomming_message[38] - 48) * (long)1000;                                  //Filter the minutes for the GGA line multiplied by 10.
      lon_gps_actual += ((int)incomming_message[39] - 48) * (long)100;                                   //Filter the minutes for the GGA line multiplied by 10.
      lon_gps_actual += ((int)incomming_message[40] - 48) * (long)10;                                    //Filter the minutes for the GGA line multiplied by 10.
      lon_gps_actual /= (long)6;                                                                         //To convert the minutes to degrees we need to divide the minutes by 6.
      lon_gps_actual += ((int)incomming_message[30] - 48) * (long)1000000000;                            //Add the degrees multiplied by 10.
      lon_gps_actual += ((int)incomming_message[31] - 48) * (long)100000000;                             //Add the degrees multiplied by 10.
      lon_gps_actual += ((int)incomming_message[32] - 48) * (long)10000000;                              //Add the degrees multiplied by 10.
      lon_gps_actual /= 10;                                                                              //Divide everything by 10.

      if (incomming_message[28] == 'N')latitude_north = 1;                                               //When flying north of the equator the latitude_north variable will be set to 1.
      else latitude_north = 0;                                                                           //When flying south of the equator the latitude_north variable will be set to 0.

      if (incomming_message[42] == 'E')longiude_east = 1;                                                //When flying east of the prime meridian the longiude_east variable will be set to 1.
      else longiude_east = 0;                                                                            //When flying west of the prime meridian the longiude_east variable will be set to 0.

      number_used_sats = ((int)incomming_message[46] - 48) * (long)10;                                   //Filter the number of satillites from the GGA line.
      number_used_sats += (int)incomming_message[47] - 48;                                               //Filter the number of satillites from the GGA line.

      if (lat_gps_previous == 0 && lon_gps_previous == 0) {                                              //If this is the first time the GPS code is used.
        lat_gps_previous = lat_gps_actual;                                                               //Set the lat_gps_previous variable to the lat_gps_actual variable.
        lon_gps_previous = lon_gps_actual;                                                               //Set the lon_gps_previous variable to the lon_gps_actual variable.
      }

      lat_gps_loop_add = (float)(lat_gps_actual - lat_gps_previous) / 10.0;                              //Divide the difference between the new and previous latitude by ten.
      lon_gps_loop_add = (float)(lon_gps_actual - lon_gps_previous) / 10.0;                              //Divide the difference between the new and previous longitude by ten.

      l_lat_gps = lat_gps_previous;                                                                      //Set the l_lat_gps variable to the previous latitude value.
      l_lon_gps = lon_gps_previous;                                                                      //Set the l_lon_gps variable to the previous longitude value.

      lat_gps_previous = lat_gps_actual;                                                                 //Remember the new latitude value in the lat_gps_previous variable for the next loop.
      lon_gps_previous = lon_gps_actual;                                                                 //Remember the new longitude value in the lat_gps_previous variable for the next loop.

      //The GPS is set to a 5Hz refresh rate. Between every 2 GPS measurments, 9 GPS values are simulated.
      gps_add_counter = 5;                                                                               //Set the gps_add_counter variable to 5 as a count down loop timer
      new_gps_data_counter = 9;                                                                          //Set the new_gps_data_counter to 9. This is the number of simulated values between 2 GPS measurements.
      lat_gps_add = 0;                                                                                   //Reset the lat_gps_add variable.
      lon_gps_add = 0;                                                                                   //Reset the lon_gps_add variable.
      new_gps_data_available = 1;                                                                        //Set the new_gps_data_available to indicate that there is new data available.
    }

    //If the line starts with SA and if there is a GPS fix we can scan the line for the fix type (none, 2D or 3D).
    if (incomming_message[4] == 'S' && incomming_message[5] == 'A')fix_type = (int)incomming_message[9] - 48;

  }

  //After 5 program loops 5 x 4ms = 20ms the gps_add_counter is 0.
  if (gps_add_counter == 0 && new_gps_data_counter > 0) {                                                 //If gps_add_counter is 0 and there are new GPS simulations needed.
    new_gps_data_available = 1;                                                                           //Set the new_gps_data_available to indicate that there is new data available.
    new_gps_data_counter --;                                                                              //Decrement the new_gps_data_counter so there will only be 9 simulations
    gps_add_counter = 5;                                                                                  //Set the gps_add_counter variable to 5 as a count down loop timer

    lat_gps_add += lat_gps_loop_add;                                                                      //Add the simulated part to a buffer float variable because the l_lat_gps can only hold integers.
    if (abs(lat_gps_add) >= 1) {                                                                          //If the absolute value of lat_gps_add is larger then 1.
      l_lat_gps += (int)lat_gps_add;                                                                      //Increment the lat_gps_add value with the lat_gps_add value as an integer. So no decimal part.
      lat_gps_add -= (int)lat_gps_add;                                                                    //Subtract the lat_gps_add value as an integer so the decimal value remains.
    }

    lon_gps_add += lon_gps_loop_add;                                                                      //Add the simulated part to a buffer float variable because the l_lat_gps can only hold integers.
    if (abs(lon_gps_add) >= 1) {                                                                          //If the absolute value of lat_gps_add is larger then 1.
      l_lon_gps += (int)lon_gps_add;                                                                      //Increment the lat_gps_add value with the lat_gps_add value as an integer. So no decimal part.
      lon_gps_add -= (int)lon_gps_add;                                                                    //Subtract the lat_gps_add value as an integer so the decimal value remains.
    }
  }

  if (new_gps_data_available) {                                                                           //If there is a new set of GPS data available.
    if (number_used_sats < 8)digitalWrite(STM32_board_LED, !digitalRead(STM32_board_LED));                //Change the LED on the STM32 to indicate GPS reception.
    else digitalWrite(STM32_board_LED, LOW);                                                              //Turn the LED on the STM solid on (LED function is inverted). Check the STM32 schematic.
    gps_watchdog_timer = millis();                                                                        //Reset the GPS watch dog tmer.
    new_gps_data_available = 0;                                                                           //Reset the new_gps_data_available variable.

    if (flight_mode >= 3 && waypoint_set == 0) {                                                          //If the flight mode is 3 (GPS hold) and no waypoints are set.
      waypoint_set = 1;                                                                                   //Indicate that the waypoints are set.
      l_lat_waypoint = l_lat_gps;                                                                         //Remember the current latitude as GPS hold waypoint.
      l_lon_waypoint = l_lon_gps;                                                                         //Remember the current longitude as GPS hold waypoint.
    }

    if (flight_mode >= 3 && waypoint_set == 1) {                                                          //If the GPS hold mode and the waypoints are stored.
      gps_lon_error = l_lon_waypoint - l_lon_gps;                                                         //Calculate the latitude error between waypoint and actual position.
      gps_lat_error = l_lat_gps - l_lat_waypoint;                                                         //Calculate the longitude error between waypoint and actual position.

      gps_lat_total_avarage -=  gps_lat_rotating_mem[ gps_rotating_mem_location];                         //Subtract the current memory position to make room for the new value.
      gps_lat_rotating_mem[ gps_rotating_mem_location] = gps_lat_error - gps_lat_error_previous;          //Calculate the new change between the actual pressure and the previous measurement.
      gps_lat_total_avarage +=  gps_lat_rotating_mem[ gps_rotating_mem_location];                         //Add the new value to the long term avarage value.

      gps_lon_total_avarage -=  gps_lon_rotating_mem[ gps_rotating_mem_location];                         //Subtract the current memory position to make room for the new value.
      gps_lon_rotating_mem[ gps_rotating_mem_location] = gps_lon_error - gps_lon_error_previous;          //Calculate the new change between the actual pressure and the previous measurement.
      gps_lon_total_avarage +=  gps_lon_rotating_mem[ gps_rotating_mem_location];                         //Add the new value to the long term avarage value.
      gps_rotating_mem_location++;                                                                        //Increase the rotating memory location.
      if ( gps_rotating_mem_location == 35) gps_rotating_mem_location = 0;                                //Start at 0 when the memory location 35 is reached.

      gps_lat_error_previous = gps_lat_error;                                                             //Remember the error for the next loop.
      gps_lon_error_previous = gps_lon_error;                                                             //Remember the error for the next loop.

      //Calculate the GPS pitch and roll correction as if the nose of the multicopter is facing north.
      //The Proportional part = (float)gps_lat_error * gps_p_gain.
      //The Derivative part = (float)gps_lat_total_avarage * gps_d_gain.
      gps_pitch_adjust_north = (float)gps_lat_error * gps_p_gain + (float)gps_lat_total_avarage * gps_d_gain;
      gps_roll_adjust_north = (float)gps_lon_error * gps_p_gain + (float)gps_lon_total_avarage * gps_d_gain;

      if (!latitude_north)gps_pitch_adjust_north *= -1;                                                   //Invert the pitch adjustmet because the quadcopter is flying south of the equator.
      if (!longiude_east)gps_roll_adjust_north *= -1;                                                     //Invert the roll adjustmet because the quadcopter is flying west of the prime meridian.

      //Because the correction is calculated as if the nose was facing north, we need to convert it for the current heading.
      gps_roll_adjust = ((float)gps_roll_adjust_north * cos(angle_yaw * 0.017453)) + ((float)gps_pitch_adjust_north * cos((angle_yaw - 90) * 0.017453));
      gps_pitch_adjust = ((float)gps_pitch_adjust_north * cos(angle_yaw * 0.017453)) + ((float)gps_roll_adjust_north * cos((angle_yaw + 90) * 0.017453));

      //Limit the maximum correction to 300. This way we still have full controll with the pitch and roll stick on the transmitter.
      if (gps_roll_adjust > 300) gps_roll_adjust = 300;
      if (gps_roll_adjust < -300) gps_roll_adjust = -300;
      if (gps_pitch_adjust > 300) gps_pitch_adjust = 300;
      if (gps_pitch_adjust < -300) gps_pitch_adjust = -300;
    }
  }

  if (gps_watchdog_timer + 1000 < millis()) {                                                             //If the watchdog timer is exceeded the GPS signal is missing.
    if (flight_mode >= 3 && start > 0) {                                                                  //If flight mode is set to 3 (GPS hold).
      flight_mode = 2;                                                                                    //Set the flight mode to 2.
      error = 4;                                                                                          //Output an error.
    }
  }

  if (flight_mode < 3 && waypoint_set > 0) {                                                              //If the GPS hold mode is disabled and the waypoints are set.
    gps_roll_adjust = 0;                                                                                  //Reset the gps_roll_adjust variable to disable the correction.
    gps_pitch_adjust = 0;                                                                                 //Reset the gps_pitch_adjust variable to disable the correction.
    if (waypoint_set == 1) {                                                                              //If the waypoints are stored
      gps_rotating_mem_location = 0;                                                                      //Set the gps_rotating_mem_location to zero so we can empty the
      waypoint_set = 2;                                                                                   //Set the waypoint_set variable to 2 as an indication that the buffer is not cleared.
    }
    gps_lon_rotating_mem[ gps_rotating_mem_location] = 0;                                                 //Reset the current gps_lon_rotating_mem location.
    gps_lat_rotating_mem[ gps_rotating_mem_location] = 0;                                                 //Reset the current gps_lon_rotating_mem location.
    gps_rotating_mem_location++;                                                                          //Increment the gps_rotating_mem_location variable for the next loop.
    if (gps_rotating_mem_location == 36) {                                                                //If the gps_rotating_mem_location equals 36, all the buffer locations are cleared.
      waypoint_set = 0;                                                                                   //Reset the waypoint_set variable to 0.
      //Reset the variables that are used for the D-controller.
      gps_lat_error_previous = 0;
      gps_lon_error_previous = 0;
      gps_lat_total_avarage = 0;
      gps_lon_total_avarage = 0;
      gps_rotating_mem_location = 0;
    }
  }
}
