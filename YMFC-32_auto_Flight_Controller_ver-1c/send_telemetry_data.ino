///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//This part sends the telemetry data to the ground station.
//The output for the serial monitor is PB0. Protocol is 1 start bit, 8 data bits, no parity, 1 stop bit.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void send_telemetry_data(void) {
  telemetry_loop_counter++;                                                                 //Increment the telemetry_loop_counter variable.
  if (telemetry_loop_counter == 1)telemetry_send_byte = 'J';                                //Send a J as start signature.
  if (telemetry_loop_counter == 2)telemetry_send_byte = 'B';                                //Send a B as start signature.
  if (telemetry_loop_counter == 3) {
    telemetry_send_byte = error;                                                              //Send the error as a byte.
  }
  if (telemetry_loop_counter == 4)telemetry_send_byte = flight_mode;                        //Send the flight mode as a byte.
  if (telemetry_loop_counter == 5)telemetry_send_byte = debug_byte;                         //Send the debug_byte as a byte.
  if (telemetry_loop_counter == 6) {
  telemetry_receive_byte = temperature;                                                    //Store the temperature as it can change during the next loop.
  telemetry_send_byte = telemetry_receive_byte;                                            //Send the first 8 bytes of the temperature variable.
  }
  if (telemetry_loop_counter == 7)telemetry_send_byte = telemetry_receive_byte >> 8;         //Send the last 8 bytes of the temperature variable.
  if (telemetry_loop_counter == 8)telemetry_send_byte = angle_roll + 100;                   //Send the roll angle as a byte. Adding 100 prevents negative numbers.
  if (telemetry_loop_counter == 9)telemetry_send_byte = angle_pitch + 100;                  //Send the pitch angle as a byte. Adding 100 prevents negative numbers.
  if (telemetry_loop_counter == 10)telemetry_send_byte = start;                             //Send the error as a byte.
  if (telemetry_loop_counter == 11) {
  if (start == 2) {                                                                       //Only send the altitude when the quadcopter is flying.
    telemetry_receive_byte = 1000 + ((ground_pressure - actual_pressure) * 0.0842);        //Calculate the altitude and add 1000 to prevent negative numbers.
  }
  else {
    telemetry_receive_byte = 1000;                                                         //Send and altitude of 0 meters if the quadcopter isn't flying.
  }
  telemetry_send_byte = telemetry_receive_byte;                                            //Send the first 8 bytes of the altitude variable.
  }
  if (telemetry_loop_counter == 12)telemetry_send_byte = telemetry_receive_byte >> 8;        //Send the last 8 bytes of the altitude variable.

  if (telemetry_loop_counter == 13) {
  telemetry_receive_byte = 1500 + takeoff_throttle;                                        //Store the take-off throttle as it can change during the next loop.
  telemetry_send_byte = telemetry_receive_byte;                                            //Send the first 8 bytes of the take-off throttle variable.
  }
  if (telemetry_loop_counter == 14)telemetry_send_byte = telemetry_receive_byte >> 8;        //Send the last 8 bytes of the take-off throttle variable.
  if (telemetry_loop_counter == 15) {
  telemetry_receive_byte = angle_yaw;                                                      //Store the compass heading as it can change during the next loop.
  telemetry_send_byte = telemetry_receive_byte;                                            //Send the first 8 bytes of the compass heading variable.
  }
  if (telemetry_loop_counter == 16)telemetry_send_byte = telemetry_receive_byte >> 8;        //Send the last 8 bytes of the compass heading variable.
  if (telemetry_loop_counter == 17)telemetry_send_byte = heading_lock;                      //Send the heading_lock variable as a byte.
  if (telemetry_loop_counter == 18)telemetry_send_byte = number_used_sats;                  //Send the number_used_sats variable as a byte.
  if (telemetry_loop_counter == 19)telemetry_send_byte = fix_type;                          //Send the fix_type variable as a byte.
  if (telemetry_loop_counter == 20) {
  telemetry_receive_byte = l_lat_gps;                                                      //Store the latitude position as it can change during the next loop.
  telemetry_send_byte = telemetry_receive_byte;                                            //Send the first 8 bytes of the latitude position variable.
  }
  if (telemetry_loop_counter == 21)telemetry_send_byte = telemetry_receive_byte >> 8;        //Send the next 8 bytes of the latitude position variable.
  if (telemetry_loop_counter == 22)telemetry_send_byte = telemetry_receive_byte >> 16;       //Send the next 8 bytes of the latitude position variable.
  if (telemetry_loop_counter == 23)telemetry_send_byte = telemetry_receive_byte >> 24;       //Send the next 8 bytes of the latitude position variable.
  if (telemetry_loop_counter == 24) {
  telemetry_receive_byte = l_lon_gps;                                                      //Store the longitude position as it can change during the next loop.
  telemetry_send_byte = telemetry_receive_byte;                                            //Send the first 8 bytes of the longitude position variable.
  }
  if (telemetry_loop_counter == 25)telemetry_send_byte = telemetry_receive_byte >> 8;        //Send the next 8 bytes of the longitude position variable.
  if (telemetry_loop_counter == 26)telemetry_send_byte = telemetry_receive_byte >> 16;       //Send the next 8 bytes of the longitude position variable.
  if (telemetry_loop_counter == 27)telemetry_send_byte = telemetry_receive_byte >> 24;       //Send the next 8 bytes of the longitude position variable.
  
  if (telemetry_loop_counter == 28) {
  telemetry_receive_byte = loop_timer_capture;                                                     //Store the latitude position as it can change during the next loop.
  telemetry_send_byte = telemetry_receive_byte;                                            //Send the first 8 bytes of the latitude position variable.
  }
  if (telemetry_loop_counter == 29)telemetry_send_byte = telemetry_receive_byte >> 8;        //Send the next 8 bytes of the latitude position variable.
  if (telemetry_loop_counter == 30)telemetry_send_byte = telemetry_receive_byte >> 16;       //Send the next 8 bytes of the latitude position variable.
  if (telemetry_loop_counter == 31)telemetry_send_byte = telemetry_receive_byte >> 24;       //Send the next 8 bytes of the latitude position variable.

  if (telemetry_loop_counter == 32) {
  telemetry_receive_byte = pid_p_gain_roll * 1000;                                                //Store the PID-P Roll Setting
  telemetry_send_byte = telemetry_receive_byte;                                            
  }
  if (telemetry_loop_counter == 33)telemetry_send_byte = telemetry_receive_byte >> 8;        

  if (telemetry_loop_counter == 34) {
  telemetry_receive_byte = pid_i_gain_roll * 1000;                                                //Store the PID-I Roll Setting
  telemetry_send_byte = telemetry_receive_byte;                                            
  }
  if (telemetry_loop_counter == 35)telemetry_send_byte = telemetry_receive_byte >> 8;        

  if (telemetry_loop_counter == 36) {
  telemetry_receive_byte = pid_d_gain_roll * 1000;                                                //Store the PID-D Roll Setting
  telemetry_send_byte = telemetry_receive_byte;                                            
  }
  if (telemetry_loop_counter == 37)telemetry_send_byte = telemetry_receive_byte >> 8;        

  if (telemetry_loop_counter == 38) {
  telemetry_receive_byte = pid_p_gain_yaw * 1000;                                                //Store the PID-P Yaw Setting
  telemetry_send_byte = telemetry_receive_byte;                                            
  }
  if (telemetry_loop_counter == 39)telemetry_send_byte = telemetry_receive_byte >> 8;        

  if (telemetry_loop_counter == 40) {
  telemetry_receive_byte = pid_i_gain_yaw * 1000;                                                //Store the PID-I Yaw Setting
  telemetry_send_byte = telemetry_receive_byte;                                            
  }
  if (telemetry_loop_counter == 41)telemetry_send_byte = telemetry_receive_byte >> 8;        

  if (telemetry_loop_counter == 42) {
  telemetry_receive_byte = pid_d_gain_yaw * 1000;                                                //Store the PID-D Yaw Setting
  telemetry_send_byte = telemetry_receive_byte;                                            
  }
  if (telemetry_loop_counter == 43)telemetry_send_byte = telemetry_receive_byte >> 8;        

  if (telemetry_loop_counter == 44) {
  telemetry_receive_byte = pid_p_gain_altitude * 1000;                                                //Store the PID-P Altitude Setting
  telemetry_send_byte = telemetry_receive_byte;                                            
  }
  if (telemetry_loop_counter == 45)telemetry_send_byte = telemetry_receive_byte >> 8;        

  if (telemetry_loop_counter == 46) {
  telemetry_receive_byte = pid_i_gain_altitude * 1000;                                                //Store the PID-I Altitude Setting
  telemetry_send_byte = telemetry_receive_byte;                                            
  }
  if (telemetry_loop_counter == 47)telemetry_send_byte = telemetry_receive_byte >> 8;        

  if (telemetry_loop_counter == 48) {
  telemetry_receive_byte = pid_d_gain_altitude * 1000;                                                //Store the PID-D Altitude Setting
  telemetry_send_byte = telemetry_receive_byte;                                            
  }
  if (telemetry_loop_counter == 49)telemetry_send_byte = telemetry_receive_byte >> 8;        

  if (telemetry_loop_counter == 50) {
  telemetry_receive_byte = battery_voltage * 100;                                                     //Store the Battery Voltage Setting
  telemetry_send_byte = telemetry_receive_byte;                                            
  }
  if (telemetry_loop_counter == 51)telemetry_send_byte = telemetry_receive_byte >> 8;
  if (telemetry_loop_counter == 52) {
    telemetry_receive_byte = manual_takeoff_throttle;                                 //Store the manual take-off setting as it can change during the next loop.
    telemetry_send_byte = telemetry_receive_byte;                                            //Send the first 8 bytes of the manual take-off setting variable.
  }
  if (telemetry_loop_counter == 53)telemetry_send_byte = telemetry_receive_byte >> 8;        //Send the last 8 bytes of the take-off throttle variable.

  // this is the last byte
  if (telemetry_loop_counter == telemetry_last_byte)telemetry_send_byte = check_byte;       //Send the check-byte (last visual number + 1)

  //After 125 loops the telemetry_loop_counter variable is reset. This way the telemetry data is send every half second.
  if (telemetry_loop_counter == 125) {
    telemetry_loop_counter = 0;                             //After 125 loops reset the telemetry_loop_counter variable
    check_byte = 0;
  }

  //Send the telemetry_send_byte via the serial protocol via output PB0.
  //Send a start bit first.
  if(telemetry_loop_counter != 0) {
    if (telemetry_loop_counter <= telemetry_last_byte) {
      check_byte ^= telemetry_send_byte;                                                           //Set output PB0 to 1;
      Serial.write(telemetry_send_byte);
    }
  }
}

void rec_telemetry_data(void) {
    if(Serial.available()) {                                                //If there are bytes available.
      receive_byte = Serial.read();                                       //Load them in the received_buffer array.
        switch (receive_state) {
            // waiting for start
            case 1:
                if(receive_byte == 'B') {
                    if(receive_byte_previous == 'J') {
                        receive_buffer_counter = 2;
                        receive_buffer[0] = 'J';
                        receive_buffer[1] = 'B';
                        receive_state = 2;
                    }
                }
                receive_byte_previous = receive_byte;
            break;
            // building buffer
            case 2:
                receive_buffer[receive_buffer_counter++] = receive_byte;
                // see if we have a valid buffer
                if(receive_buffer_counter == rec_telemetry_last_byte) {
                    check_byte = 0;                                                                         //Reset the check_byte variable.
                    for(temp_byte=0;temp_byte < (rec_telemetry_last_byte - 1); temp_byte ++) {
                        check_byte ^= receive_buffer[temp_byte];                                            //Calculate the check_byte.
                    }
                    // valid buffer
                    if(check_byte == receive_buffer[(rec_telemetry_last_byte - 1)]) {
                      // get the data
                      ymc32_command = receive_buffer[2];
                      ymc32_fval = (float)(receive_buffer[3] | receive_buffer[4] << 8)/1000;

                      // process the data
                      process_telemetry_data();

                      // wait for next command
                      receive_state = 1;
                    } else {
                        receive_byte_previous = receive_byte;
                        receive_state = 1;
                    }
                } else if (receive_buffer_counter > rec_telemetry_last_byte) {
                    receive_byte_previous = receive_byte;
                    receive_state = 1;
                }
            break;
        }
    }
}

void process_telemetry_data(void) {
    //error = ymc32_fval;
    switch (ymc32_command) {
      case 1:
        pid_p_gain_roll = ymc32_fval;
      break;
      case 2:
        pid_i_gain_roll = ymc32_fval;
      break;
      case 3:
        pid_d_gain_roll = ymc32_fval;
      break;
      case 4:
        pid_p_gain_yaw = ymc32_fval;
      break;
      case 5:
        pid_i_gain_yaw = ymc32_fval;
      break;
      case 6:
        pid_d_gain_yaw = ymc32_fval;
      break;
      case 7:
        pid_p_gain_altitude = ymc32_fval;
      break;
      case 8:
        pid_i_gain_altitude = ymc32_fval;
      break;
      case 9:
        pid_d_gain_altitude = ymc32_fval;
      break;
      case 10:
          if(manual_takeoff_throttle == 1500) {
            manual_takeoff_throttle = 0;
          } else {
            manual_takeoff_throttle = 1500;
          }
      break;
      case 99:
          rdc_start = 1;
      break;
    }
}
