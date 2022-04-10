void init_telemetry_data(String usrVal) {
    ymc32_fval = usrVal.toFloat();
    check_byte = 0;
    ready_to_send = 1;
}

void send_telemetry_data(void) {
  telemetry_loop_counter++;                                                                 //Increment the telemetry_loop_counter variable.
  if (telemetry_loop_counter == 1)telemetry_send_byte = 'J';                                //Send a J as start signature.
  if (telemetry_loop_counter == 2)telemetry_send_byte = 'B';                                //Send a B as start signature.
  if (telemetry_loop_counter == 3) {
    telemetry_send_byte = ymc32_command;                              
  }

  if (telemetry_loop_counter == 4) {
    telemetry_buffer_byte = ymc32_fval * 1000;;                                            //Store the compass heading as it can change during the next loop.
    telemetry_send_byte = telemetry_buffer_byte;                                            //Send the first 8 bytes of the compass heading variable.
  }
  if (telemetry_loop_counter == 5)telemetry_send_byte = telemetry_buffer_byte >> 8;        //Send the last 8 bytes of the compass heading variable.
  
  if (telemetry_loop_counter == send_telemetry_last_byte)telemetry_send_byte = check_byte;  //Send the check-byte (last visual number + 1)

  //After 125 loops the telemetry_loop_counter variable is reset. This way the telemetry data is send every half second.
  if (telemetry_loop_counter == (send_telemetry_last_byte + 1)) {
    telemetry_loop_counter = 0;                             //After 125 loops reset the telemetry_loop_counter variable
    
    // reset the send command and transmission vales
    ready_to_send = 0;
   if(TerminalAttached) {                              // ready for next command
       Serial.println("Telemetry Sent");
   }
  }

  //Send the telemetry_send_byte via the serial protocol via ouput PB0.
  //Send a start bit first.
  if(telemetry_loop_counter != 0) {
    if (telemetry_loop_counter <= send_telemetry_last_byte) {
       if(TerminalAttached) {                              // ready for next command
         Serial.print("Telemetry Loop Counter: ");
         Serial.print(telemetry_loop_counter);
         Serial.print(" , ");
         Serial.println(telemetry_send_byte, BIN);
       }
        check_byte ^= telemetry_send_byte;
        Serial1.write(telemetry_send_byte);
    }
  }
}
