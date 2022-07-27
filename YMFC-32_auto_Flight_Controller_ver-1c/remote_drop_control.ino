// the RDC servo wants a 1000us - 2000us pluse every 20,000us (50hz)
// the main loop runs at 4,000us so 4,000 * 5 = 20,0000
// time to open/close is 1950 (closed) - 1320 (open) = 630 delta / 10 increments = 63 units, so 4,000 * 5 * 63 = (1,260,000us = 1.26s)
// delay is 20,000us * 150 = 3 seconds
void process_rdc(void) {
  if(rdc_loop_count == 5) {
    //RDC
    switch (rdc_start) {
      // state 1 , open the door
      case 1:
          if(rdc_servoPos > 1320) {
            rdc_servoPos -= 10;
          } else {
            rdc_start = 2;
            rdc_delay = 0;
          }
      break;
      // state 2 = delay for 5 seconds
      case 2:
        //delay
        // we come here every 20,000 usec so 20,0000 * 250 = 5,000,000 usec or 5 secs
        rdc_delay++;
        if(rdc_delay > 250) {
            rdc_start = 3;
        }
      break;
      // state 3 closed the door
      case 3:
          if(rdc_servoPos < 1950) {
            rdc_servoPos += 10;
          } else {
            rdc_start = 0;
          }
      break;
      default:
      break;
    }

    TIMER3_BASE->CCR3 = rdc_servoPos;                                              // set the servo pulse time
    TIMER3_BASE->CNT = 50000;  //(I set it to simulate 50000 for a 50hz loop to run servos)
    rdc_loop_count = 0;
  }
  rdc_loop_count++;
}
