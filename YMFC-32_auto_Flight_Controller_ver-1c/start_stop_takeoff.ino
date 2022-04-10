///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//In this part the starting, stopping and take-off detection is managed.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void start_stop_takeoff(void) {
  if (channel_3 < 1050 && channel_4 < 1050)start = 1;                              //For starting the motors: throttle low and yaw left (step 1).
  if (start == 1 && channel_3 < 1050 && channel_4 > 1450) {                        //When yaw stick is back in the center position start the motors (step 2).
    throttle = motor_idle_speed;                                                   //Set the base throttle to the motor_idle_speed variable.
    angle_pitch = angle_pitch_acc;                                                 //Set the gyro pitch angle equal to the accelerometer pitch angle when the quadcopter is started.
    angle_roll = angle_roll_acc;                                                   //Set the gyro roll angle equal to the accelerometer roll angle when the quadcopter is started.
    ground_pressure = actual_pressure;                                             //Register the pressure at ground level for altitude calculations.
    course_lock_heading = angle_yaw;                                               //Set the current compass heading as the course lock heading.
    acc_total_vector_at_start = acc_total_vector;                                  //Register the acceleration when the quadcopter is started.
    start = 2;                                                                     //Set the start variable to 2 to indicate that the quadcopter is started.
    acc_alt_integrated = 0;                                                        //Reset the integrated acceleration value.
    if (manual_takeoff_throttle > 1400 && manual_takeoff_throttle < 1600) {        //If the manual hover throttle is used and valid (between 1400us and 1600us pulse).
      takeoff_throttle = manual_takeoff_throttle - 1500;                           //Use the manual hover throttle.
      takeoff_detected = 1;                                                        //Set the auto take-off detection to 1, indicated that the quadcopter is flying.
      //Reset the PID controllers for a smooth take-off.
      pid_i_mem_roll = 0;
      pid_last_roll_d_error = 0;
      pid_output_roll = 0;
      pid_i_mem_pitch = 0;
      pid_last_pitch_d_error = 0;
      pid_output_pitch = 0;
      pid_i_mem_yaw = 0;
      pid_last_yaw_d_error = 0;
      pid_output_yaw = 0;
    }
    else if (manual_takeoff_throttle) {                                            //If the manual hover throttle value is invalid.
      error = 5;                                                                   //Error = 5.
      takeoff_throttle = 0;                                                        //No hover throttle compensation.
      start = 0;                                                                   //Set the start variable to 0 to stop the motors.
    }
  }
  //Stopping the motors: throttle low and yaw right.
  if (start == 2 && channel_3 < 1050 && channel_4 > 1950) {
    start = 0;                                                                     //Set the start variable to 0 to disable the motors.
    takeoff_detected = 0;                                                          //Reset the auto take-off detection.
  }

  if (takeoff_detected == 0 && start == 2) {                                       //When the quadcopter is started and no take-off is detected.
    if (channel_3 > 1480 && throttle < 1750) throttle++;                           //When the throttle is half way or higher, increase the throttle.
    if (throttle == 1750)error = 6;                                                //If take-off is not detected when the throttle has reached 1700: error = 6.
    if (channel_3 <= 1480) {                                                       //When the throttle is below the center stick position.
      if (throttle > motor_idle_speed)throttle--;                                  //Lower the throttle to the motor_idle_speed variable.
      //Reset the PID controllers for a smooth take-off.
      else {                                                                       //When the throttle is back at idle speed reset the PID controllers.
        pid_i_mem_roll = 0;
        pid_last_roll_d_error = 0;
        pid_output_roll = 0;
        pid_i_mem_pitch = 0;
        pid_last_pitch_d_error = 0;
        pid_output_pitch = 0;
        pid_i_mem_yaw = 0;
        pid_last_yaw_d_error = 0;
        pid_output_yaw = 0;
      }
    }
    if (acc_z_average_short_total / 25 - acc_total_vector_at_start > 800) {        //A take-off is detected when the quadcopter is accelerating.
      takeoff_detected = 1;                                                        //Set the take-off detected variable to 1 to indicate a take-off.
      pid_altitude_setpoint = ground_pressure - 22;                                //Set the altitude setpoint at groundlevel + approximately 2.2 meters.
      if (throttle > 1400 && throttle < 1700) takeoff_throttle = throttle - 1530;  //If the automated throttle is between 1400 and 1600us during take-off, calculate take-off throttle.
      else {                                                                       //If the automated throttle is not between 1400 and 1600us during take-off.
        takeoff_throttle = 0;                                                      //No take-off throttle is calculated.
        error = 7;                                                                 //Show error 7 on the red LED.
      }
    }
  }
}

