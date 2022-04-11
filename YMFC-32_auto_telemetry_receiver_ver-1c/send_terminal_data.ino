void print_telemetry_data(void){
 Serial.print("Error: ");
 Serial.print(error);
 Serial.print(", ");

 Serial.print("Flight Mode: ");
 Serial.print(flight_mode);
 Serial.print(", ");

 Serial.print("Battery Voltage: ");
 Serial.print(battery_voltage, 2);
 Serial.print(", ");

 Serial.print("Temperature: ");
 Serial.print(temperature);
 Serial.print(", ");

 Serial.print("Roll Angle: ");
 Serial.print(roll_angle);
 Serial.print(", ");

 Serial.print("Pitch Angle: ");
 Serial.print(pitch_angle);
 Serial.print(", ");

 Serial.print("Yaw Angle: ");
 Serial.print(pitch_angle);
 Serial.print(", ");

 Serial.print("Start: ");
 Serial.print(start_byte);
 Serial.print(", ");

 Serial.print("Altitude: ");
 Serial.print(altitude_meters);
 Serial.print(", ");

 Serial.print("Min Altitude: ");
 Serial.print(altitude_meters_min);
 Serial.print(", ");

 Serial.print("Max Altitude: ");
 Serial.print(altitude_meters_max);
 Serial.print(", ");

 Serial.print("Takeoff Throttle: ");
 Serial.print(takeoff_throttle);
 Serial.print(", ");

 Serial.print("Manual Takeoff: ");
 Serial.print(manual_takeoff);
 Serial.print(", ");

 Serial.print("Loop Timer: ");
 Serial.print(loop_timer);
 Serial.print(", ");

 Serial.print("Rec Time: ");
 Serial.println(current_receive);
}
