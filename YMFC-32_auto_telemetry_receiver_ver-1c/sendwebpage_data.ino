void webpage_data(void){
    // System Status
    //battery data
    webSocketServer.sendData("D:B:" + String(battery_voltage) + ":" + String(battery_voltage_min) + ":" + String(battery_voltage_max));

    // telemetry rec time data
    webSocketServer.sendData("D:T:" + String(current_receive) + ":" + String(current_receive_min) + ":" + String(current_receive_max));

    // loop processing time data
    webSocketServer.sendData("D:L:" + String(loop_timer) + ":" + String(loop_timer_min)+ ":" + String(loop_timer_max));

    // Flight Status
    // Error Code
    webSocketServer.sendData("D:E:" + String(error));

    // Start Status
    webSocketServer.sendData("D:S:" + String(start_byte));

    // Takeoff Throttle
    webSocketServer.sendData("D:K:" + String(takeoff_throttle));

    // Flight Mode
    webSocketServer.sendData("D:F:" + String(flight_mode));

    // Heading Lock
    webSocketServer.sendData("D:H:" + String(heading_lock));

    // Roll Angle
    webSocketServer.sendData("D:R:" + String(roll_angle) + ":" + String(roll_angle_min) + ":" + String(roll_angle_max));

    // Pitch Angle
    webSocketServer.sendData("D:P:" + String(pitch_angle) + ":" + String(pitch_angle_min) + ":" + String(pitch_angle_max));

    // Altitude Meters
    webSocketServer.sendData("D:A:" + String(altitude_meters) + ":" + String(altitude_meters_min) + ":" + String(altitude_meters_max));

    // Temperature
    webSocketServer.sendData("D:D:" + String(temperature) + ":" + String(temperature_min) + ":" + String(temperature_max));

    // Number of Satellites
    webSocketServer.sendData("D:U:" + String(number_used_sats));

    // Fix Type
    webSocketServer.sendData("D:V:" + String(fix_type));

    // Actual Compass Heading
    webSocketServer.sendData("D:W:" + String(actual_compass_heading));

    // Latitude
    webSocketServer.sendData("D:X:" + String(l_lat_gps));

    // Longitude
    webSocketServer.sendData("D:Y:" + String(l_lon_gps));

    // Manual Takeoff Setting
    webSocketServer.sendData("D:0:" + String(manual_takeoff));
}
