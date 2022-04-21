//defines the writeEEPROM function
void writeEEPROM(int16_t deviceaddress, uint16_t eeaddress, uint8_t data ) {
  HWire.beginTransmission(deviceaddress);
  HWire.write((int16_t)(eeaddress >> 8)); //writes the MSB
  HWire.write((int16_t)(eeaddress & 0xFF)); //writes the LSB
  HWire.write(data);
  HWire.endTransmission();
  delay(10);
}

//defines the readEEPROM function
uint8_t readEEPROM(int deviceaddress, uint16_t eeaddress ) {
  uint8_t rdata = 0xFF;
  
  HWire.beginTransmission(deviceaddress);
  HWire.write((int16_t)(eeaddress >> 8)); //writes the MSB
  HWire.write((int16_t)(eeaddress & 0xFF)); //writes the LSB
  HWire.endTransmission();
  HWire.requestFrom(deviceaddress,1);
  if (HWire.available()) {
    rdata = HWire.read();
  }
  delay(10);
  return rdata;
}

// initialize the EEPROM
void initEEprom(void) {
    writeEEPROM(eeprom_address, 100, 'Y');
    writeEEPROM(eeprom_address, 101, 'M');
    writeEEPROM(eeprom_address, 102, 'F');
    writeEEPROM(eeprom_address, 103, 'C');
}

// save the calibration data
void saveCalibration(void) {
    writeEEPROM(eeprom_address, 0, compass_cal_values[0] & 0b11111111);
    writeEEPROM(eeprom_address, 1, compass_cal_values[0] >> 8);
    writeEEPROM(eeprom_address, 2, compass_cal_values[1] & 0b11111111);
    writeEEPROM(eeprom_address, 3, compass_cal_values[1] >> 8);
    writeEEPROM(eeprom_address, 4, compass_cal_values[2] & 0b11111111);
    writeEEPROM(eeprom_address, 5, compass_cal_values[2] >> 8);
    writeEEPROM(eeprom_address, 6, compass_cal_values[3] & 0b11111111);
    writeEEPROM(eeprom_address, 7, compass_cal_values[3] >> 8);
    writeEEPROM(eeprom_address, 8, compass_cal_values[4] & 0b11111111);
    writeEEPROM(eeprom_address, 9, compass_cal_values[4] >> 8);
    writeEEPROM(eeprom_address, 10, compass_cal_values[5] & 0b11111111);
    writeEEPROM(eeprom_address, 11, compass_cal_values[5] >> 8);
}

// save the compass data
void saveCompass(void) {
    writeEEPROM(eeprom_address, 12, acc_pitch_cal_value & 0b11111111);
    writeEEPROM(eeprom_address, 13, acc_pitch_cal_value >> 8);
    writeEEPROM(eeprom_address, 14, acc_roll_cal_value & 0b11111111);
    writeEEPROM(eeprom_address, 15, acc_roll_cal_value >> 8);
}

// save the PID Yaw Data
void savePIDY(void) {
    eeprom_save_byte = pid_p_gain_yaw * 1000;
    writeEEPROM(eeprom_address, 16, eeprom_save_byte & 0b11111111);
    writeEEPROM(eeprom_address, 17, eeprom_save_byte >> 8);
    eeprom_save_byte = pid_i_gain_yaw * 1000;
    writeEEPROM(eeprom_address, 18, eeprom_save_byte & 0b11111111);
    writeEEPROM(eeprom_address, 19, eeprom_save_byte >> 8);
    eeprom_save_byte = pid_d_gain_yaw * 1000;
    writeEEPROM(eeprom_address, 20, eeprom_save_byte & 0b11111111);
    writeEEPROM(eeprom_address, 21, eeprom_save_byte >> 8);
}

// save the PID Altitude Data
void savePIDA(void) {
    eeprom_save_byte = pid_p_gain_altitude * 1000;
    writeEEPROM(eeprom_address, 22, eeprom_save_byte & 0b11111111);
    writeEEPROM(eeprom_address, 23, eeprom_save_byte >> 8);
    eeprom_save_byte = pid_i_gain_altitude * 1000;
    writeEEPROM(eeprom_address, 24, eeprom_save_byte & 0b11111111);
    writeEEPROM(eeprom_address, 25, eeprom_save_byte >> 8);
    eeprom_save_byte = pid_d_gain_altitude * 1000;
    writeEEPROM(eeprom_address, 26, eeprom_save_byte & 0b11111111);
    writeEEPROM(eeprom_address, 27, eeprom_save_byte >> 8);
}

// save the PID Roll Data
void savePIDR(void) {
    eeprom_save_byte = pid_p_gain_roll * 1000;
    writeEEPROM(eeprom_address, 28, eeprom_save_byte & 0b11111111);
    writeEEPROM(eeprom_address, 29, eeprom_save_byte >> 8);
    eeprom_save_byte = pid_i_gain_roll * 1000;
    writeEEPROM(eeprom_address, 30, eeprom_save_byte & 0b11111111);
    writeEEPROM(eeprom_address, 31, eeprom_save_byte >> 8);
    eeprom_save_byte = pid_d_gain_roll * 1000;
    writeEEPROM(eeprom_address, 32, eeprom_save_byte & 0b11111111);
    writeEEPROM(eeprom_address, 33, eeprom_save_byte >> 8);
}
