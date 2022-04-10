//defines the writeEEPROM function
void writeEEPROM(int deviceaddress, unsigned int eeaddress, byte data ) {
  HWire.beginTransmission(deviceaddress);
  HWire.write((int)(eeaddress >> 8)); //writes the MSB
  HWire.write((int)(eeaddress & 0xFF)); //writes the LSB
  HWire.write(data);
  HWire.endTransmission();
  delay(10);
}

//defines the readEEPROM function
byte readEEPROM(int deviceaddress, unsigned int eeaddress ) {
  byte rdata = 0xFF;
  
  HWire.beginTransmission(deviceaddress);
  HWire.write((int)(eeaddress >> 8)); //writes the MSB
  HWire.write((int)(eeaddress & 0xFF)); //writes the LSB
  HWire.endTransmission();
  HWire.requestFrom(deviceaddress,1);
  if (HWire.available()) {
    rdata = HWire.read();
  }
  delay(10);
  return rdata;
}

