#include "ADXL345.h"
//constructor
ADXL345::ADXL345(uint8_t cs) {
  _range = ADXL345_RANGE_2_G;
  _cs = cs;
}

// Setups the HW (reads coefficients values, etc.)
bool ADXL345::begin() {
  pinMode(_cs, OUTPUT);
  digitalWrite(_cs,HIGH);
  //enable SPI
  SPI.begin();
  // Enable measurements
  writeRegister(ADXL345_REG_POWER_CTL, 0x08); 
  /* Check connection */
  uint8_t deviceid = getDeviceID();
  if (deviceid != 0xE5)
  {
    Serial.println(deviceid, HEX);    /* No ADXL345 detected ... return false */
    return false;
  }
 
    
  return true;
}
//
////write a register
void ADXL345::writeRegister(char registerAddress, char value) {
	  SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE3));
	  //Set Chip Select pin low to signal the beginning of an SPI packet.
	  digitalWrite(_cs, LOW);
	  //Transfer the register address over SPI.
	  SPI.transfer(registerAddress);
	  //Transfer the desired register value over SPI.
	  SPI.transfer(value);
	  //Set the Chip Select pin high to signal the end of an SPI packet.
	  digitalWrite(_cs, HIGH);
	  SPI.endTransaction();
}
//
////read a register
void ADXL345::readRegister(char registerAddress, int numBytes, char * values){
	  //Since we're performing a read operation, the most significant bit of the register address should be set.
	  char address = 0x80 | registerAddress;
	  //If we're doing a multi-byte read, bit 6 needs to be set as well.
	  if(numBytes > 1)address = address | 0x40;
	  SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE3));
	  //Set the Chip select pin low to start an SPI packet.
	  digitalWrite(_cs, LOW);
	  //Transfer the starting register address that needs to be read.
	  SPI.transfer(address);
	  //Continue to read registers until we've read the number specified, storing the results to the input buffer.
	  for(int i=0; i<numBytes; i++){
	    values[i] = SPI.transfer(0x00);
	  }
	  //Set the Chips Select pin high to end the SPI packet.
	  digitalWrite(_cs, HIGH);
	  SPI.endTransaction();
	}
//
//
////Read the device ID (can be used to check connection)
uint8_t ADXL345::getDeviceID(void) {
  // Check device ID register
	readRegister(ADXL345_REG_DEVID, 1, _values);
    return int(_values[0]);
}
//
//Gets the most recent X axis value in g
int16_t ADXL345::getX(void) {
  readRegister(ADXL345_REG_DATAX0, 2, _values);
  return ((int8_t)_values[1]<<8)|(int8_t)_values[0];// * ADXL345_MG2G_MULTIPLIER;
}
//
// Gets the most recent Y axis value in g
int16_t ADXL345::getY(void) {
  readRegister(ADXL345_REG_DATAY0, 2, _values);
  return ((int8_t)_values[1]<<8)|(int8_t)_values[0];// * ADXL345_MG2G_MULTIPLIER;
}
//
// Gets the most recent Z axis value in g
int16_t ADXL345::getZ(void) {
  readRegister(ADXL345_REG_DATAZ0, 2, _values);
  return ((int8_t)_values[1]<<8)|(int8_t)_values[0];// * ADXL345_MG2G_MULTIPLIER;
}



// Sets the g range for the accelerometer
void ADXL345::setRange(range_t range)
{
  _values[0] = 0x00;
  /* Red the data format register to preserve bits */
  readRegister(ADXL345_REG_DATA_FORMAT, 1, _values);
  /* Update the data rate */
  _values[0] &= ~0x0F;
  _values[0] |= range;
  /* Make sure that the FULL-RES bit is enabled for range scaling */
  _values[0] |= 0x08;
  /* Write the register back to the IC */
  writeRegister(ADXL345_REG_DATA_FORMAT, _values[0]);
   /* Keep track of the current range (to avoid readbacks) */
  _range = range;
}

//Gets the g range for the accelerometer
range_t ADXL345::getRange(void)
{
  /* Read the data format register to preserve bits */
  readRegister(ADXL345_REG_DATA_FORMAT, 1, _values);
  return (range_t)(int(_values[0]) & 0x03);
}

//Sets the data rate for the ADXL345 (controls power consumption)
void ADXL345::setDataRate(dataRate_t dataRate)
{
  /* Note: The LOW_POWER bits are currently ignored and we always keep
     the device in 'normal' mode */
  writeRegister(ADXL345_REG_BW_RATE, dataRate);
}

//gets the data rate for the ADXL345 (controls power consumption)
dataRate_t ADXL345::getDataRate(void)
{ 
	readRegister(ADXL345_REG_BW_RATE, 1, _values);
    return (dataRate_t)(int(_values[0]) & 0x0F);
}


