#include "ADXL344.h"
//constructor
ADXL344::ADXL344(uint8_t cs) {
  _range = ADXL344_RANGE_2_G;
  _cs = cs;
}

// Setups the HW (reads coefficients values, etc.)
bool ADXL344::begin() {
  pinMode(_cs, OUTPUT);
  /* Check connection */
  uint8_t deviceid = 1;//getDeviceID();
  if (deviceid != 0xE5)
  {
    Serial.println(deviceid, HEX);    /* No ADXL344 detected ... return false */
    return false;
  }
  // Enable measurements
  writeRegister(ADXL344_REG_POWER_CTL, 0x08);  
    
  return true;
}
//
////write a register
void ADXL344::writeRegister(uint8_t registerAddress, char value) {
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
void ADXL344::readRegister(uint8_t registerAddress, int numBytes, char * values){
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
uint8_t ADXL344::getDeviceID(void) {
  // Check device ID register
	char val;
	readRegister(ADXL344_REG_DEVID, 1, &val);
  return val;
}
//
//Gets the most recent X axis value in g
int16_t ADXL344::getX(void) {
  char val;
  readRegister(ADXL344_REG_DATAX0, 2,   &val);
  return int(val) * ADXL344_MG2G_MULTIPLIER;
}
//
// Gets the most recent Y axis value in g
int16_t ADXL344::getY(void) {
  char val;
  readRegister(ADXL344_REG_DATAY0, 2,   &val);
  return int(val) * ADXL344_MG2G_MULTIPLIER;
}
//
// Gets the most recent Z axis value in g
int16_t ADXL344::getZ(void) {
  char val;
  readRegister(ADXL344_REG_DATAZ0, 2,   &val);
  return int(val) * ADXL344_MG2G_MULTIPLIER;
}



// Sets the g range for the accelerometer
void ADXL344::setRange(range_t range)
{
  /* Red the data format register to preserve bits */
  char format;
  readRegister(ADXL344_REG_DATA_FORMAT, 1, & format);
  /* Update the data rate */
  format &= ~0x0F;
  format |= range;
  /* Make sure that the FULL-RES bit is enabled for range scaling */
  format |= 0x08;
  /* Write the register back to the IC */
  writeRegister(ADXL344_REG_DATA_FORMAT, format);
   /* Keep track of the current range (to avoid readbacks) */
  _range = range;
}

//Gets the g range for the accelerometer
range_t ADXL344::getRange(void)
{
  /* Read the data format register to preserve bits */
  char val;
  readRegister(ADXL344_REG_DATA_FORMAT, 1, &val);
  return (range_t)(val & 0x03);
}

//Sets the data rate for the ADXL344 (controls power consumption)
void ADXL344::setDataRate(dataRate_t dataRate)
{
  /* Note: The LOW_POWER bits are currently ignored and we always keep
     the device in 'normal' mode */
  writeRegister(ADXL344_REG_BW_RATE, dataRate);
}

//gets the data rate for the ADXL344 (controls power consumption)
dataRate_t ADXL344::getDataRate(void)
{ 
	char val;
	readRegister(ADXL344_REG_BW_RATE, 1, &val);
    return (dataRate_t)(val & 0x0F);
}


