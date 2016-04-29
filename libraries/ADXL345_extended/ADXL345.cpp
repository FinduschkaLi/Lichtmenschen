#include "ADXL345.h"

uint8_t 	 (* ADXL345::_fpISR)(void) = 0; //super important. A static member is just declared not defined in the header
//constructor
ADXL345::ADXL345(uint8_t cs) {
  _range = A_RG_2_G;
  _cs = cs;
  _intenable = false;
}
ADXL345::ADXL345(uint8_t cs, uint8_t int1) {
  _range = A_RG_2_G;
  _cs = cs;
  _int1 = int1;
  _intenable = true;
}

// Setups the HW (reads coefficients values, etc.)
bool ADXL345::begin() {
  pinMode(_cs, OUTPUT);
  //initialize interrupt
 
  digitalWrite(_cs,HIGH);
  //enable SPI
  SPI.begin();
  
  if(_intenable == true){
	  pinMode(_int1, INPUT);
	  //Create an interrupt that will trigger when a tap is detected.
	  // tax,	 lat,	 win,	 tres,	 tdur
	  configTap(A_TAP_Z|A_TAP_Y|A_TAP_X , 0x10 , 0x10 , 0x10 , 0x10);
	  startTab();
	  clearIntSource();//needed to allow next interrupt
  }
  
  // Enable measurements
  writeRegister(A_R_POWER_CTL, 0x08); 
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
	readRegister(A_R_DEVID, 1, _values);
    return int(_values[0]);
}
//
//Gets the most recent X axis value in g
int16_t ADXL345::getX(void) {
  readRegister(A_R_DATAX0, 2, _values);
  return ((int8_t)_values[1]<<8)|(int8_t)_values[0];// * A_MG2G_MULTIPLIER;
}
//
// Gets the most recent Y axis value in g
int16_t ADXL345::getY(void) {
  readRegister(A_R_DATAY0, 2, _values);
  return ((int8_t)_values[1]<<8)|(int8_t)_values[0];// * A_MG2G_MULTIPLIER;
}
//
// Gets the most recent Z axis value in g
int16_t ADXL345::getZ(void) {
  readRegister(A_R_DATAZ0, 2, _values);
  return ((int8_t)_values[1]<<8)|(int8_t)_values[0];// * A_MG2G_MULTIPLIER;
}



// Sets the g range for the accelerometer
void ADXL345::setRange(range_t range)
{
  _values[0] = 0x00;
  /* Red the data format register to preserve bits */
  readRegister(A_R_DATA_FORMAT, 1, _values);
  /* Update the data rate */
  _values[0] &= ~0x0F;
  _values[0] |= range;
  /* Make sure that the FULL-RES bit is enabled for range scaling */
  _values[0] |= 0x08;
  /* Write the register back to the IC */
  writeRegister(A_R_DATA_FORMAT, _values[0]);
   /* Keep track of the current range (to avoid readbacks) */
  _range = range;
}

//Gets the g range for the accelerometer
range_t ADXL345::getRange(void)
{
  /* Read the data format register to preserve bits */
  readRegister(A_R_DATA_FORMAT, 1, _values);
  return (range_t)(int(_values[0]) & 0x03);
}

//Sets the data rate for the ADXL345 (controls power consumption)
void ADXL345::setDataRate(dataRate_t dataRate)
{
  /* Note: The LOW_POWER bits are currently ignored and we always keep
     the device in 'normal' mode */
  writeRegister(A_R_BW_RATE, dataRate);
}

//gets the data rate for the ADXL345 (controls power consumption)
dataRate_t ADXL345::getDataRate(void)
{ 
	readRegister(A_R_BW_RATE, 1, _values);
    return (dataRate_t)(int(_values[0]) & 0x0F);
}
//initializes the tab settings (Tab sensitive Axis, Latency between two tabs, Window for 2nd tab to occur, Treshhold, tab duration)
void ADXL345::configTap(char tax,char lat, char win,char tres,char tdur){
	  //Send the Tap and Double Tap Interrupts to INT1 pin
	    writeRegister(A_R_INT_MAP, A_TAB_S1D1);
	    //Look for taps on XYZ Axes
	    writeRegister(A_R_TAP_AXES, tax);
	    //Set the Tap Threshold
	    writeRegister(A_R_THRESH_TAP, tres);
	    //Set the Tap Duration that must be reached
	    writeRegister(A_R_DUR, tdur);
	    //100ms Latency before the second tap can occur.
	    writeRegister(A_R_LATENT, lat);
	    writeRegister(A_R_WINDOW, win);
	    //Enable the Single
	    writeRegister(A_R_INT_ENABLE, A_TAB_S);

}
void ADXL345::stopTab(void){
	detachInterrupt(digitalPinToInterrupt(_int1));
}

void ADXL345::attachISR( uint8_t (*fp)(void) ) {
	ADXL345::_fpISR = fp; 
}
void ADXL345::callISR(void){
	  uint8_t r;
	  if( 0 != 	ADXL345::_fpISR ) {
	    r = (*	ADXL345::_fpISR)();
	  }
	  else {
	    // some error or default action here
		r=0;
	  }
	  //delete Interrupt register for next event

}
void ADXL345::startTab(void){
	attachInterrupt(digitalPinToInterrupt(_int1),ADXL345::callISR,RISING);
}
void ADXL345::clearIntSource(void){
	readRegister(A_R_INT_SOURCE,1,_values);
}
