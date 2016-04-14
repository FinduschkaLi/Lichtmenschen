/**************************************************************************/
/*!

*/
/**************************************************************************/



/**************************************************************************/
/*!
    @brief  Writes 8-bits to the specified destination register
*/
/**************************************************************************/
void ADXL345::writeRegister(uint8_t registerAddress, uint8_t value) {
	  //Set Chip Select pin low to signal the beginning of an SPI packet.
	  digitalWrite(P_MAG_CS, LOW);
	  //Transfer the register address over SPI.
	  SPI.transfer(registerAddress);
	  //Transfer the desired register value over SPI.
	  SPI.transfer(value);
	  //Set the Chip Select pin high to signal the end of an SPI packet.
	  digitalWrite(P_MAG_CS, HIGH);
}

/**************************************************************************/
/*!
    @brief  Reads 8-bits from the specified register
*/
/**************************************************************************/
void ADXL345::readRegister(uint8_t registerAddress, int numBytes, uint8_t * values){
	  //Since we're performing a read operation, the most significant bit of the register address should be set.
	  char address = 0x80 | registerAddress;
	  //If we're doing a multi-byte read, bit 6 needs to be set as well.
	  if(numBytes > 1)address = address | 0x40;
	  
	  //Set the Chip select pin low to start an SPI packet.
	  digitalWrite(P_MAG_CS, LOW);
	  //Transfer the starting register address that needs to be read.
	  SPI.transfer(address);
	  //Continue to read registers until we've read the number specified, storing the results to the input buffer.
	  for(int i=0; i<numBytes; i++){
	    values[i] = SPI.transfer(0x00);
	  }
	  //Set the Chips Select pin high to end the SPI packet.
	  digitalWrite(P_MAG_CS, HIGH);
	}


/**************************************************************************/
/*! 
    @brief  Read the device ID (can be used to check connection)
*/
/**************************************************************************/
uint8_t ADXL345::getDeviceID(void) {
  // Check device ID register
	readRegister(ADXL345_REG_DEVID, 1, uint8_t &val);
  return val;
}

/**************************************************************************/
/*! 
    @brief  Gets the most recent X axis value
*/
/**************************************************************************/
int16_t ADXL345::getX(void) {
  readRegister(ADXL345_REG_DATAX0, 2, int16_t_t &val );
  return val;
}

/**************************************************************************/
/*! 
    @brief  Gets the most recent Y axis value
*/
/**************************************************************************/
int16_t ADXL345::getY(void) {
	readRegister(ADXL345_REG_DATAY0, 2, int16_t_t &val );
	  return val;
}

/**************************************************************************/
/*! 
    @brief  Gets the most recent Z axis value
*/
/**************************************************************************/
int16_t ADXL345::getZ(void) {
	readRegister(ADXL345_REG_DATAZ0, 2, int16_t_t &val );
	  return val;
}


/**************************************************************************/
/*!
    @brief  Instantiates a new ADXL345 class in SPI mode
*/
/**************************************************************************/
ADXL345::ADXL345(uint8_t clock, uint8_t miso, uint8_t mosi, uint8_t cs, int32_t sensorID) {
  _sensorID = sensorID;
  _range = ADXL345_RANGE_2_G;
  _cs = cs;
  _clk = clock;
  _do = mosi;
  _di = miso;
}

/**************************************************************************/
/*!
    @brief  Setups the HW (reads coefficients values, etc.)
*/
/**************************************************************************/
bool ADXL345::begin() {
  
  
    pinMode(_cs, OUTPUT);
    pinMode(_clk, OUTPUT);
    digitalWrite(_clk, HIGH);
    pinMode(_do, OUTPUT);
    pinMode(_di, INPUT);


  /* Check connection */
  uint8_t deviceid = getDeviceID();
  if (deviceid != 0xE5)
  {
    /* No ADXL345 detected ... return false */
    Serial.println(deviceid, HEX);
    return false;
  }
  
  // Enable measurements
  writeRegister(ADXL345_REG_POWER_CTL, 0x08);  
    
  return true;
}

/**************************************************************************/
/*!
    @brief  Sets the g range for the accelerometer
*/
/**************************************************************************/
void Adafruit_ADXL345_Unified::setRange(range_t range)
{
  /* Red the data format register to preserve bits */
  uint8_t format = readRegister(ADXL345_REG_DATA_FORMAT);

  /* Update the data rate */
  format &= ~0x0F;
  format |= range;
  
  /* Make sure that the FULL-RES bit is enabled for range scaling */
  format |= 0x08;
  
  /* Write the register back to the IC */
  writeRegister(ADXL345_REG_DATA_FORMAT, format);
  
  /* Keep track of the current range (to avoid readbacks) */
  _range = range;
}

/**************************************************************************/
/*!
    @brief  Sets the g range for the accelerometer
*/
/**************************************************************************/
range_t Adafruit_ADXL345_Unified::getRange(void)
{
  /* Red the data format register to preserve bits */
  return (range_t)(readRegister(ADXL345_REG_DATA_FORMAT) & 0x03);
}

/**************************************************************************/
/*!
    @brief  Sets the data rate for the ADXL345 (controls power consumption)
*/
/**************************************************************************/
void Adafruit_ADXL345_Unified::setDataRate(dataRate_t dataRate)
{
  /* Note: The LOW_POWER bits are currently ignored and we always keep
     the device in 'normal' mode */
  writeRegister(ADXL345_REG_BW_RATE, dataRate);
}

/**************************************************************************/
/*!
    @brief  Sets the data rate for the ADXL345 (controls power consumption)
*/
/**************************************************************************/
dataRate_t Adafruit_ADXL345_Unified::getDataRate(void)
{
  return (dataRate_t)(readRegister(ADXL345_REG_BW_RATE) & 0x0F);
}

/**************************************************************************/
/*! 
    @brief  Gets the most recent sensor event
*/
/**************************************************************************/
bool Adafruit_ADXL345_Unified::getEvent(sensors_event_t *event) {
  /* Clear the event */
  memset(event, 0, sizeof(sensors_event_t));
  
  event->version   = sizeof(sensors_event_t);
  event->sensor_id = _sensorID;
  event->type      = SENSOR_TYPE_ACCELEROMETER;
  event->timestamp = 0;
  event->acceleration.x = getX() * ADXL345_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
  event->acceleration.y = getY() * ADXL345_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
  event->acceleration.z = getZ() * ADXL345_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
  
  return true;
}

/**************************************************************************/
/*! 
    @brief  Gets the sensor_t data
*/
/**************************************************************************/
void Adafruit_ADXL345_Unified::getSensor(sensor_t *sensor) {
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy (sensor->name, "ADXL345", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name)- 1] = 0;
  sensor->version     = 1;
  sensor->sensor_id   = _sensorID;
  sensor->type        = SENSOR_TYPE_PRESSURE;
  sensor->min_delay   = 0;
  sensor->max_value   = -156.9064F; /* -16g = 156.9064 m/s^2  */
  sensor->min_value   = 156.9064F;  /*  16g = 156.9064 m/s^2  */
  sensor->resolution  = 0.03923F;   /*  4mg = 0.0392266 m/s^2 */ 
}
