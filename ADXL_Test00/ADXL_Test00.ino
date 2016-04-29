#include <SPI.h>
char values[10];
int x,y,z;
//Pin Settings----------------------------------------------
#define P_MAG_CS 10
//Magnetometer ADXL345 specific-----------------------------
#define POWER_CTL 0x2D    //Power Control Register
#define DATA_FORMAT 0x31
#define DATAX0 0x32   //X-Axis Data 0
#define DATAX1 0x33   //X-Axis Data 1
#define DATAY0 0x34   //Y-Axis Data 0
#define DATAY1 0x35   //Y-Axis Data 1
#define DATAZ0 0x36   //Z-Axis Data 0
#define DATAZ1 0x37   //Z-Axis Data 1
//RF24 Settings
#define R_RETRY 1

void setup() {
  // put your setup code here, to run once:
     //Start ADXL-----------------------------------------------------------------------
    SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE3));
   //Put the ADXL345 into +/- 4G range by writing the value 0x01 to the DATA_FORMAT register.
  writeRegister(DATA_FORMAT, 0x01);
  //Put the ADXL345 into Measurement Mode by writing 0x08 to the POWER_CTL register.
  writeRegister(POWER_CTL, 0x08);  //Measurement mode  
  SPI.endTransaction(); 
  pinMode(3,OUTPUT);
  Serial.begin(115200);
}




void loop() {
  SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE3));
  readRegister(DATAX0, 2, values);
  SPI.endTransaction();
  x = ((int)values[1]<<8)|(int)values[0];
  analogWrite(3,values[1]);
  Serial.println((int)values[0],DEC);
}










//------------ADXL
void writeRegister(char registerAddress, char value){
  //Set Chip Select pin low to signal the beginning of an SPI packet.
  digitalWrite(P_MAG_CS, LOW);
  //Transfer the register address over SPI.
  SPI.transfer(registerAddress);
  //Transfer the desired register value over SPI.
  SPI.transfer(value);
  //Set the Chip Select pin high to signal the end of an SPI packet.
  digitalWrite(P_MAG_CS, HIGH);
}

//This function will read a certain number of registers starting from a specified address and store their values in a buffer.
//Parameters:
//  char registerAddress - The register addresse to start the read sequence from.
//  int numBytes - The number of registers that should be read.
//  char * values - A pointer to a buffer where the results of the operation should be stored.
void readRegister(char registerAddress, int numBytes, char * values){
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
