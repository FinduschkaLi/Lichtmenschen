
//This is the nRF24 setup routine

#include <SPI.h>
#include "RF24.h"
//#include <inttypes.h>

//#include "ADXL345.h"
//#include "ReLight.h"

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

//This buffer will hold values read from the ADXL345 registers.
char values[10];
int val = 0; 
//These variables will be used to hold the x,y and z axis accelerometer values.
int x,y,z;//---------------------------------------


/* Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 7 & 8 */
RF24 radio(7,8);

uint32_t address[2] = {0xAACCAAAA};

//boolean timeout = false; 
uint8_t r_r_no = 0; //channel number belonging to received signal
uint8_t r_s_no = 0; //channel number belonging to sending signal
uint8_t r_dev_no = 0; //device number in chain.
uint8_t r_chainlength = 0;
unsigned long millitime; // holds a time 
unsigned long microtime; // holds a time 
uint32_t r_tout = 5000; //radio timeout in milliseconds, not constant, as this is primary value
const uint32_t r_tout2 = 20000;//set timeout to 2 minutes
const uint32_t r_tout3 = 120000;//set timeout to 2 minutes
uint32_t data; //payload
uint8_t r_PaySize = 3; //Payload Size in bytes

//radio.testCarrier() //did we get a carrier? (measurement)
void setup() {
    Serial.begin(115200); 
    Serial.println("Initialize...");
     //Pin settings
     //Led
    pinMode(5, OUTPUT);
    //Magnetometer
    pinMode(P_MAG_CS,OUTPUT);
    digitalWrite(P_MAG_CS,HIGH); 

   //RF24 
   address[1] = {0xAACC0000 | 0x0001 | 0x0001<<8};
   //address[0] = {0xAACC0000 | 0x0001 | 0x0001<<8};
  
   //Initialize RF24--------------------------------------------------------
   initRadio();
   //Listen----------------------------------------------------------------
  uint8_t retry = 0;  //needed to start

   while(retry<R_RETRY){
     Serial.print("Try ");
     Serial.print(retry+1);
     Serial.print(": ");
       if(listenForSignal(r_tout) == true) //this takes 5s to verify.
       {
          if(lowByte(data>>16) == 0xFF){
            r_dev_no = lowByte(data);
            Serial.print("Device number set to: ");
            Serial.println(r_dev_no);
            data = 0xFE0000;
            sendData(data);
            retry=R_RETRY;//end while
          }else{
            Serial.println("Signal received but not understood.");
            //->ask to resend and go back to listen
            retry++;
          }
       }else{
          radio.stopListening();
          break;
       }    
   }
   if(r_dev_no == 0){
       r_dev_no = 1;// number 2 is first sending channel (number 1 is the receiving channel of unit 1)
       r_tout = r_tout2;
       Serial.println("No Signal, starting chain...");
       Serial.print("Increased timeout to ");
       Serial.print(r_tout);
       Serial.println("ms");

   }
  // r_r_no = r_dev_no;//important to execute here to make this unit stop from listening on channel 0 (is updated by calling sendData();
    
  
   
  //Switch to ping mode and look for new-----------------------------------
  delay(100);
  data = 0xFF0000;
  data += r_dev_no+1;
  bool timeout = false;
  Serial.print("Now pinging node ");
  Serial.print(r_dev_no+1);
  Serial.println("...");
  if (sendwPayloadAck(r_tout)) {
      Serial.println("Received P-Ack.");
      //-->verify reception and listen for confirmation.
      if(r_dev_no == 1){
        //if first in line, listen on 1
        r_r_no = 1;
        Serial.print("Switch to listen on ");
        Serial.print(r_r_no);
        Serial.println(". channel");
        Serial.print("Timeout set to: ");
        Serial.println(r_tout3,DEC);
        //listening was stopped because of sending before
        radio.stopListening(); //safety measure
        radio.setChannel(r_r_no);
        radio.openReadingPipe(1,address[1]);
        if(listenForSignal(r_tout3)){
            r_chainlength = lowByte(data);
            Serial.print("Chainlength: ");
            Serial.print(r_chainlength);
            Serial.println(". Chain successfully closed. ********.");
         }else{
           Serial.println("ERROR: No Signal received. Could not close chain. #########");
           r_dev_no = 0;
           radio.stopListening();
         }

      }else{
        //all set.
        Serial.println("Device ready.");
      }
  }else{
     Serial.println("No answer found.");
     if(r_dev_no == 1){
       //go to default mode, as no partner found
       Serial.println("No partner found. Switch to default mode.");
       r_dev_no = 0;
       radio.stopListening();
     }else{
       Serial.println("Start pinging first node...");
       r_s_no = 1;
       //was sending before, listening stopped before.
       radio.stopListening(); //safety measure
       radio.openWritingPipe(address[1]); //Do not call this while actively listening. Remember to stopListening() first.
       //r_dev_no = 1; //IS THIS NEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEDED?????????????
       data = 0xFF0001;
        if (sendData(r_tout)){
           Serial.println("Successfully closed chain. *************"); 
        }else{
           Serial.println("ERROR: Could not close chain. ###############");
           r_dev_no = 0;
        }
     }
  }
 
  //Finish setup----------------------------------------------------------
  if(r_dev_no != 0){
    if (r_s_no == 0){ //if not talking to number 1
        r_s_no = r_dev_no+1;
    }
    r_r_no = r_dev_no;
    address[0] = {0xAACC0000 | (0x0000 | r_s_no)<<8 | r_s_no};
    address[1] = {0xAACC0000 | (0x0000 | r_r_no)<<8 | r_r_no};
    radio.openWritingPipe(address[0]); //Do not call this while actively listening. Remember to stopListening() first.
    radio.openReadingPipe(1,address[1]);
    Serial.print("Listening as ");
    Serial.print(r_r_no);
    Serial.print(", talking to ");
    Serial.print(r_s_no);  
    Serial.println(". Adresses: ");
    Serial.print("Receiving: ");
    Serial.println(address[1],HEX);
    Serial.print("Sending: ");
    Serial.println(address[0],HEX);
    radio.setChannel(r_r_no);
    Serial.print("Channel set to ");
    Serial.println(radio.getChannel());
    radio.startListening();
  }
     //Start ADXL-----------------------------------------------------------------------
    SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE3));
   //Put the ADXL345 into +/- 4G range by writing the value 0x01 to the DATA_FORMAT register.
  writeRegister(DATA_FORMAT, 0x01);
  //Put the ADXL345 into Measurement Mode by writing 0x08 to the POWER_CTL register.
  writeRegister(POWER_CTL, 0x08);  //Measurement mode  
  SPI.endTransaction(); 
}

void loop() {
  SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE3));
  readRegister(DATAX0, 6, values);
  SPI.endTransaction();
  x = ((int)values[1]<<8)|(int)values[0];
  //delay(100);
  Serial.println(x,DEC);
  // put your main code here, to run repeatedly:
 flashNB2(r_r_no);
 //Serial.println(r_r_no);
}
///////////////////////////////////////////////////////////////////////////////////
//--------------------------------------------------------
void initRadio(){
  radio.begin(); 
  // Set the PA Level low to prevent power supply related issues since this is a
  // getting_started sketch, and the likelihood of close proximity of the devices. RF24_PA_MAX is default.
  radio.setPALevel(RF24_PA_LOW); 
  //set default channel and address
  radio.setChannel(0);
  radio.setPayloadSize(r_PaySize);
  radio.openWritingPipe(address[0]); //Do not call this while actively listening. Remember to stopListening() first.
  radio.openReadingPipe(1,address[0]);
  radio.setAutoAck(true);
  radio.startListening();
}

bool listenForSignal(uint32_t ms){ //returns true if signal was received. Received bytes are stored in data
    radio.startListening();
    millitime = millis();
    bool timeout = false;
   Serial.print("Receiving Ch(");
   Serial.print(radio.getChannel());
   Serial.print(")...");
   while ( ! radio.available() ) {
     analogWrite(5, abs(lowByte(millis()/8)-128));
     printCounter(millis()-millitime);
     if (millis() - millitime > ms ) {       // If waited too long, indicate timeout and exit while loop
          timeout = true;
          Serial.println("Timeout");
          break;
     }
   }
   if(!timeout){
       Serial.println("OK");
   }
   return receiveData();// Get the payload
}
//Send & Receive routines----------------------------------------------------------------------------------
bool receiveData(){ //not required
  if(radio.available()){
     Serial.print("Received ");
     radio.read( &data, sizeof(data) ); 
     // Get the payload
     radio.stopListening();
     Serial.println(data,HEX);
     return true;
  }else{
    return false;
  }
}

bool sendData(uint16_t ms){
    radio.stopListening();
    radio.setChannel(r_s_no);
    Serial.print("Sending Ch(");
    Serial.print(radio.getChannel());
    Serial.print(")...");
    millitime = millis();
    bool timeout = false;
    while(! radio.write( &data, sizeof(data) )){
       
       if(ms>100){//show time progress / animate led
           printCounter(millis()-millitime);
           if(lowByte(millis()-millitime/2)<50){
             digitalWrite(5,HIGH);
           }else{
             digitalWrite(5,LOW);
           }
       }
       
       if(millis() - millitime > ms){
           Serial.println("Timeout");
           timeout = true;
           break; 
       }
       
    }
    if(timeout == false){
      Serial.println("OK");
    }
    radio.setChannel(r_r_no);
    return !timeout;
}

bool sendwPayloadAck(uint16_t ms){
  uint8_t n=0;
  while(n<=R_RETRY){
    if(sendData(ms)){
        if(listenForSignal(100)){
          if(data == 0xFE0000){
            return true;
          }
        }
        radio.stopListening();
        n++;
    }else{
      return false;
    }
    
  }
  return false;
  
}


////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////

//------------------------------------------------------------------------------

void printCounter(unsigned long tme){
  static bool stat = false;
 if(tme%1000<100){
       if(stat == false){
         Serial.print(int(tme/1000));
         Serial.print(" ");
         stat = true;
       }
     }else{
       stat = false;
  }
}
//--------------------------------------------------------------------
void flashNB2(uint8_t nb){
  static uint16_t cnt =0; //if Number increases above 16 should be set to uint16_t
  static uint16_t lastt;
  if(abs(uint16_t(millis())-lastt) > 40){
    cnt++;
    cnt=cnt%((nb+10)*5);
    lastt = millis();
  }
  if(nb > 0){
    if (cnt<nb*5 && cnt%5 < 1){
      digitalWrite(5,HIGH);
    }else{
      digitalWrite(5,LOW);
    }
  }else{
    digitalWrite(5,HIGH);
  }
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
