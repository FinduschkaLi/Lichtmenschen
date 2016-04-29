#ifndef ADXL345_H
#define ADXL345_H
#include <Arduino.h>
#include <SPI.h>
#include <avr/interrupt.h>
//Some register definitions were taken from Adafruit Library by K. Townsend (Adafruit Industries)
//=========================================================================
    #define GRAV                 (9.80665F) 
//=========================================================================
   // REGISTERS
//-----------------------------------------------------------------------
    #define A_R_DEVID               (0x00)    // Device ID
    #define A_R_THRESH_TAP          (0x1D)    // Tap threshold
    #define A_R_OFSX                (0x1E)    // X-axis offset
    #define A_R_OFSY                (0x1F)    // Y-axis offset
    #define A_R_OFSZ                (0x20)    // Z-axis offset
    #define A_R_DUR                 (0x21)    // Tap duration
    #define A_R_LATENT              (0x22)    // Tap latency
    #define A_R_WINDOW              (0x23)    // Tap window
    #define A_R_THRESH_ACT          (0x24)    // Activity threshold
    #define A_R_THRESH_INACT        (0x25)    // Inactivity threshold
    #define A_R_TIME_INACT          (0x26)    // Inactivity time
    #define A_R_ACT_INACT_CTL       (0x27)    // Axis enable control for activity and inactivity detection
    #define A_R_THRESH_FF           (0x28)    // Free-fall threshold
    #define A_R_TIME_FF             (0x29)    // Free-fall time
    #define A_R_TAP_AXES            (0x2A)    // Axis control for single/double tap
    #define A_R_ACT_TAP_STATUS      (0x2B)    // Source for single/double tap
    #define A_R_BW_RATE             (0x2C)    // Data rate and power mode control
    #define A_R_POWER_CTL           (0x2D)    // Power-saving features control
    #define A_R_INT_ENABLE          (0x2E)    // Interrupt enable control
    #define A_R_INT_MAP             (0x2F)    // Interrupt mapping control
    #define A_R_INT_SOURCE          (0x30)    // Source of interrupts
    #define A_R_DATA_FORMAT         (0x31)    // Data format control
    #define A_R_DATAX0              (0x32)    // X-axis data 0
    #define A_R_DATAX1              (0x33)    // X-axis data 1
    #define A_R_DATAY0              (0x34)    // Y-axis data 0
    #define A_R_DATAY1              (0x35)    // Y-axis data 1
    #define A_R_DATAZ0              (0x36)    // Z-axis data 0
    #define A_R_DATAZ1              (0x37)    // Z-axis data 1
    #define A_R_FIFO_CTL            (0x38)    // FIFO control
    #define A_R_FIFO_STATUS         (0x39)    // FIFO status
/*=========================================================================*/
	//Values for the TAB Register
	#define A_TAP_Z 				(0x01) //Z-Axis value can cause tab event
	#define A_TAP_Y 				(0x02) //Y-Axis value can cause tab event
	#define A_TAP_X 				(0x04) //X-Axis value can cause tab event
	#define A_TAB_S1D2 				(0xBF) //Map Single tab to int1 and double tab to int2
	#define A_TAB_S1D1 				(0x9F) //Map both tabs to int1
	#define A_TAB_TH_MG2LSB 		(1/62.5) //Scaling Factor threshold 62.5mg per bit
	#define A_TAB_TLW_MS2LSB 		(1/1.25) //Scaling Factor latency and window 1.25ms per bit
	#define A_TAB_TT_US2LSB 		(1/625) //Scaling Factor Tabtime 625us per bit
	#define A_TAB_SD 				(0x60) //Enable Single and double Tab
	#define A_TAB_S 				(0x40) //Enable Single Tab
	#define A_TAB_D 				(0x20) //Enable double tab
/*=========================================================================
    -----------------------------------------------------------------------*/
    #define A_LSB2MG 				(0.0039)  // 4mg per LS-bit
/*=========================================================================*/

/* Used with register 0x2C (A_R_BW_RATE) to set bandwidth */
typedef enum
{
  A_DR_3200_HZ    = 0b1111, // 1600Hz Bandwidth   140µA IDD
  A_DR_1600_HZ    = 0b1110, //  800Hz Bandwidth    90µA IDD
  A_DR_800_HZ     = 0b1101, //  400Hz Bandwidth   140µA IDD
  A_DR_400_HZ     = 0b1100, //  200Hz Bandwidth   140µA IDD
  A_DR_200_HZ     = 0b1011, //  100Hz Bandwidth   140µA IDD
  A_DR_100_HZ     = 0b1010, //   50Hz Bandwidth   140µA IDD
  A_DR_50_HZ      = 0b1001, //   25Hz Bandwidth    90µA IDD
  A_DR_25_HZ      = 0b1000, // 12.5Hz Bandwidth    60µA IDD
  A_DR_12_5_HZ    = 0b0111, // 6.25Hz Bandwidth    50µA IDD
  A_DR_6_25HZ     = 0b0110, // 3.13Hz Bandwidth    45µA IDD
  A_DR_3_13_HZ    = 0b0101, // 1.56Hz Bandwidth    40µA IDD
  A_DR_1_56_HZ    = 0b0100, // 0.78Hz Bandwidth    34µA IDD
  A_DR_0_78_HZ    = 0b0011, // 0.39Hz Bandwidth    23µA IDD
  A_DR_0_39_HZ    = 0b0010, // 0.20Hz Bandwidth    23µA IDD
  A_DR_0_20_HZ    = 0b0001, // 0.10Hz Bandwidth    23µA IDD
  A_DR_0_10_HZ    = 0b0000  // 0.05Hz Bandwidth    23µA IDD (default value)
} dataRate_t;

/* Used with register 0x31 (A_R_DATA_FORMAT) to set g range */
typedef enum
{
  A_RG_16_G          = 0b11,   // +/- 16g
  A_RG_8_G           = 0b10,   // +/- 8g
  A_RG_4_G           = 0b01,   // +/- 4g
  A_RG_2_G           = 0b00    // +/- 2g (default value)
} range_t;

class ADXL345 {
	
 public:
	
  ADXL345(uint8_t cs);
  ADXL345(uint8_t cs, uint8_t int1);
  
  bool       begin(void);
  void       setRange(range_t range);
  range_t    getRange(void);
  void       setDataRate(dataRate_t dataRate);
  dataRate_t getDataRate(void);
  
  uint8_t    getDeviceID(void);
  void       writeRegister(char registerAddress, char value);
  void       readRegister(char registerAddress, int numBytes, char * values);
  int16_t    getX(void), getY(void), getZ(void);
  
  void		 configTap(char tax,char lat, char win,char tres,char tdur);
  void 		 stopTab(void);
  void 		 startTab(void);
  void 		 attachISR( uint8_t (*fp)(void) );
  void 		 clearIntSource(void);

 private:
  static uint8_t 	 (*_fpISR)(void);
  static  void 		 callISR(void); 

  
  uint8_t 	 _cs;
  range_t 	 _range;
  char 	 	 _values[2];
  uint8_t 	 _int1;
  bool 	 	 _intenable;

  

};
#endif