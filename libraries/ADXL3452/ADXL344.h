#ifndef ADXL344_H
#define ADXL344_H
#include <Arduino.h>
#include <SPI.h>
//=========================================================================
    #define SENSORS_GRAVITY                 (9.80665F) 
//=========================================================================
   // REGISTERS
//-----------------------------------------------------------------------
    #define ADXL344_REG_DEVID               (0x00)    // Device ID
    #define ADXL344_REG_THRESH_TAP          (0x1D)    // Tap threshold
    #define ADXL344_REG_OFSX                (0x1E)    // X-axis offset
    #define ADXL344_REG_OFSY                (0x1F)    // Y-axis offset
    #define ADXL344_REG_OFSZ                (0x20)    // Z-axis offset
    #define ADXL344_REG_DUR                 (0x21)    // Tap duration
    #define ADXL344_REG_LATENT              (0x22)    // Tap latency
    #define ADXL344_REG_WINDOW              (0x23)    // Tap window
    #define ADXL344_REG_THRESH_ACT          (0x24)    // Activity threshold
    #define ADXL344_REG_THRESH_INACT        (0x25)    // Inactivity threshold
    #define ADXL344_REG_TIME_INACT          (0x26)    // Inactivity time
    #define ADXL344_REG_ACT_INACT_CTL       (0x27)    // Axis enable control for activity and inactivity detection
    #define ADXL344_REG_THRESH_FF           (0x28)    // Free-fall threshold
    #define ADXL344_REG_TIME_FF             (0x29)    // Free-fall time
    #define ADXL344_REG_TAP_AXES            (0x2A)    // Axis control for single/double tap
    #define ADXL344_REG_ACT_TAP_STATUS      (0x2B)    // Source for single/double tap
    #define ADXL344_REG_BW_RATE             (0x2C)    // Data rate and power mode control
    #define ADXL344_REG_POWER_CTL           (0x2D)    // Power-saving features control
    #define ADXL344_REG_INT_ENABLE          (0x2E)    // Interrupt enable control
    #define ADXL344_REG_INT_MAP             (0x2F)    // Interrupt mapping control
    #define ADXL344_REG_INT_SOURCE          (0x30)    // Source of interrupts
    #define ADXL344_REG_DATA_FORMAT         (0x31)    // Data format control
    #define ADXL344_REG_DATAX0              (0x32)    // X-axis data 0
    #define ADXL344_REG_DATAX1              (0x33)    // X-axis data 1
    #define ADXL344_REG_DATAY0              (0x34)    // Y-axis data 0
    #define ADXL344_REG_DATAY1              (0x35)    // Y-axis data 1
    #define ADXL344_REG_DATAZ0              (0x36)    // Z-axis data 0
    #define ADXL344_REG_DATAZ1              (0x37)    // Z-axis data 1
    #define ADXL344_REG_FIFO_CTL            (0x38)    // FIFO control
    #define ADXL344_REG_FIFO_STATUS         (0x39)    // FIFO status
/*=========================================================================*/

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
    #define ADXL344_MG2G_MULTIPLIER (0.0039)  // 4mg per bit
/*=========================================================================*/

/* Used with register 0x2C (ADXL344_REG_BW_RATE) to set bandwidth */
typedef enum
{
  ADXL344_DATARATE_3200_HZ    = 0b1111, // 1600Hz Bandwidth   140�A IDD
  ADXL344_DATARATE_1600_HZ    = 0b1110, //  800Hz Bandwidth    90�A IDD
  ADXL344_DATARATE_800_HZ     = 0b1101, //  400Hz Bandwidth   140�A IDD
  ADXL344_DATARATE_400_HZ     = 0b1100, //  200Hz Bandwidth   140�A IDD
  ADXL344_DATARATE_200_HZ     = 0b1011, //  100Hz Bandwidth   140�A IDD
  ADXL344_DATARATE_100_HZ     = 0b1010, //   50Hz Bandwidth   140�A IDD
  ADXL344_DATARATE_50_HZ      = 0b1001, //   25Hz Bandwidth    90�A IDD
  ADXL344_DATARATE_25_HZ      = 0b1000, // 12.5Hz Bandwidth    60�A IDD
  ADXL344_DATARATE_12_5_HZ    = 0b0111, // 6.25Hz Bandwidth    50�A IDD
  ADXL344_DATARATE_6_25HZ     = 0b0110, // 3.13Hz Bandwidth    45�A IDD
  ADXL344_DATARATE_3_13_HZ    = 0b0101, // 1.56Hz Bandwidth    40�A IDD
  ADXL344_DATARATE_1_56_HZ    = 0b0100, // 0.78Hz Bandwidth    34�A IDD
  ADXL344_DATARATE_0_78_HZ    = 0b0011, // 0.39Hz Bandwidth    23�A IDD
  ADXL344_DATARATE_0_39_HZ    = 0b0010, // 0.20Hz Bandwidth    23�A IDD
  ADXL344_DATARATE_0_20_HZ    = 0b0001, // 0.10Hz Bandwidth    23�A IDD
  ADXL344_DATARATE_0_10_HZ    = 0b0000  // 0.05Hz Bandwidth    23�A IDD (default value)
} dataRate_t;

/* Used with register 0x31 (ADXL344_REG_DATA_FORMAT) to set g range */
typedef enum
{
  ADXL344_RANGE_16_G          = 0b11,   // +/- 16g
  ADXL344_RANGE_8_G           = 0b10,   // +/- 8g
  ADXL344_RANGE_4_G           = 0b01,   // +/- 4g
  ADXL344_RANGE_2_G           = 0b00    // +/- 2g (default value)
} range_t;

class ADXL344 {
 public:
  ADXL344(uint8_t cs);
  
  bool       begin(void);
  void       setRange(range_t range);
  range_t    getRange(void);
  void       setDataRate(dataRate_t dataRate);
  dataRate_t getDataRate(void);

  uint8_t    getDeviceID(void);
  void       writeRegister(uint8_t registerAddress, char value);
  void       readRegister(uint8_t registerAddress, int numBytes, char * values);
  int16_t    getX(void), getY(void), getZ(void);
 private:
  uint8_t _cs;
  range_t _range;
};
#endif