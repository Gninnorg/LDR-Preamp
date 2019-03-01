/**************************************************************************/
/*!
    @file     ADS1115.h
    @author   Carsten Grønning
    @license  BSD (see license.txt)

    @section  HISTORY

*/
/**************************************************************************/

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <Wire.h>



// I2C ADDRESS/BITS
//-----------------------------------------------------------------------------
typedef enum
{
    ADDRESS_GND     = 0x48,   // 1001 000 (ADDR = GND)
    ADDRESS_VDD     = 0x49,   // 1001 001 (ADDR = VDD)
    ADDRESS_SDA     = 0x4A,   // 1001 010 (ADDR = SDA)
    ADDRESS_SCL     = 0x4B,   // 1001 011 (ADDR = VDD)
    ADDRESS_DEFAULT = 0x48
} ads1115_I2C_address_t;


// Pointer register
// ----------------------------------------------------------------------------
typedef enum
{
    REG_CONVERSION  = 0x00,    // 0000 0000 = Conversion register
    REG_CONFIG      = 0x01,    // 0000 0001 = Config register
    REG_LOWTHRESH   = 0x02,    // 0000 0010 = Lo_thresh register
    REG_HITHRESH    = 0x03     // 0000 0011 = Hi_tresh register
} ads1115_reg_pointer_t;

// Operational status or single-shot conversion start
// ----------------------------------------------------------------------------
typedef enum
{
    OS_SET_NO_EFFECT      = 0x0000,    // Write 0 = No effect
    OS_SET_SINGLE_CONV    = 0x8000,    // Write: 1 = Start a single conversion
    OS_GET_BUSY           = 0x0000,    // Read:  0 = Conversion is performed
    OS_GET_NOT_BUSY       = 0x8000     // Read:  1 = Conversion is not currently performed
} ads1115_conf_os_t;

// Input multiplexer configuration
// ----------------------------------------------------------------------------
typedef enum
{
  MUX_DIFF_0_1 = 0x0000,    // 000 Differential P = AIN0, N = AIN1 (default)
  MUX_DIFF_0_3 = 0x1000,    // 001 Differential P = AIN0, N = AIN3
  MUX_DIFF_1_3 = 0x2000,    // 010 Differential P = AIN1, N = AIN3
  MUX_DIFF_2_3 = 0x3000,    // 011 Differential P = AIN2, N = AIN3
  MUX_SINGLE_0 = 0x4000,    // 100 Single-ended AIN0
  MUX_SINGLE_1 = 0x5000,    // 101 Single-ended AIN1
  MUX_SINGLE_2 = 0x6000,    // 110 Single-ended AIN2
  MUX_SINGLE_3 = 0x7000     // 111 Single-ended AIN3
} ads1115_conf_mux_t;

// Programmable gain amplifier configuration
// ----------------------------------------------------------------------------
typedef enum
{
  PGA_6_144V = 0x0000,  // 000 +/-6.144V range = Gain 2/3
  PGA_4_096V = 0x0200,  // 001 +/-4.096V range = Gain 1
  PGA_2_048V = 0x0400,  // 010 +/-2.048V range = Gain 2
  PGA_1_024V = 0x0600,  // 011 +/-1.024V range = Gain 4
  PGA_0_512V = 0x0800,  // 100 +/-0.512V range = Gain 8
  PGA_0_256V = 0x0A00   // 101 +/-0.256V range = Gain 16
} ads1115_conf_pga_t;


// Device operating mode
// ----------------------------------------------------------------------------
typedef enum
{
   DEV_MODE_CONTIN = 0x0000,
   DEV_MODE_SINGLE = 0x0100
} ads1115_conf_mode_t;

// Data rate
// ----------------------------------------------------------------------------
typedef enum
{
  DR_8SPS   =  0x0000,  // 000   8 samples per second
  DR_16SPS  =  0x0020,  // 001  16 samples per second
  DR_32SPS  =  0x0040,  // 010  32 samples per second
  DR_64SPS  =  0x0060,  // 011  64 samples per second
  DR_128SPS =  0x0080,  // 100 128 samples per second (default)
  DR_250SPS =  0x00A0,  // 101 250 samples per second
  DR_475SPS =  0x00C0,  // 110 475 samples per second
  DR_860SPS =  0x00E0   // 111 860 samples per second
} ads1115_conf_dr_t;


// Comparator mode
// ----------------------------------------------------------------------------
typedef enum
{
  CMODE_TRAD    = 0x0000,   // 0 Traditional comparator with hysteresis (default)
  CMODE_WINDOW  = 0x0010    // 1 Window comparator
} ads1115_conf_comp_mode_t;

// Comparator polarity
// ----------------------------------------------------------------------------
typedef enum
{
  CPOL_ACTIVE_LOW = 0x0000,  // ALERT/RDY pin is low when active (default)
  CPOL_ACTVIVE_HI = 0x0008   // ALERT/RDY pin is high when active

} ads1115_conf_comp_pol_t;

// Latching comparator
// ----------------------------------------------------------------------------
typedef enum
{
  CLAT_NONLAT = 0x0000,  // Non-latching comparator (default)
  CLAT_LATCH  = 0x0004   // Latching comparator
} ads1115_conf_comp_lat_t;

// Comparator queue and disable
// ----------------------------------------------------------------------------
typedef enum
{
  CQUE_1CONV    = 0x0000,   // Assert ALERT/RDY after one conversions
  CQUE_2CONV    = 0x0001,   // Assert ALERT/RDY after two conversions
  CQUE_4CONV    = 0x0002,   // Assert ALERT/RDY after four conversions
  CQUE_DISABLE  = 0x0003    // Disable the comparator and put ALERT/RDY in high state (default)
} ads1115_conf_comp_que_t;




class ADS1115
{
protected:
   // Configuration
   ads1115_I2C_address_t    m_i2cAddress;
   ads1115_conf_os_t        m_config_os     = OS_SET_NO_EFFECT;
   ads1115_conf_mux_t       m_config_mux    = MUX_SINGLE_0;
   ads1115_conf_pga_t       m_config_pga;
   ads1115_conf_mode_t      m_config_mode;
   ads1115_conf_dr_t        m_config_dr     = DR_250SPS;
   ads1115_conf_comp_mode_t m_config_cmode  = CMODE_TRAD;
   ads1115_conf_comp_pol_t  m_config_cpol   = CPOL_ACTIVE_LOW;
   ads1115_conf_comp_lat_t  m_config_clat   = CLAT_NONLAT;
   ads1115_conf_comp_que_t  m_config_cque   = CQUE_DISABLE;

 public:
  // Addresses for global use
  static const ads1115_I2C_address_t ADD_GND = ADDRESS_GND;
  static const ads1115_I2C_address_t ADD_VDD = ADDRESS_VDD;
  static const ads1115_I2C_address_t ADD_SCL = ADDRESS_SCL;
  static const ads1115_I2C_address_t ADD_SDA = ADDRESS_SDA;
/*
** Set data rate
*/
  // Programmable Gain Modes for global use
  static const ads1115_conf_pga_t GAIN_6_144V = PGA_6_144V;
  static const ads1115_conf_pga_t GAIN_4_096V = PGA_4_096V;
  static const ads1115_conf_pga_t GAIN_2_048V = PGA_2_048V;
  static const ads1115_conf_pga_t GAIN_1_024V = PGA_1_024V;
  static const ads1115_conf_pga_t GAIN_0_512V = PGA_0_512V;
  static const ads1115_conf_pga_t GAIN_0_256V = PGA_0_256V;

  // Device modes for global use
  static const ads1115_conf_mode_t SINGLESHOT = DEV_MODE_SINGLE;
  static const ads1115_conf_mode_t CONTINUES  = DEV_MODE_CONTIN;

  ADS1115(ads1115_I2C_address_t i2cAddress = ADDRESS_DEFAULT, ads1115_conf_pga_t config_pga = PGA_6_144V, ads1115_conf_mode_t config_mode = DEV_MODE_SINGLE);

  /* Main functions */
  void      begin(void);
  int16_t   getCountA0Gnd();
  int16_t   getCountA1Gnd();
  int16_t   getCountA2Gnd();
  int16_t   getCountA3Gnd();
  int16_t   getCountA0A1();
  int16_t   getCountA0A3();
  int16_t   getCountA1A3();
  int16_t   getCountA2A3();
  float     getMvPerCount();

  //Utility functions
  float     convertCountToVolt(int16_t count);
  void      showConfigRegister();
  uint16_t  getR2ValueVoltageDivider(float input, float output, uint16_t r1);
  uint16_t  getR1ValueVoltageDivider(float input, float output, uint16_t r2);
  float     getShuntForAttenuation(double attenuation, uint16_t pot_size );
  float     getSerieForAttenuation(double attenuation, uint16_t pot_size );


 private:
  void      setMultiplexer(ads1115_conf_mux_t config_mux);
  void      setDataRate(ads1115_conf_dr_t config_dr);
  uint16_t  readRegister(ads1115_reg_pointer_t reg);
  void      writeRegister(ads1115_reg_pointer_t reg, uint16_t value);
  int16_t   getConversion();
  void      triggerConversion();
  boolean   pollConversion(uint8_t max_retries);
  boolean   isConversionReady();
  int16_t   getLastConversionResults();
  uint16_t  getConfiguration();
  void      triggerContReading();
};
