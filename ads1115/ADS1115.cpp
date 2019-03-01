/**************************************************************************/
/*!
    @file     ADS1115.cpp
    @author   Carsten Grønning
    @license  BSD (see license.txt)

    Inspired by the Adafruit_ADS1015 library

    Driver for the ADS1115 ADC

    @section  HISTORY

    v1.0 - First release
*/

/**************************************************************************/
#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <Wire.h>
#include "ADS1115.h"


/**************************************************************************
    Instantiates a new ADS1115 class with an ADS1115 address and a
    selected programable gain.
    Default address: ADDRESS_GND
    Default PGA: PGA_6_144V
/**************************************************************************/
ADS1115::ADS1115(ads1115_I2C_address_t i2cAddress, ads1115_conf_pga_t config_pga, ads1115_conf_mode_t config_mode)
{
   m_i2cAddress = i2cAddress;
   m_config_pga = config_pga;
   m_config_mode = config_mode;
}


/**************************************************************************
    Start Wire for for i2c communication
/**************************************************************************/
void ADS1115::begin()
{
  Wire.begin();
  if (m_config_mode == DEV_MODE_CONTIN) {
    // If device mode is continuous reading ask ADS1115 to start reading
    triggerContReading();
  }
}

/**************************************************************************
    Gets a single-ended ADC reading from AIN0 to GND
/**************************************************************************/
int16_t ADS1115::getCountA0Gnd()
{
  setMultiplexer(MUX_SINGLE_0);
  return getConversion();
}

/**************************************************************************
    Gets a single-ended ADC reading from AIN1 to GND
/**************************************************************************/
int16_t ADS1115::getCountA1Gnd()
{
  setMultiplexer(MUX_SINGLE_1);
  return getConversion();
}

/**************************************************************************
    Gets a single-ended ADC reading from AIN2 to GND
/**************************************************************************/
int16_t ADS1115::getCountA2Gnd()
{
  setMultiplexer(MUX_SINGLE_2);
  return getConversion();
}

/**************************************************************************
    Gets a single-ended ADC reading from AIN3 to GND
/**************************************************************************/
int16_t ADS1115::getCountA3Gnd()
{
  setMultiplexer(MUX_SINGLE_3);
  return getConversion();
}

/**************************************************************************
    Gets a differential ADC reading from AIN0 to AIN1
/**************************************************************************/
int16_t ADS1115::getCountA0A1()
{
  setMultiplexer(MUX_DIFF_0_1);
  return getConversion();
}

/**************************************************************************
    Gets a differential ADC reading from AIN0 to AIN3
/**************************************************************************/
int16_t ADS1115::getCountA0A3()
{
  setMultiplexer(MUX_DIFF_0_3);
  return getConversion();
}

/**************************************************************************
    Gets a differential ADC reading from AIN1 to AIN3
/**************************************************************************/
int16_t ADS1115::getCountA1A3()
{
  setMultiplexer(MUX_DIFF_1_3);
  return getConversion();
}

/**************************************************************************
    Gets a differential ADC reading from AIN1 to AIN3
/**************************************************************************/
int16_t ADS1115::getCountA2A3()
{
  setMultiplexer(MUX_DIFF_2_3);
  return getConversion();
}

/**************************************************************************
    Trigger a conversion and wait for conversion to complete
/**************************************************************************/
int16_t ADS1115::getConversion()
{

  if (m_config_mode == DEV_MODE_SINGLE) {
    triggerConversion();
    if(pollConversion((uint8_t)10000)) {
      return getLastConversionResults();
    }
  } else {
    return getLastConversionResults();
  }
}

/**************************************************************************
    Utility to get millivolt pr. ADC count for at the configured PGA
/**************************************************************************/
float ADS1115::getMvPerCount()
{
  switch (m_config_pga)
  {
    case (PGA_6_144V):
      return 0.187500;
      break;
    case (PGA_4_096V):
      return 0.125000;
      break;
    case (PGA_2_048V):
      return 0.062500;
      break;
    case (PGA_1_024V):
      return 0.031250;
      break;
    case (PGA_0_512V):
      return 0.015625;
      break;
    case (PGA_0_256V):
      return 0.007813;
      break;
  }
}

/**************************************************************************
    Utility to convert ADC count to Volt
/**************************************************************************/
float ADS1115::convertCountToVolt(int16_t count)
{
  return count * getMvPerCount() / 1000;
}

/**************************************************************************
    Trigger til ADC1115 to do a single shot reading
/**************************************************************************/
void ADS1115::triggerConversion()
{
  uint16_t config = getConfiguration();
  config |= OS_SET_SINGLE_CONV;
  writeRegister(REG_CONFIG, config);
}

/**************************************************************************
    Trigger til ADC1115 to do a continuous reading
/**************************************************************************/
void ADS1115::triggerContReading()
{
  uint16_t config = getConfiguration();
  config |= OS_SET_NO_EFFECT;
  writeRegister(REG_CONFIG, config);
}

/**************************************************************************
    Check if the ADC1115 has finished the conversion
/**************************************************************************/
boolean ADS1115::isConversionReady()
{
  return readRegister(REG_CONFIG)>>15;
}

/**************************************************************************
    Wait for conversion to finish by retrying to check if conversion is
    ready. Function will continue to check until max_retries is reached.
/**************************************************************************/
boolean ADS1115::pollConversion(uint8_t max_retries)
{
  for(uint16_t i = 0; i < max_retries; i++) {
    if (isConversionReady()) return true;
  }
  return false;
}

/**************************************************************************
    Gets a last conversion from the ADC1115
/**************************************************************************/
int16_t ADS1115::getLastConversionResults()
{
  return (int16_t) readRegister(REG_CONVERSION);
}

/**************************************************************************
    Sets the datarate for the ADC1115 -
    the setting only affect continuous readings
/**************************************************************************/
void ADS1115::setDataRate(ads1115_conf_dr_t config_dr)
{
  m_config_dr = config_dr;
}

/**************************************************************************
    Sets the selected input settings for the ADS1115
/**************************************************************************/
void ADS1115::setMultiplexer(ads1115_conf_mux_t config_mux)
{

  if (m_config_mux != config_mux) {
    // Only change mux if setting has changed
    m_config_mux = config_mux;

    if (m_config_mode == DEV_MODE_CONTIN) {
      // If ADS1115 is continuous mode write the changed mux to the device
      triggerContReading();
      // Wait a bit to be sure we are not reading a conversion on a different mux
      delay(300);
    }
  }

}

/**************************************************************************
    Builds and returns a 16 bit setting - used for setting up the ADS1115
/**************************************************************************/
uint16_t ADS1115::getConfiguration()
{
  uint16_t config = m_config_os     |
                    m_config_mux    |
                    m_config_pga    |
                    m_config_mode   |
                    m_config_dr     |
                    m_config_cmode  |
                    m_config_cpol   |
                    m_config_clat   |
                    m_config_cque;
  return config;
}

/**************************************************************************
    Gets a single-ended ADC reading from AIN0 to GND
/**************************************************************************/
uint16_t ADS1115::readRegister(ads1115_reg_pointer_t reg)
{
  Wire.beginTransmission(m_i2cAddress);
  // Select the register from which you want to read from
  Wire.write((uint8_t)reg);
  Wire.endTransmission();

  // Read 2 bytes form register and return the register
  Wire.requestFrom(m_i2cAddress, (uint8_t)2);
  return ((Wire.read() << 8) | Wire.read());
}

/**************************************************************************
    Write 16 bit to a selected ADC1115 register
**************************************************************************/
void ADS1115::writeRegister(ads1115_reg_pointer_t reg, uint16_t value)
{
  Wire.beginTransmission(m_i2cAddress);
  Wire.write((uint8_t)reg);
  Wire.write((uint8_t)(value>>8));
  Wire.write((uint8_t)(value & 0xFF));
  Wire.endTransmission();
}

/**************************************************************************
    Utility to calculate R2 (Ohm) in a voltage divider from a given input,
    output and R1
/**************************************************************************/
uint16_t  ADS1115::getR2ValueVoltageDivider(float input, float output, uint16_t r1){
  return (uint16_t)((r1 / (1 - (output / input)))-r1);
}

/**************************************************************************
    Utility to calculate R1 (Ohm) in a voltage divider from a given input,
    output and R2
/**************************************************************************/
uint16_t  ADS1115::getR1ValueVoltageDivider(float input, float output, uint16_t r2){
  return (uint16_t)((r2 / ((output / input)))-r2);
}

/**************************************************************************
    Utility to calculate shunt value for a pontentiometer of a given size
    by providing af pot_size of 1 the function will return the series
    relative value in percent of a given pontentiometer
/**************************************************************************/
float ADS1115::getShuntForAttenuation(double attenuation, uint16_t pot_size)
{
  if (attenuation < 0) {
    return pow(10,(attenuation/20)) * pot_size;
  } else {
    return 0;
  }
}

/**************************************************************************
    Utility to calculate shunt value for a pontentiometer of a given size
    by providing af pot_size of 1 the function will return the shunts
    relative value in percent of a given pontentiometer
/**************************************************************************/
float ADS1115::getSerieForAttenuation(double attenuation, uint16_t pot_size )
{
  if (attenuation < 0) {
    return (1 - pow(10,(attenuation/20))) * pot_size;
  } else {
    return 0;
  }
}

/**************************************************************************
    Utility to output the current settings of the ADC1115 object to the
    Serial object
/**************************************************************************/
void ADS1115::showConfigRegister()
{
    //#ifdef ADS1115_SERIAL_DEBUG
    uint16_t configRegister = getConfiguration();
    Serial.print("Configuration file is:");
    Serial.println(configRegister,BIN);

    configRegister = readRegister(REG_CONFIG);
    Serial.print("Device configuration file is:");
    Serial.println(configRegister,BIN);
    //#endif
}

/**************************************************************************/
/*!
    @brief  Reads the conversion results, measuring the voltage
            difference between the P (AIN0) and N (AIN1) input.  Generates
            a signed value since the difference can be either
            positive or negative.
*/
/**************************************************************************

int16_t ADS1115::readADC_Differential_0_1() {
  // Start with default values
  uint16_t config = ADS1115_REG_CONFIG_CQUE_NONE    | // Disable the comparator (default val)
                    ADS1115_REG_CONFIG_CLAT_NONLAT  | // Non-latching (default val)
                    ADS1115_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
                    ADS1115_REG_CONFIG_CMODE_TRAD   | // Traditional comparator (default val)
                    ADS1115_REG_CONFIG_DR_128SPS    | // 1600 samples per second (default)
                    ADS1115_REG_CONFIG_MODE_SINGLE;   // Single-shot mode (default)

  // Set PGA/voltage range
  config |= m_gain;

  // Set channels
  config |= ADS1115_REG_CONFIG_MUX_DIFF_0_1;          // AIN0 = P, AIN1 = N

  // Set 'start single-conversion' bit
  config |= ADS1115_REG_CONFIG_OS_SINGLE;

  // Write config register to the ADC
  writeRegister(m_i2cAddress, ADS1115_REG_POINTER_CONFIG, config);

  // Wait for the conversion to complete
  delay(m_conversionDelay);

  // Read the conversion results
  uint16_t res = readRegister(m_i2cAddress, ADS1115_REG_POINTER_CONVERT) >> m_bitShift;
  if (m_bitShift == 0)
  {
    return (int16_t)res;
  }
  else
  {
    // Shift 12-bit results right 4 bits for the ADS1115,
    // making sure we keep the sign bit intact
    if (res > 0x07FF)
    {
      // negative number - extend the sign to 16th bit
      res |= 0xF000;
    }
    return (int16_t)res;
  }
} */

/**************************************************************************/
/*!
    @brief  Reads the conversion results, measuring the voltage
            difference between the P (AIN2) and N (AIN3) input.  Generates
            a signed value since the difference can be either
            positive or negative.
*/
/**************************************************************************
int16_t ADS1115::readADC_Differential_2_3() {
  // Start with default values
  uint16_t config = ADS1115_REG_CONFIG_CQUE_NONE    | // Disable the comparator (default val)
                    ADS1115_REG_CONFIG_CLAT_NONLAT  | // Non-latching (default val)
                    ADS1115_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
                    ADS1115_REG_CONFIG_CMODE_TRAD   | // Traditional comparator (default val)
                    ADS1115_REG_CONFIG_DR_128SPS    | // 1600 samples per second (default)
                    ADS1115_REG_CONFIG_MODE_SINGLE;   // Single-shot mode (default)

  // Set PGA/voltage range
  config |= m_gain;

  // Set channels
  config |= ADS1115_REG_CONFIG_MUX_DIFF_2_3;          // AIN2 = P, AIN3 = N

  // Set 'start single-conversion' bit
  config |= ADS1115_REG_CONFIG_OS_SINGLE;

  // Write config register to the ADC
  writeRegister(m_i2cAddress, ADS1115_REG_POINTER_CONFIG, config);

  // Wait for the conversion to complete
  delay(m_conversionDelay);

  // Read the conversion results
  uint16_t res = readRegister(m_i2cAddress, ADS1115_REG_POINTER_CONVERT) >> m_bitShift;
  if (m_bitShift == 0)
  {
    return (int16_t)res;
  }
  else
  {
    // Shift 12-bit results right 4 bits for the ADS1115,
    // making sure we keep the sign bit intact
    if (res > 0x07FF)
    {
      // negative number - extend the sign to 16th bit
      res |= 0xF000;
    }
    return (int16_t)res;
  }
}
*/

/**************************************************************************/
/*!
    @brief  Sets up the comparator to operate in basic mode, causing the
            ALERT/RDY pin to assert (go from high to low) when the ADC
            value exceeds the specified threshold.

            This will also set the ADC in continuous conversion mode.
*/
/**************************************************************************
void ADS1115::startComparator_SingleEnded(uint8_t channel, int16_t threshold)
{
  // Start with default values
  uint16_t config = ADS1115_REG_CONFIG_CQUE_1CONV   | // Comparator enabled and asserts on 1 match
                    ADS1115_REG_CONFIG_CLAT_LATCH   | // Latching mode
                    ADS1115_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
                    ADS1115_REG_CONFIG_CMODE_TRAD   | // Traditional comparator (default val)
                    ADS1115_REG_CONFIG_DR_128SPS    | // 1600 samples per second (default)
                    ADS1115_REG_CONFIG_MODE_CONTIN  | // Continuous conversion mode
                    ADS1115_REG_CONFIG_MODE_CONTIN;   // Continuous conversion mode

  // Set PGA/voltage range
  config |= m_gain;

  // Set single-ended input channel
  switch (channel)
  {
    case (0):
      config |= ADS1115_REG_CONFIG_MUX_SINGLE_0;
      break;
    case (1):
      config |= ADS1115_REG_CONFIG_MUX_SINGLE_1;
      break;
    case (2):
      config |= ADS1115_REG_CONFIG_MUX_SINGLE_2;
      break;
    case (3):
      config |= ADS1115_REG_CONFIG_MUX_SINGLE_3;
      break;
  }

  // Set the high threshold register
  // Shift 12-bit results left 4 bits for the ADS1115
  writeRegister(m_i2cAddress, ADS1115_REG_POINTER_HITHRESH, threshold << m_bitShift);

  // Write config register to the ADC
  writeRegister(m_i2cAddress, ADS1115_REG_POINTER_CONFIG, config);
}
*/

/**************************************************************************/
/*!
    @brief  In order to clear the comparator, we need to read the
            conversion results.  This function reads the last conversion
            results without changing the config value.
*/
/**************************************************************************
int16_t ADS1115::getLastConversionResults()
{
  // Wait for the conversion to complete
  delay(m_conversionDelay);

  // Read the conversion results
  uint16_t res = readRegister(m_i2cAddress, ADS1115_REG_POINTER_CONVERT) >> m_bitShift;
  if (m_bitShift == 0)
  {
    return (int16_t)res;
  }
  else
  {
    // Shift 12-bit results right 4 bits for the ADS1115,
    // making sure we keep the sign bit intact
    if (res > 0x07FF)
    {
      // negative number - extend the sign to 16th bit
      res |= 0xF000;
    }
    return (int16_t)res;
  }
}
*/
