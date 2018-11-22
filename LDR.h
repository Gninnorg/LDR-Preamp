/*
  LDR
  Library for LDR resistor
  Created by Jan Abkjer Tofft, November 13, 2018.
  Released into the public domain.
*/

#ifndef LDR_h
#define LDR_h

#include "DAC7678.h"                   // This library is used for i2c control of the TI DAC7678
#include "ADS1115.h"                   // This library is for both the 12 bit ADS1015 and the 16 bit ADS1115 - we use the ADS1115: please note: there is a timing error in the library, that have to be corrected (the ESP32 is much faster than the Arduino). In Adafruit_ADS1015.h change ADS1115_CONVERSIONDELAY to 16: #define ADS1115_CONVERSIONDELAY (16)

#include "Arduino.h"
#include <mutex>

class LDR
{
  private:
    std::mutex &myi2cMutex;
    DAC7678 &mydac;
    ADS1115 &myadc;
    byte DACBiasChannel = 0;
    byte DACAdjChannel = 1;
    byte ADCchannel_LDR = 0;
    float reference_voltage = 3.3;
    unsigned int value_of_reference_resistor = 2000;
    byte samples_for_measurements = 8;
    unsigned int lastSetOutputBiasChannel = 0;
    unsigned int lastSetOutputAdjChannel = 0;
    long minimumResistance = 99999999;
    long maximumResistance = 0;

  public:
//
    LDR(std::mutex& passedi2cMutex, DAC7678& passedDAC7678, byte _DACBiasChannel, byte _DACAdjChannel, ADS1115& passedADS1115, byte _ADCchannel_LDR, float _refvoltage, unsigned int _value_of_reference_resistor, byte _samples_for_measurements);

    // Sets the LDR to minimum resistance = On
    void turnOn();

    // Sets the LDR to maximum resistance = Off
    void turnOff();

    // Set the 10K and 100K DAC channels to values
    void setDACchannels(unsigned int _10K_channel_value, unsigned int _100K_channel_value);

    //private:

    /*
    	The functions defined below will only be used during calibration. Measurements can not be done if the calibration relays have not been activated
    */

    // Set the 10K DAC channel to value
    void setDACBiasChannel(unsigned int _value);

    // Set the 100K DAC channel to value
    void setDACAdjChannel(unsigned int _value);

    // Return the resistance in Ohms
    long getResistance();

    // Measure the minimum resistance
    long measureMinimumResistance();

    // Return the minimum resistance in Ohms - for NSL32SR2 it will normally be around 45 Ohms as we do not drive the LDR with more than 7mA
    long getMinimumResistance();

    // Measure the maximum resistance
    long measureMaximumResistance();

    // Return the maximum resistance in Ohms - for NSL32SR2 it will normally be around 5 MOhms or more
    long getMaximumResistance();

    // Set the specified DAC channel to value and return the resulting resistance of the LDR after a stabilization delay has been done
    long setDACChannelAndMeasureResistance(bool _quick_calibration, byte _channel, unsigned int _value, byte _delay_duration_modifier);

    // Calculate and perform delay to let LDR stabilize during calibration
    // The delay must be longer if the bias channel has been modified and shorter if the fine adjustment channel has been modified.
    // If _delay_duration_modifier > 0 the delay time will be increased - this is used during calibration if the precision targets could not be met with the standard delay. In that case the delay will be increased to give the LDR longer time to stabilize
    long doStabilizationDelay(byte _channel, byte _delay_duration_modifier);

    // Run Exponential Search to find the DAC output needed to get the LDR-resistance to match target. with startPosition = ie. 40 the algorithm will try 40, 80, 320, 2560 or with startPostion 2 it will be 2, 4, 16, 256 etc.
    int ExponentialSearch(bool _quick_calibration, int _startPosition, int _endPosition, long _target, byte _channel, byte _retries);

    // Perform Binary Search for target by adjusting the specified channel of the DAC
    int BinarySearch(bool _quick_calibration, int _startPosition, int _endPosition, long _target, byte _channel, byte _retries);

    // Perform calibration
    int calibrateLDR(bool _quick_calibration, byte _ldr_no, bool _is_shunt);

};
#endif
