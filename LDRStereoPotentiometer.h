/*
  LDRStereoPotentiometer -
  Library for a LDR based stereo potentiometer

  Created by Jan Abkjer Tofft, November 18, 2018.
  Released into the public domain.
*/

#ifndef LDRStereoPotentiometer_h
#define LDRStereoPotentiometer_h

#include "Arduino.h"

#include "LDR.h"
#include "LDRPotentiometer.h"
#include "Wire.h"
#include <thread>
#include <mutex>

// This library is used for i2c control of the TI DAC7678
#include "DAC7678.h"
// This library is for the 16 bit ADS1115. Adapted from the Adafruit_ADS1015 library. Please note: a timing error in the Adafruit library have been corrected (the ESP32 is much faster than the Arduino): ADS1115_CONVERSIONDELAY have been changed from 8 to 16 in the line #define ADS1115_CONVERSIONDELAY (16)
#include "ADS1115.h"

// Define level of debug information: 0 = no debug infomation, 1 = basic debug information, 2 = verbose debug information
#define DEBUG 1

// Exact resistance of precision 2K resistor for each LDR
#define L_SER_RES 2000
#define L_SHU_RES 2000
#define R_SER_RES 2000
#define L_SHU_RES 2000

// Measured reference voltage with decimal(s)
#define REFERENCE_VOLTAGE 3.3

// DO NOT CHANGE BELOW THIS LINE ----------------------------------------------------------------------------------------------------

// Address of the DAC7678
#define DAC7678ADDRESS 0x4C

// Address of the ADS1115
#define ADS1115ADDRESS 0x48

// Samples per measurement (1 = single measurement, 2 = 2x oversampling, 3 = 3x oversampling etc.
#define SAMPLES_PER_MEASUREMENT 8

// Numbers of the LDRs in array
#define L_SER 0
#define L_SHU 1
#define R_SER 2
#define R_SHU 3

// LDR TYPES
#define SERIES 0
#define SHUNT 1

// DAC channel definitions for each LDR
#define L_SER_DAC_BIAS 0
#define L_SER_DAC_ADJ  1
#define L_SHU_DAC_BIAS 2
#define L_SHU_DAC_ADJ  3
#define R_SER_DAC_BIAS 4
#define R_SER_DAC_ADJ  5
#define R_SHU_DAC_BIAS 6
#define R_SHU_DAC_ADJ  7

// ADC channel for measurements during calibration
#define L_SER_ADC 1 // <- change back to 0
#define L_SHU_ADC 1
#define R_SER_ADC 2
#define L_SHU_ADC 3

// Definition of calibration speed types, quick is quicker than accurate but not as accurate ;-)
#define QUICK true
#define ACCURATE false

class LDRStereoPotentiometer
{
  public:
    LDRStereoPotentiometer();

    // Setters and getters for initializing the stereo potentiometer
    void setSize(long _size);
    long getSize();

    void setSteps(byte _num);
    byte getSteps();

    void setType(byte _type);
    byte getType();

    // Do we also need setter/getter for max_attenuation??

    void setInputRes(long _inputResAmp);
    long getInputRes();

    void setCalibrationSpeed(bool _quick_calibration);
    bool getCalibrationSpeed();

    // Member functions to use when the calibration has been performed/is loaded
    void setVolume(byte _step);
    void mute();
    void unmute();

    // Member functions used for calibration
    void doCalibration();
    void doCalibrationLDR(bool _quick_calibration, byte _ldr_no, bool _is_shunt);
    bool isCalibrated();
    bool isCalibrationRunning();
    byte getCalibrationProgress();
    bool loadCalibration();
    bool saveCalibration();
    void activateCalibrationRelays();
    void deactivateCalibrationRelays();

  private:
    long size            =  10000; // Value in Ohm
    byte numberOfSteps   =     99;
    byte type            =      1; // 1 = Logaritmic, 2 = Linear
    long max_attenuation =    -60; // dB
    long inputResAmp     = 100000; // Value in Ohm

    byte selectedStep;

    bool calibration_speed = QUICK;
    bool is_calibrated = false;
    bool is_calibration_in_progress = false;
    bool is_muted = false;
    bool is_calibration_relays_active = false;
    byte calibration_progress = 0; // The progress of on-going calibration in percent
};

#endif
