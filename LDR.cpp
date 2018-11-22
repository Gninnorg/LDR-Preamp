/*
  LDR -
  Library for control and measurements of a LDR - NSL32SR2-type using DAC7678 and ADS1115
  Created by Jan Abkjer Tofft, November 17, 2018.
  Released into the public domain.
*/

#define DEBUG 2

#include "Arduino.h"
#include "LDR.h"
#include <Wire.h>

#include <mutex>                       // Included to make sure more instances of LDR have mutually exclusive access to the I2C bus

#include "DAC7678.h"                   // This library is used for i2c control of the TI DAC7678
#include "ADS1115.h"                   // This library is for both the 12 bit ADS1015 and the 16 bit ADS1115 - we use the ADS1115: please note: there is a timing error in the library, that have to be corrected (the ESP32 is much faster than the Arduino). In Adafruit_ADS1015.h change ADS1115_CONVERSIONDELAY to 16: #define ADS1115_CONVERSIONDELAY (16)

// DO NOT CHANGE BELOW THIS LINE
#define ADC_RESOLUTION 0.000125        // The resolution of the ADS1115 in volts which depends on the set gain of the ADC

LDR::LDR(std::mutex& passedmyi2cMutex, DAC7678& passedDAC7678, byte _DACBiasChannel, byte _DACAdjChannel, ADS1115& passedADS1115, byte _ADCchannel_LDR, float _refvoltage, unsigned int _value_of_reference_resistor, byte _samples_for_measurements) : myi2cMutex(passedmyi2cMutex), mydac(passedDAC7678), myadc(passedADS1115) {

  //Serial.println("LDR Constructor");
  //Serial.println((long)&myi2cMutex);
  DACBiasChannel = _DACBiasChannel;
  DACAdjChannel = _DACAdjChannel;
  ADCchannel_LDR = _ADCchannel_LDR;
  reference_voltage = _refvoltage;
  value_of_reference_resistor = _value_of_reference_resistor;
  samples_for_measurements = _samples_for_measurements;
  lastSetOutputBiasChannel = 0;
  lastSetOutputAdjChannel = 0;
  //minimumResistance = getMinimumResistance();
  //setDACchannels(0, 0);
}

void LDR::turnOn()
{
  setDACchannels(4095, 4095);
  lastSetOutputBiasChannel = 4095;
  lastSetOutputAdjChannel = 4095;
}

void LDR::turnOff()
{
  setDACchannels(0, 0);
  lastSetOutputBiasChannel = 0;
  lastSetOutputAdjChannel = 0;
}

void LDR::setDACBiasChannel(unsigned int _value)
{
  LDR::myi2cMutex.lock();
  mydac.set(DACBiasChannel, _value);
  LDR::myi2cMutex.unlock();
  lastSetOutputBiasChannel = _value;
}

void LDR::setDACAdjChannel(unsigned int _value)
{
  myi2cMutex.lock();
  mydac.set(DACAdjChannel, _value);
  myi2cMutex.unlock();
  lastSetOutputAdjChannel = _value;
}

void LDR::setDACchannels(unsigned int _10K_channel_value, unsigned int _100K_channel_value)
{
  setDACBiasChannel(_10K_channel_value);
  setDACAdjChannel(_100K_channel_value);
}

// Return the current resistance - if the LDR object has been initialized with > 0 _samples_for_measurements, oversampling will be used
long LDR::getResistance()
{
  myi2cMutex.lock();
  unsigned int refvoltage = reference_voltage / ADC_RESOLUTION; // Calculate the number of ADC measurement steps that equals the set reference_voltage in Volts.
  long total_sum = 0;

  for (int i = 0; i < samples_for_measurements; i++) {
    unsigned int LDRvoltage = myadc.readADC_SingleEnded(ADCchannel_LDR);
    delayMicroseconds(900);
    //Serial.print("LDRvoltage: "); Serial.println(LDRvoltage);
    //Serial.print("refvoltage: "); Serial.println(refvoltage);
    if (LDRvoltage >= refvoltage)
    {
      // Serial.println("LDRvoltage >= refvoltage");
      total_sum += 10000000; // When measuring maximum resistance we must avoid division by zero (if equal) and take care of small measurement imprecision of the ADC when DAC is turned off
    }
    else
      total_sum += ((LDRvoltage * value_of_reference_resistor) / (refvoltage - LDRvoltage));
  }
  myi2cMutex.unlock();
  if (samples_for_measurements > 1) return total_sum / samples_for_measurements; else return total_sum;
}

// Return the minimum resistance in Ohms - for NSL32SR2 it will normally be around 45 Ohms as we do not drive the LDR with more than 7mA
long LDR::measureMinimumResistance()
{
  turnOn();
  minimumResistance = getResistance();
  return minimumResistance;
}

long LDR::getMinimumResistance()
{
  if (minimumResistance == 99999999)
    return measureMinimumResistance();
  else
    return minimumResistance;
}

// Measure the maximum resistance
long LDR::measureMaximumResistance()
{
  turnOff();
  doStabilizationDelay(DACBiasChannel, 0);
  maximumResistance = getResistance();
  return maximumResistance;
}

// Return the maximum resistance in Ohms - for NSL32SR2 it will normally be 10MOhms or more
long LDR::getMaximumResistance()
{
  if (maximumResistance == 0)
    return measureMaximumResistance();
  else
    return maximumResistance;
}

long LDR::setDACChannelAndMeasureResistance(bool _quick_calibration, byte _channel, unsigned int _value, byte _delay_duration_modifier)
{
  if (_channel == DACBiasChannel)
    setDACBiasChannel(_value);
  else
    setDACAdjChannel(_value);
  doStabilizationDelay(_channel, _delay_duration_modifier);
  return getResistance();
}

long LDR::doStabilizationDelay(byte _channel, byte _delay_duration_modifier)
{
  int ldr_delay;

  if (_channel == DACBiasChannel) // If the bias has been adjusted we need looong delays - if the adj channel has been adjusted it is not needed as the power is much smaller ;-)
  {
    if (lastSetOutputBiasChannel < 5) ldr_delay = 6000;        // increased from 4000 -> 6000
    else if (lastSetOutputBiasChannel < 20) ldr_delay = 5000;  // increased from 2500 -> 5000
    else if (lastSetOutputBiasChannel < 50) ldr_delay = 1500;  // increased from 1250 -> 1500
    else if (lastSetOutputBiasChannel < 100) ldr_delay = 1000; // increased from 750 -> 1000
    else if (lastSetOutputBiasChannel < 1000) ldr_delay = 750; // increased from 600 -> 750
    else if (lastSetOutputBiasChannel < 2000) ldr_delay = 300; // increased from 150 -> 300
    else if (lastSetOutputBiasChannel < 3000) ldr_delay = 150; // increased from 125 -> 150
    else ldr_delay = 125;                                      // increased from 100 -> 125
  }
  else
  {
    if (lastSetOutputAdjChannel < 10) ldr_delay = 500;
    else if (lastSetOutputAdjChannel < 20) ldr_delay = 250;
    else if (lastSetOutputAdjChannel < 50) ldr_delay = 200;
    else if (lastSetOutputAdjChannel < 100) ldr_delay = 150;
    else if (lastSetOutputAdjChannel < 500) ldr_delay = 140;
    else if (lastSetOutputAdjChannel < 1000) ldr_delay = 130;
    else if (lastSetOutputAdjChannel < 2000) ldr_delay = 125;
    else ldr_delay = 100;
  }

  // Delay for twice as long if we are doing a precise calibration and we are in a retry loop
  if (_delay_duration_modifier > 0) ldr_delay *= 2;

  delay(ldr_delay);

  return ldr_delay;
}

// Run Exponential Search to find the DAC output needed to get the LDR-resistance to match target. with _startPosition = ie. 40 the algorithm will try 40, 80, 320, 2560 or with startPostion 2 it will be 2, 4, 16, 256 etc.
int LDR::ExponentialSearch(bool _quick_calibration, int _startPosition, int _endPosition, long _target, byte _channel, byte _retries)
{
  int bound = _startPosition;

  // find the interval in which the target would reside
  while (bound < _endPosition && setDACChannelAndMeasureResistance(_quick_calibration, _channel, bound, _retries) > _target)
      if (bound < 1) bound = 1; else bound *= 2;  // calculate the next power of 2
  if (getResistance() == _target)
    return bound;
  else
    // call binary search for found interval
    return BinarySearch(_quick_calibration, bound / 2, min(bound, 4095), _target, _channel, _retries);
}

// Perform Binary Search for target by adjusting the specified channel of the DAC
int LDR::BinarySearch(bool _quick_calibration, int _startPosition, int _endPosition, long _target, byte _channel, byte _retries)
{
  int pos = _startPosition;
  int testpos;
  int limit = _endPosition;

  while (pos < limit)
  {
    testpos = pos + ((limit - pos) >> 1); // Find next test position by adding half of the remaining positions to the current position
    long result = setDACChannelAndMeasureResistance(_quick_calibration, _channel, testpos, _retries);
    if (result > _target)
      pos = testpos + 1;
    else if (result == _target)
      return testpos;
    else
      limit = testpos;
  }
  if (pos > 0) return pos - 1; // return best matching higher value (which is found at pos - 1)
  else return pos;
}

int LDR::calibrateLDR(bool _quick_calibration, byte _ldr_no, bool _is_shunt)
{
  long arr[] = {53195, 23276, 21692, 20233, 18887, 17644, 16494, 15429, 14442, 13524, 12672, 11879, 11140, 10452, 9810, 9211, 8651, 8128, 7638, 7180, 6752, 6350, 5973, 5621, 5290, 4979, 4687, 4414, 4156, 3915, 3688, 3474, 3274, 3085, 2907, 2740, 2583, 2435, 2296, 2164, 2041, 1924, 1815, 1712, 1614, 1523, 1436, 1355, 1278, 1206, 1137, 1073, 1013, 955, 901, 851, 803, 757, 715, 674, 636, 601, 567, 535, 505, 477, 450, 424, 401, 378, 357, 337, 318, 300, 283, 267, 252, 238, 225, 212, 200, 189, 179, 169, 159, 150, 142, 134, 126, 119, 113, 106, 100, 95, 89, 84, 80, 75, 71, 67, 63, 60, 56, 53, 45};

  int steps = 105;                        // Max. is 105 as there is 105 values in the array used for testing

  int current_bias_adj = 4;              // We'll start with the bias (the 10K channel) set to 4 (just to save some time as 0 to 3 results in resistances way too high to be useful)
  int current_fine_adj = 1;              // We'll start with the fine adjustment (the 100K channel) set to 1 to avoid turning the DAC output completely off
  long maximum_with_set_bias = 0;        // Holds the maximum possible resistance with the selected bias value. Initialized to "out of range" to ensure bias adjustment will take place on first step
  long minimum_with_set_bias = 99999999; // Holds the minimum possible resistance with the selected bias value. Initialized to "out of range" to ensure bias adjustment will take place on first step
  long result = 0;                       // The resistance of the LDR with the found calibration settings for the step
  byte retries = 0;                      // Keep track of the number of retries - only used for accurate calibration (_quick_calibration is false)
  int prev_bias = current_bias_adj;      // Keep track of the bias of the previous step - only used for accurate calibration (_quick_calibration is false) where we need this value if a retry is needed
  bool found = false;                    // Used to indicate if the calibration of the current step is done
  float precision = 0.0;                 // The precision of the current step
  float low_precision = 0.0;             // The precision of the step with biggest negative deviation of the calibration
  float high_precision = 0.0;            // The precision of the step with biggest positive deviation of the calibration
  uint32_t ts1, ts2 = 0;                 // Timers used to calculate the run time

#if DEBUG > 0
  Serial.println("---------------------------------------------------"); Serial.print("LDR "); Serial.print(_ldr_no); Serial.print(" "); if (_quick_calibration) Serial.print("QUICK"); else Serial.print("ACCURATE"); Serial.println(" calibration started"); Serial.println("---------------------------------------------------");
#endif

  // Reset DAC channels to start values
  setDACBiasChannel(current_bias_adj);
  setDACAdjChannel(current_fine_adj);
  doStabilizationDelay(DACBiasChannel, retries); // Let LDR stabilize (initially retries is 0, so a normal delay will be done)

  // Used to calculate duration of the calibration
  ts1 = millis();

  for (int step = 0; step < steps; step++)
  {
    // Find the target for this step ---depends on the type of LDR (series or shunt). The calibration algorithm assumes that increased output to the DAC channels results in decreased resistance so we must make sure it starts with the highest resistance target---
    long step_target = arr[step];

    // Save the current bias setting (if we are doing a precise calibration, we might need to retry or recalibrate this step if the wanted precision is not achieved)
    prev_bias = current_bias_adj;

    // Adjust the bias if the target is smaller than what is possible with the current bias setting or if we are doing a precise calibration and the target differs with more than 5K from the maximum resistance of the currently set bias
    if ( (minimum_with_set_bias > step_target) || (!_quick_calibration && (step_target < (maximum_with_set_bias - 5000))) ) {
#if DEBUG > 1
      Serial.print("LDR "); Serial.print(_ldr_no); Serial.print(" "); if (_quick_calibration) Serial.print("QUICK:"); else Serial.print("ACCURATE:"); Serial.print(" Step "); Serial.print(step); Serial.print(" "); Serial.println("Adjusting bias");
#endif
      // We reset the fine step (100K channel) to 1 when we adjust bias
      current_fine_adj = 1;
      setDACAdjChannel(current_fine_adj);
      doStabilizationDelay(DACAdjChannel, retries);
      
      if ((current_bias_adj < 2000) && (retries == 0))
        current_bias_adj = ExponentialSearch(_quick_calibration, current_bias_adj, 4095, step_target, DACBiasChannel, retries);
      else
        current_bias_adj = BinarySearch(_quick_calibration, current_bias_adj, 4095, step_target, DACBiasChannel, retries);

      // Set the DAC to the found adjustment and measure resistance. As current_fine_adj has already been set to 1 we will get the maximum resistance
      setDACBiasChannel(current_bias_adj);
      doStabilizationDelay(DACBiasChannel, retries);
      maximum_with_set_bias = getResistance();

      // As a precaution we check if the maximum resistance with the found bias is less than our target (at bias, 1) - if it is (because of drift of the LDR or measurement imprecision) we adjust bias down until the resistance is higher than the target value
      while (maximum_with_set_bias < step_target) {
#if DEBUG > 1
        Serial.print("LDR "); Serial.print(_ldr_no); Serial.print(" "); if (_quick_calibration) Serial.print("QUICK:"); else Serial.print("ACCURATE:"); Serial.print(" Step "); Serial.print(step); Serial.print(" "); Serial.print("Bias set to "); Serial.print(current_bias_adj); Serial.print(" = "); Serial.print(maximum_with_set_bias); Serial.print(": It is too low for target "); Serial.print(step_target); Serial.println(" so adjust 10K (bias) one step down");
#endif
        current_bias_adj--;
        if (current_bias_adj < 0) {
#if DEBUG
          Serial.print("LDR "); Serial.print(_ldr_no); Serial.print(" "); if (_quick_calibration) Serial.print("QUICK:"); else Serial.print("ACCURATE:"); Serial.print(" Step "); Serial.print(step); Serial.print(" "); Serial.print("ERROR: Unable to find bias resistance high enough to calibrate for target "); Serial.println(step_target);
#endif
          return -1;  // The calibration has failed - should never happen but just in case...
        }
        setDACBiasChannel(current_bias_adj);
        doStabilizationDelay(DACBiasChannel, retries);
        maximum_with_set_bias = getResistance();
      }

      // Find minimum resistance with found bias. We need to know when to adjust bias again (when we need to search for a target resistance that is less than the minimum possible with the current bias)
      setDACAdjChannel(4095);
      doStabilizationDelay(DACAdjChannel, retries);
      minimum_with_set_bias = getResistance();

#if DEBUG > 1
      Serial.print("LDR "); Serial.print(_ldr_no); Serial.print(" "); if (_quick_calibration) Serial.print("QUICK:"); else Serial.print("ACCURATE:"); Serial.print(" Step "); Serial.print(step); Serial.print(" "); Serial.print("Bias set to "); Serial.print(current_bias_adj); Serial.print(" = "); Serial.print(maximum_with_set_bias); Serial.print(": Minimum value with this bias is "); Serial.println(minimum_with_set_bias);
#endif
    }
    
    setDACBiasChannel(current_bias_adj);
    setDACAdjChannel(current_fine_adj);
    doStabilizationDelay(DACBiasChannel, retries);
    result = getResistance();            // Reset to value used before measuring minimum value and measure the resulting resistance

    // If the resistance does not match the target after adjustment of the bias only, we also have to adjust the 100K channel
    if (!(result == step_target)) {
      if (current_fine_adj < 2000)
        current_fine_adj = ExponentialSearch(_quick_calibration, current_fine_adj, 4095, step_target, DACAdjChannel, retries);
      else
        current_fine_adj = BinarySearch(_quick_calibration, current_fine_adj, 4095, step_target, DACAdjChannel, retries);

      setDACAdjChannel(current_fine_adj);
      doStabilizationDelay(DACAdjChannel, retries);
      result = getResistance();

      // current_fine_adj will either give an exact match of the target, and if that is not possible, current_fine_adj will be the closest result bigger than our target
      // If the found value is not exactly equal to target we check if the resistance at current_fine_adj+1 is closer to the target than the resistance at current_fine_adj to improve the precision even more
      if (!_quick_calibration)
      {
        if (current_fine_adj < 4095)
        {
          if (result != step_target)
          {
            setDACAdjChannel(current_fine_adj + 1);
            doStabilizationDelay(DACAdjChannel, retries);
            long lower_result = getResistance();

#if DEBUG > 1
            Serial.print("LDR "); Serial.print(_ldr_no); Serial.print(" "); if (_quick_calibration) Serial.print("QUICK:"); else Serial.print("ACCURATE:"); Serial.print(" Step "); Serial.print(step); Serial.print(" "); Serial.print(" Lower: "); Serial.print(lower_result); Serial.print(" Target: "); Serial.print(step_target); Serial.print(" Higher: "); Serial.print(result);
#endif
            if (abs(step_target - lower_result) < abs(step_target - result)) {
#if DEBUG > 1
              Serial.println("-> Lower is the better match");
#endif
              current_fine_adj = current_fine_adj + 1;
              result = lower_result;
            }
#if DEBUG > 1
            else Serial.println("-> Higher is the better match");
#endif
          }
        }
      }
    }

    // Calculate the achieved precision
    precision = (float)((result - step_target) * 100.00) / step_target;

    // If quick calibration is true the first result is the final one - if quick is false we check if the result meets the precision criteria (and retry the calibration up to 3 times if it doesn't)
    if (_quick_calibration) found = true;
    else {
      found = false;
      // If target > 10000R and precision is not within 0.25% then retry. If target <= 10000R and target > 2500 and precision is not within 0.15% then retry. If target < 2500 and precision is not within 0.1% then retry
      if ((((precision < -0.25) || (precision > 0.25)) && (step_target > 10000) && (retries < 3)) ||
          ((((precision < -0.15) || (precision > 0.15)) && ((step_target <= 10000) && (step_target > 2500))) && (retries < 3)) ||
          ((((precision < -0.1) || (precision > 0.1)) && (step_target <= 2500)) && (retries < 3)))  {
        retries++;
#if DEBUG > 1
        Serial.print("LDR "); Serial.print(_ldr_no); Serial.print(" "); if (_quick_calibration) Serial.print("QUICK:"); else Serial.print("ACCURATE:"); Serial.print(" Step "); Serial.print(step); Serial.print(" "); 
        Serial.print("Retry attempt "); Serial.print(retries); Serial.print(" for step "); Serial.print(step); Serial.print(" with target "); Serial.print(step_target); Serial.print(" as result achieved at "); Serial.print(current_bias_adj); Serial.print(","); Serial.print(current_fine_adj); Serial.print(" is "); Serial.print(result); Serial.print(" = "); Serial.print(precision); Serial.println("%");
#endif
        current_bias_adj = prev_bias;
        minimum_with_set_bias = 99999999;
        maximum_with_set_bias = 0;
        current_fine_adj = 1;
        step--;
      }
      else {
        // We have a match or have retried 3 times: keep the found calibration result as it will not get any better
        found = true;
        retries = 0;
      }
    }

    if (found) {
      // Print and save result
#if DEBUG
      Serial.print("LDR "); Serial.print(_ldr_no); Serial.print(" "); if (_quick_calibration) Serial.print("QUICK:"); else Serial.print("ACCURATE:"); Serial.print(" Step "); Serial.print(step); Serial.print(" at "); Serial.print(current_bias_adj); Serial.print(","); Serial.print(current_fine_adj); Serial.print(" with target value "); Serial.print(step_target); Serial.print(": Found "); Serial.print(result); Serial.print(" = "); Serial.print(precision); Serial.println("% precision");
#endif
      // Save the precision of this step if it is lower or higher than previously measured in this calibration run
      if (precision < low_precision) low_precision = precision;
      else if (precision > high_precision) high_precision = precision;
    }
  }

#if DEBUG
  ts2 = millis();
  Serial.print("LDR "); Serial.print(_ldr_no); Serial.print(" "); if (_quick_calibration) Serial.print("QUICK"); else Serial.print("ACCURATE"); Serial.print(" calibration completed in "); Serial.print((ts2 - ts1) / 1000.0); Serial.print(" seconds with a precision of "); Serial.print(low_precision); Serial.print("/+"); Serial.println(high_precision);
#endif

  // Make sure the calibration_in_progress is set to false but only for the actual LDR ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  return 0; // Calibration is done
}
