/*
  LDR Calibration v2.4.1
  Uses ModifiedExponentialSearch for bias if target >2K and BinarySearch for bias for smaller targets
  Fine adjustment is done by ExponentialSearch
  Runtime approx. 460 seconds, precision approx. +2%/-2% - usually better
*/

#include <Wire.h>
#include <DAC7678.h>             // This library is used for i2c control of the TI DAC7678
#include <defines.h>
#include <Adafruit_ADS1015.h>    // This library is for both the 12 bit ADS1015 and the 16 bit ADS1115 - we use the ADS1115: please note: there is a timing error in the library, that have to be corrected (the ESP32 is much faster than the Arduino). In Adafruit_ADS1015.h change ADS1115_CONVERSIONDELAY to 16: #define ADS1115_CONVERSIONDELAY (16)
DAC7678 dac(0x4C);
Adafruit_ADS1115 ads(0x48);

float adc_resolution = 0.125;    // The resolution of the ADC1115 (which depends on the set gain of the ADC - see below)

long arr[] = {53195, 23276, 21692, 20233, 18887, 17644, 16494, 15429, 14442, 13524, 12672, 11879, 11140, 10452, 9810, 9211, 8651, 8128, 7638, 7180, 6752, 6350, 5973, 5621, 5290, 4979, 4687, 4414, 4156, 3915, 3688, 3474, 3274, 3085, 2907, 2740, 2583, 2435, 2296, 2164, 2041, 1924, 1815, 1712, 1614, 1523, 1436, 1355, 1278, 1206, 1137, 1073, 1013, 955, 901, 851, 803, 757, 715, 674, 636, 601, 567, 535, 505, 477, 450, 424, 401, 378, 357, 337, 318, 300, 283, 267, 252, 238, 225, 212, 200, 189, 179, 169, 159, 150, 142, 134, 126, 119, 113, 106, 100, 95, 89, 84, 80, 75, 71, 67, 63, 60, 56, 53, 45};

int steps = 105; // Max. is 105

void setup() {
  Serial.begin(115200);

  // Check communication with DAC7678 and ADS1115
  Wire.begin();
  bool i2c_error = false;

  Wire.beginTransmission(0x4C);
  if (Wire.endTransmission() > 0) {
    Serial.println("Unable to communicate with DAC7678");
    i2c_error = true;
  }

  Wire.beginTransmission(0x48);
  if (Wire.endTransmission() > 0) {
    Serial.println("Unable to communicate with ADS1115");
    i2c_error = true;
  }

  if (i2c_error) {
    Serial.println("Not able to continue :-(");
    while (true);
  }

  dac.begin();                        // Initialize the DAC
  //dac.setVREF(INT);                 // Configure for internal voltage reference - not used as we use 3.3 supply as reference voltage
  dac.set(0);                         // Set output of all DAC channels to zero - might be omitted as it is default
  dac.enable();                       // Power up all DAC Channels

  ads.begin();                        // Initialize the ADC, ADS1115

  // The ADC input range (or gain) can be adjusted, but we need the +/- 4.096V range. Be careful never to exceed VDD +0.3V max (The LDR controller uses 3.3V, so absolute max. = 3.6V).
  ads.setGain(GAIN_ONE);              // 1x gain   +/- 4.096V  1 bit = 0.125mV
}

// Set the output of the specified channel of the DAC to output. Channel is 0-7, Output 0-4095.
void setDAC(byte channel, int output)
{
  dac.set(channel, output);
}

// Set the delay before measuring to give the LDR time to stabilize - only used during calibration proces
void DACdelay(byte channel, int output)
{
  if (channel == 0) // If channel 0 (10K) then we need looong delays - if channel 1 (100K) it is not needed ;-)
  {
    if (output < 10) delay(10000);
    else if (output < 20) delay(8000);
    else if (output < 50) delay(5000);
    else if (output < 100) delay(3000);
    else if (output < 1000) delay(500);
    else delay(250);
  }
  else
  {
    if (output < 20) delay(1000);
    else if (output < 50) delay(800);
    else if (output < 500) delay(500);
    else if (output < 1000) delay(300);
    else delay(100);
  }
}

long set_and_getResistance(byte channel, int output)
{
  setDAC(channel, output);
//  Serial.print(setting); Serial.print(" -> ");
  DACdelay(channel, output);
  long resistorValue = measureResistance();
//  Serial.println(resistorValue);
  return resistorValue;
}

long measureResistance()
{
  long refvoltage = ads.readADC_SingleEnded(0);
  long testresult = ads.readADC_SingleEnded(1);
  return ((testresult * 1994) / (refvoltage - testresult));
}

// Run Exponential Search to find the DAC output needed to get the LDR-resistance to match target. with startPosition = ie. 40 the algorithm version will try 40, 1600, or with startPostion 2 it will be 2, 4, 16, 256 etc.
int ExponentialSearch(int startPosition, int endPosition, long target, byte channel)
{
  //  Serial.println("ExponentialSearch");

  int bound = startPosition;

  // find the range in which the target would reside
  while (bound < endPosition && set_and_getResistance(channel, bound) > target)
      if (bound < 1) bound = 1; else bound *= 2;  // calculate the next power of 2
  if (measureResistance() == target)
    return bound;
  else
    // call binary search for found interval
    return BinarySearch(bound / 2, min(bound, 4095), target, channel);
}

// Run modified Exponential Search to find the DAC output needed to get the LDR-resistance to match target. with startPosition = ie. 40 the modified version will try 40, 41, 42, 44, 48, 56, 72, 104, 168, 296, 552 etc. With startPosition 2 it will try 2, 3, 4, 6, 10, 18, 34 etc.
int ModifiedExponentialSearch(int startPosition, int endPosition, long target, byte channel)
{
  //  Serial.println("ModifiedExponentialSearch");

  int bound = startPosition;
  int step = 0;

  // find the range in which the target would reside
  while (bound < endPosition && set_and_getResistance(channel, bound) > target) {
    if (step < 1) step = 1; else step *= 2; // calculate the next power of 2
    bound += step; // and add it to the bound
  }
  if (measureResistance() == target)
    return bound;
  else
    // call binary search for found interval
    return BinarySearch(bound - step, min(bound, 4095), target, channel);
}

// Perform Binary Search for target by adjusting the specified channel of the DAC
int BinarySearch(int startPosition, int endPosition, long target, byte channel)
{
  //  Serial.println("BinarySearch");

  int pos = startPosition;
  int testpos;
  int limit = endPosition;

  while (pos < limit)
  {
    testpos = pos + ((limit - pos) >> 1); // Find next test position by adding half of the remaining positions to the current position
    long result = set_and_getResistance(channel, testpos);
    if (result > target)
      pos = testpos + 1;
    else if (result == target)
      return testpos;
    else
      limit = testpos;
  }
  return pos - 1; // return best matching position
}

void Calibrate()
{
  int bias = 4; // We'll start with the bias (the 10K channel) set to 4 (just to save some time as 1 to 3 results in too high resistances) 
  int index = 1; // Start value of the 100K channel
  long maximum_with_set_bias = 0; // Holds the maximum possible resistance with the selected bias value
  long minimum_with_set_bias = 99999999; // Holds the minimum possible resistance with the selected bias value. Initialized to "out of range" to ensure bias adjustment will take place on first step
  long result = 0;
  float low_precision = 0.0; // Keep the 
  float high_precision = 0.0;
  uint32_t ts1, ts2 = 0; // Timers

  Serial.println("LDR calibration ");
  Serial.println("---------------------------------------------------");

  // Reset DAC channels to start values
  setDAC(0, bias);
  setDAC(1, index);

  ts1 = millis();

  for (int step = 0; step < steps; step++)
  {
    // Serial.print("Calibrating step "); Serial.print(step); Serial.print(" with target value "); Serial.println(arr[step]);

    long step_target = arr[step]; 

    // Adjust the bias if the target is smaller than what is possible with the current bias setting
    if (minimum_with_set_bias > step_target) {
      //Serial.println("Adjusting bias: ");
      // When we adjust bias we reset the fine step (100K channel) to 1 - but only if it isn't already
      if (index != 1) {
        index = 1;
        setDAC(1, index);
        DACdelay(1, index);
      }
      if (step_target > 2000)
        bias = ModifiedExponentialSearch(bias, 4095, step_target, 0); // At targets >2k modified exponential search have an advantage as the needed steps are close to each other
      else
        bias = BinarySearch(bias, 4095, step_target, 0);              // At low level targets binary search is quicker as the needed steps are far from each other
      maximum_with_set_bias = measureResistance();

      // As a precaution we check if the found bias is high enough - if not: adjust bias down until the resistance is higher than the target value
      while (maximum_with_set_bias < step_target) {
      // Serial.println("Adjust 10K (bias) one step down... ");
        bias--; // Maybe we should check for bias < 0 even though it should never occur
        maximum_with_set_bias = set_and_getResistance(0, bias);
      }

      // Find minimum value with found bias
      minimum_with_set_bias = set_and_getResistance(1, 4095);
      setDAC(1, index); // Reset to value used before measuring minimum value
      DACdelay(1, index);
//      Serial.print("Bias set to "); Serial.print(bias); Serial.print(" = "); Serial.print(maximum_with_set_bias); Serial.print(". Minimum value with this bias is "); Serial.println(minimum_with_set_bias);
    }
    
    // If the resistance does not match the target after adjustment of the bias only, we also have to adjust the 100K channel
    if (!(maximum_with_set_bias == arr[step])) {
      //      Serial.println("100K channel adjustment");
      index = ExponentialSearch(index, 4095, arr[step], 1);
    }

    // Print result
    Serial.print("Step "); Serial.print(step); Serial.print(" at "); Serial.print(bias); Serial.print(","); Serial.print(index); Serial.print(" with target value "); Serial.print(step_target); Serial.print(" found ");
    result = set_and_getResistance(1, index);
    float precision = (float)((result - step_target) * 100.00) / arr[step];
    Serial.print(result); Serial.print(" = "); Serial.print(precision); Serial.println("% precision");

    // Save the precision value of this step, if it is lower or higher than previously measured in this calibration run
    if (precision < low_precision)
      low_precision = precision;
    else if (precision > high_precision)
      high_precision = precision;
  }

  ts2 = millis();

  Serial.print("Calibration completed in "); Serial.print((ts2 - ts1) / 1000.0); Serial.print(" seconds. Precision is "); Serial.print(low_precision); Serial.print("/+"); Serial.println(high_precision);
}

void loop() {

  for (int i = 0; i < 100; i++) Calibrate();

  while (true);
}
