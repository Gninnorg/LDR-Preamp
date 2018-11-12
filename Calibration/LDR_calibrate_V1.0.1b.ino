/*
  LDR Calibration v1.0

  Calibration of the test targets is done in approx. 746 seconds with precision within -0.5/+0.5 (and often better)
*/

#include <Wire.h>
#include <DAC7678.h>                    // This library is used for i2c control of the TI DAC7678
#include <defines.h>
#include <Adafruit_ADS1015.h>           // This library is for both the 12 bit ADS1015 and the 16 bit ADS1115 - we use the ADS1115: please note: there is a timing error in the library, that have to be corrected (the ESP32 is much faster than the Arduino). In Adafruit_ADS1015.h change ADS1115_CONVERSIONDELAY to 16: #define ADS1115_CONVERSIONDELAY (16)

#define DEBUG 1                        // Can be set to 0 for no debug information, 1 for basic debug information, 2 for verbose debug information
#define PRECISION_RESISTOR_VALUE 1994  // The exact value in Ohms of the precision 2K resistor

// DO NOT CHANGE BELOW THIS LINE
#define ADC_RESOLUTION 0.125           // The resolution of the ADS1115 (which depends on the set gain of the ADC - see further below)
#define DAC7678ADDRESS 0x4C            // Address of the DAC7678
#define ADS1115ADDRESS 0x48            // Address of the ADS1115

DAC7678 dac(DAC7678ADDRESS);
Adafruit_ADS1115 ads(ADS1115ADDRESS);


void setup() {
  Serial.begin(115200);

  // Check communication with DAC7678 and ADS1115
  // POSSIBLE IMPROVEMENTS to error handling (decide what to do in case of errors)
  Wire.begin();
  bool i2c_error = false;

  Wire.beginTransmission(DAC7678ADDRESS);
  if (Wire.endTransmission() > 0) {
    Serial.println("Unable to communicate with DAC7678");
    i2c_error = true;
  }

  Wire.beginTransmission(ADS1115ADDRESS);
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

  // The ADC input range (or gain) of the ADS1115 can be adjusted, but we need the +/- 4.096V range. Be careful never to exceed VDD +0.3V max (The LDR controller uses 3.3V, so absolute max. = 3.6V).
  ads.setGain(GAIN_ONE);              // 1x gain   +/- 4.096V  1 bit = 0.125mV
}

// Set the output of the specified channel of the DAC to output. Channel is 0-7, Output 0-4095.
void setDAC(byte channel, int output)
{
  dac.set(channel, output);
}

// Set the delay before measuring to give the LDR time to stabilize - only used during calibration proces
void DACdelay(byte channel, int output, byte retries)
{
  int ldr_delay;

  if (channel == 0) // If channel 0 (10K) then we need looong delays - if channel 1 (100K) it is not needed ;-)
  {
    if (output < 5) ldr_delay = 4000;
    else if (output < 20) ldr_delay = 2500;
    else if (output < 50) ldr_delay = 1250;
    else if (output < 100) ldr_delay = 750;
    else if (output < 1000) ldr_delay = 600;
    else if (output < 2000) ldr_delay = 150;
    else if (output < 3000) ldr_delay = 125;
    else ldr_delay = 100;
  }
  else
  {
    if (output < 10) ldr_delay = 500;
    else if (output < 20) ldr_delay = 250;
    else if (output < 50) ldr_delay = 200;
    else if (output < 100) ldr_delay = 150;
    else if (output < 500) ldr_delay = 140;
    else if (output < 1000) ldr_delay = 130;
    else if (output < 2000) ldr_delay = 125;
    else ldr_delay = 100;
  }

  // Delay for twice as long if we are doing a precise calibration and we are in a retry loop
  if (retries > 0) ldr_delay *= 2;

  delay(ldr_delay);
}

long set_and_getResistance(byte channel, int output, byte retries)
{
  setDAC(channel, output);
  DACdelay(channel, output, retries);
  return measureResistance(8);
}

// Return the current resistance - if the parameter samples is > 0 (or > 1 actually) oversampling will be used 
long measureResistance(byte samples)
{
  long total_sum = 0;
  for (int i = 0; i < samples; i++) {
    long refvoltage = ads.readADC_SingleEnded(0);
    delayMicroseconds(900);
    long testresult = ads.readADC_SingleEnded(1);
    delayMicroseconds(900);
    total_sum += ((testresult * PRECISION_RESISTOR_VALUE) / (refvoltage - testresult));
  }
  if (samples > 1) return total_sum / samples; else return total_sum;
}

// Run Exponential Search to find the DAC output needed to get the LDR-resistance to match target. with startPosition = ie. 40 the algorithm will try 40, 80, 320, 2560 or with startPostion 2 it will be 2, 4, 16, 256 etc.
int ExponentialSearch(int startPosition, int endPosition, long target, byte channel, byte retries)
{
  int bound = startPosition;

  // find the interval in which the target would reside
  while (bound < endPosition && set_and_getResistance(channel, bound, retries) > target)
      if (bound < 1) bound = 1; else bound *= 2;  // calculate the next power of 2
  if (measureResistance(8) == target)
    return bound;
  else
    // call binary search for found interval
    return BinarySearch(bound / 2, min(bound, 4095), target, channel, retries);
}

// Perform Binary Search for target by adjusting the specified channel of the DAC
int BinarySearch(int startPosition, int endPosition, long target, byte channel, byte retries)
{
  int pos = startPosition;
  int testpos;
  int limit = endPosition;

  while (pos < limit)
  {
    testpos = pos + ((limit - pos) >> 1); // Find next test position by adding half of the remaining positions to the current position
    long result = set_and_getResistance(channel, testpos, retries);
    if (result > target)
      pos = testpos + 1;
    else if (result == target)
      return testpos; 
    else
      limit = testpos;
  }
  return pos - 1; // return best matching higher value (which is found at pos - 1)
}

int Calibrate(bool quick)
{
  long arr[] = {53195, 23276, 21692, 20233, 18887, 17644, 16494, 15429, 14442, 13524, 12672, 11879, 11140, 10452, 9810, 9211, 8651, 8128, 7638, 7180, 6752, 6350, 5973, 5621, 5290, 4979, 4687, 4414, 4156, 3915, 3688, 3474, 3274, 3085, 2907, 2740, 2583, 2435, 2296, 2164, 2041, 1924, 1815, 1712, 1614, 1523, 1436, 1355, 1278, 1206, 1137, 1073, 1013, 955, 901, 851, 803, 757, 715, 674, 636, 601, 567, 535, 505, 477, 450, 424, 401, 378, 357, 337, 318, 300, 283, 267, 252, 238, 225, 212, 200, 189, 179, 169, 159, 150, 142, 134, 126, 119, 113, 106, 100, 95, 89, 84, 80, 75, 71, 67, 63, 60, 56, 53, 45};
  
  int steps = 105;                        // Max. is 105 as there is 105 values in the array used for testing
  
  int current_bias_adj = 4;              // We'll start with the bias (the 10K channel) set to 4 (just to save some time as 0 to 3 results in resistances too high to be useful)
  int current_fine_adj = 1;              // We'll start with the fine adjustment (the 100K channel) set to 1 to avoid turning the DAC output completely off
  long maximum_with_set_bias = 0;        // Holds the maximum possible resistance with the selected bias value
  long minimum_with_set_bias = 99999999; // Holds the minimum possible resistance with the selected bias value. Initialized to "out of range" to ensure bias adjustment will take place on first step
  long result = 0;                       // The resistance of the LDR with the found calibration settings for the step
  byte retries = 0;                      // Keep track of the number of retries - only used for precise calibration (quick is false)
  int prev_bias = current_bias_adj;      // Keep track of the bias of the previous step - only used for precise calibration (quick is false) where we need this value if a retry is needed
  bool found = false;                    // Used to indicate if the calibration of the current step is done
  float precision = 0.0;                 // The precision of the current step
  float low_precision = 0.0;             // The precision of the step with biggest negative deviation of the calibration
  float high_precision = 0.0;            // The precision of the step with biggest positive deviation of the calibration
  uint32_t ts1, ts2 = 0;                 // Timers used to calculate the run time

#if DEBUG > 0
  Serial.println("---------------------------------------------------"); if (quick) Serial.print("QUICK "); else Serial.print("PRECISE "); Serial.println("calibration started"); Serial.println("---------------------------------------------------");
#endif

  // Reset DAC channels to start values
  setDAC(0, current_bias_adj);
  setDAC(1, current_fine_adj);
  DACdelay(0, current_bias_adj, 0);  // Let LDR stabilize

  // Used to calculate duration of the calibration
  ts1 = millis();

  for (int step = 0; step < steps; step++)
  {
    // Find the target for this step
    long step_target = arr[step];

    // Save the current bias setting (if we are doing a precise calibration, we might need to retry or recalibrate this step if the wanted precision is not achieved)
    prev_bias = current_bias_adj;

    // Adjust the bias if the target is smaller than what is possible with the current bias setting or if we are doing a precise calibration and the target differs with more than 5K from the maximum resistance of the currently set bias
    if ( (minimum_with_set_bias > step_target) || (!quick && (step_target < (maximum_with_set_bias - 5000))) ) {
#if DEBUG > 1
      Serial.println("Adjusting bias: ");
#endif
      // We reset the fine step (100K channel) to 1 when we adjust bias - but only if it isn't already
      if (current_fine_adj != 1) {
        current_fine_adj = 1;
        setDAC(1, current_fine_adj);
        DACdelay(1, current_fine_adj, retries);
      }
      if ((current_bias_adj < 2000) && (retries == 0))
        current_bias_adj = ExponentialSearch(current_bias_adj, 4095, step_target, 0, retries);
      else
        current_bias_adj = BinarySearch(current_bias_adj, 4095, step_target, 0, retries);

      // Set the DAC to the maximum resistance and measure it
      maximum_with_set_bias = set_and_getResistance(0, current_bias_adj, retries);

      // As a precaution we check if the maximum resistance with the found bias is less than our target (at bias, 1) - if it is (because of drift of the LDR or measurement imprecision) we adjust bias down until the resistance is higher than the target value
      while (maximum_with_set_bias < step_target) {
#if DEBUG > 1
        Serial.print("Bias set to "); Serial.print(current_bias_adj); Serial.print(" = "); Serial.print(maximum_with_set_bias); Serial.print(": It is too low for target "); Serial.print(step_target); Serial.println(" so adjust 10K (bias) one step down");
#endif
        current_bias_adj--;
        if (current_bias_adj < 0) {
#if DEBUG 
          Serial.print("ERROR: Unable to find bias resistance high enough to calibrate for target "); Serial.println(step_target);
#endif
          return -1;  // The calibration has failed - should never happen but just in case...
        }
        maximum_with_set_bias = set_and_getResistance(0, current_bias_adj, retries);
      }

      // Find minimum resistance with found bias. We need to know when to adjust bias again (when we need to search for a target resistance that is less than the minimum possible with the current bias)
      minimum_with_set_bias = set_and_getResistance(1, 4095, retries);
      
#if DEBUG > 1
      Serial.print("Bias set to "); Serial.print(current_bias_adj); Serial.print(" = "); Serial.print(maximum_with_set_bias); Serial.print(": Minimum value with this bias is "); Serial.println(minimum_with_set_bias);
#endif
    }

    result = set_and_getResistance(1, current_fine_adj, retries); // Reset to value used before measuring minimum value and measure the resulting resistance 

    // If the resistance does not match the target after adjustment of the bias only, we also have to adjust the 100K channel
    if (!(maximum_with_set_bias == step_target)) {
      if (current_fine_adj < 2000)
        current_fine_adj = ExponentialSearch(current_fine_adj, 4095, step_target, 1, retries);
      else
        current_fine_adj = BinarySearch(current_fine_adj, 4095, step_target, 1, retries);

      result = set_and_getResistance(1, current_fine_adj, retries);  
      // current_fine_adj will either give an exact match of the target, and if that is not possible, current_fine_adj will be the closest result bigger than our target
      // If the found value is not exactly equal to target we check if the resistance at current_fine_adj+1 is closer to the target than the resistance at current_fine_adj to improve the precision even more
      if (!quick)
      {
        if (current_fine_adj < 4095) 
        {
          if (result != step_target)
          {
            long lower_result = set_and_getResistance(1, current_fine_adj + 1, 0);
#if DEBUG > 1
            Serial.print(" Lower: "); Serial.print(lower_result); Serial.print(" Target: "); Serial.print(step_target); Serial.print(" Higher: "); Serial.print(result);
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

    // If quick calibration is true the first result is the final one - if quick is false we check if the result meets the precision criteria (and retry the calibration up to 5 times if it doesn't)
    if (quick) found = true;
    else {
      found = false;
      // If target > 10000R and precision is not within 0.5% then retry. If target <= 10000R and target > 2500 and precision is not within 0.2% then retry. If target < 2500 and precision is not within 0.1% then retry
      if ((((precision < -0.5) || (precision > 0.5)) && (step_target > 10000) && (retries < 3)) || ((((precision < -0.2) || (precision > 0.2)) && ((step_target <= 10000) && (step_target > 2500))) && (retries < 3)) || ((((precision < -0.1) || (precision > 0.1)) && (step_target < 2500)) && (retries < 3)))  {
        retries++;
#if DEBUG > 1
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
      if (quick) Serial.print("QUICK: "); else Serial.print("PRECISE: "); Serial.print("Step "); Serial.print(step); Serial.print(" at "); Serial.print(current_bias_adj); Serial.print(","); Serial.print(current_fine_adj); Serial.print(" with target value "); Serial.print(step_target); Serial.print(": Found "); Serial.print(result); Serial.print(" = "); Serial.print(precision); Serial.println("% precision");
#endif
      // Save the precision of this step if it is lower or higher than previously measured in this calibration run
      if (precision < low_precision) low_precision = precision;
      else if (precision > high_precision) high_precision = precision;
    }
  }

#if DEBUG 
  ts2 = millis();
  if (quick) Serial.print("QUICK"); else Serial.print("PRECISE"); Serial.print(" calibration completed in "); Serial.print((ts2 - ts1) / 1000.0); Serial.print(" seconds with a precision of "); Serial.print(low_precision); Serial.print("/+"); Serial.println(high_precision);
#endif

  return 0; // Calibration is done
}

void loop() {

  for (int i = 0; i < 100; i++) {
    //Calibrate(true); // Perform quick calibration - expected precision is -2/+2% or better
    Calibrate(false);  // Perform precise calibration - expected precision is -0.5/+0.5% or better
  }

  while (true);
}
