/*
     This sketch accepts two numbers from the serial input, ie. 345;1034, and sets output channel 0 of the DAC7678 to the first value
     and channel 1 to the other. The DAC7678 is a eight channel 12-bit DAC so the range is from 0-4095. The DAC can output up to 5V
     so each step is 1,22mV. The initial accuracy is +-5mV.

     After setting the output the 16 bit ADC ADS1115 is used to measure two voltages (as default in the range +/- 6.144V) on its
     channel 0 and 1. The precision with the default gain is 0.1875mV.

*/


#include <Wire.h>
#include "DAC7678.h"
#include "ADS1115.h"         // This library is used for both the 12 bit ADS1015 and the 16 bit ADS1115 - we use the ADS1115
DAC7678 dac(0x4C);
ADS1115 ads(0x48);

unsigned int vref = 3300;             // Voltage reference value for the DAC7678 for calculations 
unsigned int read_value = 0;          // Value read from DAC

float adc_resolution = 0.125;        // The resolution of the ADC1115 (which depends on the set gain of the ADC - see below)

float voltage = 0.0;

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing DAC7678 and ADS1115...");
  dac.begin();                        // Initialize the DAC
  dac.setVREF(INT);                   // Configure for internal voltage reference
  dac.set(0);                         // Set output of all DAC channels to zero - might be omitted
  dac.enable();                       // Power up all DAC Channels

  ads.begin();                        // Initialize the ADC, ADS1115

  // The ADC input range (or gain) can be changed via the following
  // functions, but be careful never to exceed VDD +0.3V max, or to
  // exceed the upper and lower limits if you adjust the input range!
  // Setting these values incorrectly may destroy your ADC!
  //                                                                ADS1115
  //                                                                -------
  // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 0.1875mV (default)
   ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 0.125mV
  // ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 0.0625mV
  // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.03125mV
  // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.015625mV
  // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.0078125mV

  Serial.println("Enter one number in the range 0-4095 and press enter to send to DAC7678 channel 0. Channel 1 will be set to 0.");
  Serial.println("Enter two numbers in the range 0-4095 delimited by semicolon and press return to send to DAC7678 channel 0 and 1.");
  Serial.println("If you press ENTER without any parameters both DAC channels will be set to 0.");
  Serial.println("If you enter a number out of range, ie. 9999, the DAC channels are left as-is and only ADC measurements will be done.");
  Serial.println();
}

void loop() {
  if (Serial.available() > 0)
  {
    int dacA = Serial.readStringUntil(';').toInt();
    int dacB = Serial.readStringUntil('\n').toInt();

    if (((dacA >= 0) && (dacA <= 4095)) && ((dacB >= 0) && (dacB <= 4095)))
    {
      // Send values to DAC7678
      Serial.print("Setting dac channel 0 to ");
      Serial.print(dacA);
      Serial.print(" and channel 1 to ");
      Serial.println(dacB);
      // Send to dac
      dac.set(0, dacA);
      dac.set(1, dacB);
      delay(1000);
      print_measurements();
    }
    else
    {
      Serial.println("Printing measurements:");
      print_measurements();
    }
    Serial.println();
    Serial.println("Enter two numbers (0-4095) delimited by ; and press enter to send to DAC7678:");
  }
}

void print_measurements()
{
  // Read values from DAC7678
  read_value = dac.readDAC(0);              // Read value of DAC channel 0
  voltage = read_value * (vref / 4095.0);   // Calculate voltage output according to the voltage reference
  Serial.print("DAC channel 0 set to ");
  Serial.print((voltage / 1000), 7);        // Prints the calculated voltage
  Serial.println("V");

  read_value = dac.readDAC(1);              // Read value of DAC channel 1
  voltage = read_value * (vref / 4095.0);   // Calculate voltage output according to the voltage reference
  Serial.print("DAC channel 1 set to ");
  Serial.print((voltage / 1000), 7);        // Prints the calculated voltage
  Serial.println("V");

  // Read values from ADC
  int16_t adc0;                             // We read from the ADC, we have a sixteen bit integer as a result
  int16_t adc1;                             // We read from the ADC, we have a sixteen bit integer as a result
  int16_t adc2;                             // We read from the ADC, we have a sixteen bit integer as a result
  int16_t adc3;                             // We read from the ADC, we have a sixteen bit integer as a result

  adc0 = ads.readADC_SingleEnded(0);

  voltage = (adc0 * adc_resolution) / 1000; // Calculate voltage according to the resolution (which depends on the set gain of the ADC)

  Serial.print("ADC measurement channel 0: ");
  Serial.print(adc0);
  Serial.print(" Voltage: ");
  Serial.print(voltage, 7);
  Serial.println("V");

  adc1 = ads.readADC_SingleEnded(1);

  voltage = (adc1 * adc_resolution) / 1000; // Calculate voltage according to the resolution (which depends on the set gain of the ADC)

  Serial.print("ADC measurement channel 1: ");
  Serial.print(adc1);
  Serial.print(" Voltage: ");
  Serial.print(voltage, 7);
  Serial.println("V");

  adc2 = ads.readADC_SingleEnded(2);

  voltage = (adc2 * adc_resolution) / 1000; // Calculate voltage according to the resolution (which depends on the set gain of the ADC)

  Serial.print("ADC measurement channel 2: ");
  Serial.print(adc2);
  Serial.print(" Voltage: ");
  Serial.print(voltage, 7);
  Serial.println("V");

  adc3 = ads.readADC_SingleEnded(3);

  voltage = (adc3 * adc_resolution) / 1000; // Calculate voltage according to the resolution (which depends on the set gain of the ADC)

  Serial.print("ADC measurement channel 3: ");
  Serial.print(adc3);
  Serial.print(" Voltage: ");
  Serial.print(voltage, 7);
  Serial.println("V");
}
