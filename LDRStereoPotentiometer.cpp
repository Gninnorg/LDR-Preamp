/*
  LDRStereoPotentiometer -
  Library for a LDR based stereo potentiometer

  Created by Jan Abkjer Tofft, November 18, 2018.
  Released into the public domain.
*/

#include "Arduino.h"
#include "LDRStereoPotentiometer.h"

DAC7678 dac(DAC7678ADDRESS);
ADS1115 adc(ADS1115ADDRESS);
std::mutex i2cMutex;

LDRPotentiometer Potentiometer_calc;

LDR LDRs[4] {
  {std::ref(i2cMutex), std::ref(dac), L_SER_DAC_BIAS, L_SER_DAC_ADJ, std::ref(adc), L_SER_ADC, REFERENCE_VOLTAGE, L_SER_RES, SAMPLES_PER_MEASUREMENT},  // Left series
  {std::ref(i2cMutex), std::ref(dac), L_SHU_DAC_BIAS, L_SHU_DAC_ADJ, std::ref(adc), L_SHU_ADC, REFERENCE_VOLTAGE, L_SHU_RES, SAMPLES_PER_MEASUREMENT},  // Left shunt
  {std::ref(i2cMutex), std::ref(dac), R_SER_DAC_BIAS, R_SER_DAC_ADJ, std::ref(adc), R_SER_ADC, REFERENCE_VOLTAGE, R_SER_RES, SAMPLES_PER_MEASUREMENT},  // Right series
  {std::ref(i2cMutex), std::ref(dac), R_SHU_DAC_BIAS, R_SHU_DAC_ADJ, std::ref(adc), L_SHU_ADC, REFERENCE_VOLTAGE, L_SHU_RES, SAMPLES_PER_MEASUREMENT}   // Right shunt
};

// May need to be extended with a reference to i2cMutex if it needs to be global (when threaded calibration is in progress, the main thread might need to update an i2c display via i2c)
LDRStereoPotentiometer::LDRStereoPotentiometer() {
  Serial.begin(115200);
  Wire.begin();
  
  dac.begin(DAC7678ADDRESS);          // Initialize the DAC
  //dac.setVREF(INT);                 // Configure for internal voltage reference - not used as we use 3.3 supply as reference voltage
  dac.set(0);                         // Set lastSetOutput10KChannel of all DAC channels to zero - might be omitted as it is default
  dac.enable();                       // Power up all DAC Channels

  adc.begin();                        // Initialize the ADC, ADS1115
  adc.setGain(GAIN_ONE);              // The ADC input range (or gain) of the ADS1115 can be adjusted, but we use the +/- 4.096V range. Be careful never to exceed VDD +0.3V max (The LDR controller uses 3.3V, so absolute max. = 3.6V). GAIN_ONE is +/- 4.096V, 1 bit = 0.125mV 
  
byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print(" 0x");
      if (address<16) 
        Serial.print("0");
      Serial.print(address,HEX);
      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknown error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }
    else
    {
      Serial.print(" ----");
    }
    if (address % 16 == 0) Serial.println();
  }
  Serial.println();
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("Done\n");

  bool i2c_error = false;

  Wire.beginTransmission(DAC7678ADDRESS);
  if (Wire.endTransmission() > 0) {
    #if DEBUG > 0 
	Serial.println("Unable to communicate with DAC7678");
	#endif
    i2c_error = true;
  }

  Wire.beginTransmission(ADS1115ADDRESS);
  if (Wire.endTransmission() > 0) {
	#if DEBUG > 0 
    Serial.println("Unable to communicate with ADS1115");
	#endif
    i2c_error = true;
  }

  if (i2c_error) {
	#if DEBUG > 0 
    Serial.println("Not able to continue :-(");
	#endif
    while (true);
  }
}


void LDRStereoPotentiometer::setSize(long _size)
{
  size = _size;
  is_calibrated = false;
}

long LDRStereoPotentiometer::getSize()
{
  return size;
}

void LDRStereoPotentiometer::setSteps(byte _numberOfSteps)
{
  numberOfSteps = _numberOfSteps;
  is_calibrated = false;
}

byte LDRStereoPotentiometer::getSteps()
{
  return numberOfSteps;
}

// 1 = Logarithmic, 2 = Linear
void LDRStereoPotentiometer::setType(byte _type)
{
  type = _type;
  is_calibrated = false;
}

byte LDRStereoPotentiometer::getType()
{
  return type;
}

void LDRStereoPotentiometer::setInputRes(long _inputResAmp)
{
  is_calibrated = false;
  inputResAmp = _inputResAmp;
}

long LDRStereoPotentiometer::getInputRes()
{
  return inputResAmp;
}

void LDRStereoPotentiometer::setCalibrationSpeed(bool _quick_calibration)
{
	calibration_speed = _quick_calibration;
}

bool LDRStereoPotentiometer::getCalibrationSpeed()
{
	return calibration_speed;
}

// Member functions to use when the calibration has been performed/is loaded
void LDRStereoPotentiometer::setVolume(byte _step)
{
	
}

void LDRStereoPotentiometer::mute()
{
  // Should this be a method of the potentiometer? Yes, because we need to mute before we start a calibration?
  is_muted = true;
}

void LDRStereoPotentiometer::unmute()
{
  // Should this be a method of the potentiometer? Yes, because we need to unmute when a calibration has been done/loaded?
  is_muted = false;
}	
	
// Member functions used only for calibration

// Activate the calibration relays
void LDRStereoPotentiometer::activateCalibrationRelays()
{
	mute();
	// Activate the calibration relays
	is_calibration_relays_active = true;
}

// Deactivate the calibration relays
void LDRStereoPotentiometer::deactivateCalibrationRelays()
{
	// Deactivate the calibration relays
	is_calibration_relays_active = false;
	unmute();
}

// Perform calibration with the values set for the StereoPotentiometer
void LDRStereoPotentiometer::doCalibration()
{
  activateCalibrationRelays();
  is_calibrated = false;
  is_calibration_in_progress = true;

  // Calibration code goes here - lets save some time and start a thread for each LDR
  // First we pass the address of the member function of the LDR class as &LDR::calibrateLDR.
  // Second we pass the object as a parameter considering "this" is an implicit parameter of a member function
  // Then we pass the needed parameters for the member function
  std::thread t1(&LDR::calibrateLDR, LDRs[L_SER], calibration_speed, L_SER, SERIES);
  //std::thread t2(&LDR::calibrateLDR, LDRs[L_SHU], calibration_speed, L_SHU, SHUNT);
  //std::thread t3(&LDR::calibrateLDR, LDRs[R_SER], calibration_speed, R_SER, SERIES);
  //std::thread t4(&LDR::calibrateLDR, LDRs[R_SHU], calibration_speed, R_SHU, SHUNT);
      
  // While loop that calls getCalibrationProgress() until it reaches 100 - maybe doCalibration can be started in its own thread from the main program so progress can be displayed?
  
  t1.join();
  //t2.join();
  //t3.join();
  //t4.join();
  
  is_calibration_in_progress = false;
}

bool LDRStereoPotentiometer::isCalibrated()
{
  return is_calibrated;	
}

bool LDRStereoPotentiometer::isCalibrationRunning()
{
  return is_calibration_in_progress;
}

byte LDRStereoPotentiometer::getCalibrationProgress()
{
  if (isCalibrationRunning())
    return 95; // Must call all LDRs to check on their progress and calculate an overall progress in percent
  else
	return 100;
}

bool LDRStereoPotentiometer::loadCalibration()
{
  // Do stuff to load calibration from static ram (simulated EEPROM). Return true if possible, false if no data found
  // Also load settings used to perform the saved calibration
  // Input/output setting and settings for volume of all inputs, start volume etc. is not loaded here as it has nothing to do with the potentiometer
  is_calibrated = true;
  return true;
}

// Do stuff to save calibration to static ram (simulated EEPROM). Return true if possible, false if not
bool LDRStereoPotentiometer::saveCalibration()
{
  // Do stuff to save calibration to static ram (simulated EEPROM). Return true if possible, false if not
  // Also save settings used to perform the calibration: steps etc.
  // Input/output setting and settings for volume of all inputs, start volume etc. is not saved here as it has nothing to do with the potentiometer
  
  is_calibration_in_progress = false;
  is_calibrated = true;
  return true;
}
