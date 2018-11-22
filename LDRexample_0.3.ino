
#include "LDRStereoPotentiometer.h"

#define DEBUG 1                        // Can be set to 0 for no debug information, 1 for basic debug information, 2 for verbose debug information

void setup()
{
  Serial.begin(115200);
}

void loop()
{
  LDRStereoPotentiometer myStereoPotentiometer;
  int i = 1;
  do {
    myStereoPotentiometer.setCalibrationSpeed(ACCURATE);
    Serial.print("Started..."); Serial.println(i++);
    myStereoPotentiometer.doCalibration();
    Serial.println("Done...");
  } while (true);
}
