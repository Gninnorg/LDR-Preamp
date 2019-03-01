#include <Wire.h>
#include <ADS1115.h>

  ADS1115 ads(ADS1115::ADD_GND, ADS1115::GAIN_6_144V, ADS1115::CONTINUES);

int i;

void setup(void)  
{
  Serial.begin(115200);
  Serial.println("Hello!");
  
  Serial.println("Getting single-ended readings from AIN0..3");
  Serial.println("ADC Range: +/- 6.144V (1 BIT = 0.1875mV/ADS1115)");

  ads.showConfigRegister(); 
    
  ads.begin();
  Serial.println("Sample\tadc0\tadc1\tadc2\tadc3");
  Serial.println("------\t------\t------\t------\t------");
  i = 0;      
} 
  
void loop(void) 
{
  
  int16_t adc0, adc1, adc2, adc3;
  i++; 
    
  
  adc0 = ads.getCountA0Gnd();   
  adc1 = ads.getCountA1Gnd();
  adc2 = ads.getCountA2Gnd();
  adc3 = ads.getCountA3Gnd();
      
 
  Serial.print(i); 
  Serial.print("\t");   Serial.print(adc0);
  Serial.print("\t");   Serial.print(adc1);
  Serial.print("\t");   Serial.print(adc2);
  Serial.print("\t");   Serial.print(adc3);
  Serial.print("\t");   Serial.print(ads.getR2ValueVoltageDivider(adc3, adc2, 2000));
  Serial.print("\t");   Serial.print(ads.convertCountToVolt(adc2),4);
  Serial.print("\t");   Serial.print(ads.getSerieForAttenuation(-2, 10000));
  Serial.print("\t");   Serial.print(ads.getShuntForAttenuation(-2, 10000));
  Serial.print("\t");   Serial.print(ads.getSerieForAttenuation(-3, 1));
  Serial.print("\t");   Serial.print(ads.getShuntForAttenuation(-3, 1));
  Serial.print("\t");   Serial.print(ads.getSerieForAttenuation(-6.02, 1));
  Serial.print("\t");   Serial.print(ads.getShuntForAttenuation(-6.02, 1));
  Serial.println(" ");
  
  delay(1000);
  
  
}
