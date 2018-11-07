#include <LDRPotentiometer.h>

LDRPotentiometer chLeft;

void setup()
{
  Serial.begin(9600);

  Serial.print("Pot. size       = ");Serial.println(chLeft.getSize());
  Serial.print("Number of steps = ");Serial.println(chLeft.getSteps());
  Serial.print("Min Ohm serie   = ");Serial.println(chLeft.getSerieMin());
  Serial.print("Min Ohm shunt   = ");Serial.println(chLeft.getShuntMin());
  Serial.println();


  chLeft.setType(1); //Logaritmic
  Serial.println();
  Serial.println("Logaritmic"); 
  for (int i = chLeft.getSteps(); i> 0; i--) {
    Serial.print("step=");Serial.print(i);  
    Serial.print(" Serial=");Serial.print(chLeft.getSerieAtStep(i));
    Serial.print(" Shunt=");Serial.println(chLeft.getShuntAtStep(i));
  }
  

  chLeft.setType(2); //Linear
  Serial.println();
  Serial.println("Linear");
  for (int i = chLeft.getSteps(); i> 0; i--) {
    Serial.print("step=");Serial.print(i);  
    Serial.print(" Serial=");Serial.print(chLeft.getSerieAtStep(i));
    Serial.print(" Shunt=");Serial.println(chLeft.getShuntAtStep(i));
  }
}

void loop()
{

}
