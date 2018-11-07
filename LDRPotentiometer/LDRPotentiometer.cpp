/*
  LDRPotentiometer -
  Library for calculating resistor values for LDR based potentiometer

  Created by Casten Grønning, November 2, 2018.
  Released into the public domain.
*/

#include "Arduino.h"
#include "LDRPotentiometer.h"
#include "math.h"

LDRPotentiometer::LDRPotentiometer() {
}


void LDRPotentiometer::setSize(long _size)
{
  size = _size;
}

long LDRPotentiometer::getSize()
{
  return size;
}

void LDRPotentiometer::setSteps(byte _numberOfSteps)
{
  numberOfSteps = _numberOfSteps;
}

byte LDRPotentiometer::getSteps()
{
  return numberOfSteps;
}

void LDRPotentiometer::setType(byte _type)
{
  type = _type;
}

byte LDRPotentiometer::getType()
{
  return type;
}

void LDRPotentiometer::setShuntMin(byte _minResShunt)
{
  minResShunt = _minResShunt;
}

byte LDRPotentiometer::getShuntMin()
{
  return minResShunt;
}

void LDRPotentiometer::setSerieMin(byte _minResSerie)
{
  minResSerie = _minResSerie;
}

byte LDRPotentiometer::LDRPotentiometer::getSerieMin()
{
  return minResSerie;
}

double LDRPotentiometer::calcInOutRatio(float _dB)
{
  return pow(10, _dB / 20);
}

void LDRPotentiometer::calcResistorsAtStep(byte _step)
{
  double in_out;
  double stepsize;

  if (_step >= 0 && _step <= numberOfSteps) selectedStep = numberOfSteps -_step;

  if (type == 1) // Logaritmic
  {
    stepsize  = float(max_attenuation) / numberOfSteps;
    in_out = calcInOutRatio(stepsize*selectedStep);

  }

  if (type == 2) //Linear
  {
    selectedStep = _step;
    stepsize = 0.9988 / numberOfSteps;
    in_out = stepsize*selectedStep;

  }

  serieAtStep = max((1 - in_out) * size, minResSerie);
  shuntAtStep = max(size * in_out, minResShunt);

  if (serieAtStep ==  minResSerie) {
    shuntAtStep = serieAtStep / (1 - min(in_out,0.9988));
  }

  if (shuntAtStep == minResShunt) {
    serieAtStep = shuntAtStep / in_out;
  }

}

long LDRPotentiometer::getSerieAtStep(byte _step)
{
  calcResistorsAtStep(_step);
  return serieAtStep;
}

long LDRPotentiometer::getShuntAtStep(byte _step)
{
  calcResistorsAtStep(_step);
  return shuntAtStep;
}
