/*
  LDRPotentiometer -
  Library for calculating resistor values for LDR based potentiometer

  Created by Casten Grønning, November 2, 2018.
  Released into the public domain.
*/

#ifndef LDRPotentiometer_h
#define LDRPotentiometer_h
#endif

#include "Arduino.h"

class LDRPotentiometer
{
  public:
    LDRPotentiometer();

    void setSize(long _size);
    long getSize();

    void setSteps(byte _num);
    byte getSteps();

    void setType(byte _type);
    byte getType();

    void setShuntMin(byte _minResShunt);
    byte getShuntMin();

    void setSerieMin(byte _minResSerie);
    byte getSerieMin();

    void setInputRes(long _inputResAmp);
    long getInputRes();

    double calcInOutRatio(float _dB);
    void calcResistorsAtStep(byte _step);
    long getSerieAtStep(byte _step);
    long getShuntAtStep(byte _step);


  private:
    long size            =  10000; // Value in Ohm
    byte numberOfSteps   =     99;
    byte type            =      1; // 1 = Logaritmic, 2 = Linear
    long max_attenuation =    -60; // dB

    byte minResSerie   =     50; // Value in Ohm
    byte minResShunt   =     50; // Value in Ohm
    long inputResAmp   = 100000; // Value in Ohm


    byte selectedStep;
    long shuntAtStep;
    long serieAtStep;
};
