#ifndef _COLOURSENSOR_H_
#define _COLOURSENSOR_H_
#include <Wire.h>
#include "Adafruit_TCS34725.h"

#define pinSensorControl PA6

#define S_LEFT 0
#define S_RIGHT 1

enum Sense_Colours{
  RED = 0,
  BLUE = 1,
  GREEN = 2,
  BLACK = 3,
  WHITE = 4,
};

void getRGB(Adafruit_TCS34725 tcs, float *r, float *g, float *b, int side);

Sense_Colours getColour(Adafruit_TCS34725 tcs, int side);

void getRGB_Percent(Adafruit_TCS34725 tcs);

#endif