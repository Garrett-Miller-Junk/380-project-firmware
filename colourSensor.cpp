#include "colourSensor.h"

void setSensorSide(int side){
  side == S_LEFT ? digitalWrite(pinSensorControl, HIGH): digitalWrite(pinSensorControl, LOW);
  delay(1);
}


void getRGB(Adafruit_TCS34725 tcs, float *r, float *g, float *b, int side) {
  setSensorSide(side);
  tcs.getRGB(r, g, b);
}


Sense_Colours getColour(Adafruit_TCS34725 tcs, int side, float *red_percent) {
  uint16_t red, green, blue, c;
  setSensorSide(side);
  tcs.getRawData(&red, &green, &blue, &c);
  *red_percent = (float) red/c;
  if(c<70) {
    return BLACK;
  } else if(*red_percent > 0.59) { 
    return RED;
  } else if(((float) green/c) > 0.38) {
    return GREEN;
  } else if(((float) blue/c) > 0.29) {
    return BLUE;
  } else {
    return WHITE;
  }
}

float getRGB_Percent(Adafruit_TCS34725 tcs, int side) {
  uint16_t red, green, blue, c;
  setSensorSide(side);
  tcs.getRawData(&red, &green, &blue, &c);
  return ((float) red/c);
}
