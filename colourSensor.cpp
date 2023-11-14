#include "colourSensor.h"

void setSensorSide(int side){
  if(side == S_LEFT) {
    digitalWrite(pinSensorL, HIGH);
    digitalWrite(pinSensorR, LOW);
  } else {
    digitalWrite(pinSensorR, HIGH);
    digitalWrite(pinSensorL, LOW);
  }
  delay(50);
}


void getRGB(Adafruit_TCS34725 tcs, float *r, float *g, float *b, int side) {
  setSensorSide(side);
  tcs.setInterrupt(false);  // turn on LED

  delay(60);  // takes 50ms to read

  tcs.getRGB(r, g, b);
  
  tcs.setInterrupt(true);  // turn off LED
}


Sense_Colours getColour(Adafruit_TCS34725 tcs, int side) {
  uint16_t red, green, blue, c;
  setSensorSide(side);
  tcs.setInterrupt(false);  // turn on LED

  delay(60);  // takes 50ms to read

  tcs.getRawData(&red, &green, &blue, &c);
  
  tcs.setInterrupt(true);  // turn off LED

  if(c<2000) {
    return BLACK;
  } else if(((float) red/c) > 0.43) { 
    return RED;
  } else if(((float) green/c) > 0.39) {
    return GREEN;
  } else if(((float) blue/c) > 0.36) {
    return BLUE;
  } else {
    return WHITE;
  }
}