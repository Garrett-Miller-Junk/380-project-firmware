#include "colourSensor.h"
// note: check pwm limit

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

//will just be used to initialize the second sensor
Adafruit_TCS34725 tcs2 = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

void setup()
{
  pinMode(pinSensorControl, OUTPUT);
  digitalWrite(pinSensorControl, HIGH);
  Serial.begin(9600);
  if (tcs.begin()) {
    Serial.println("Sensor 1 Found");
  } else {
    Serial.println("No SENSOR 1 found ... check your connections");
  }
  digitalWrite(pinSensorControl, LOW);
  delay(100);
    if (tcs2.begin()) {
    Serial.println("Sensor 2 Found");
  } else {
    Serial.println("No SENSOR 2 found ... check your connections");

  }
  digitalWrite(pinSensorControl, HIGH);
  
}

void loop() {
  float red, green, blue;
  Sense_Colours left_colour, right_colour = RED;

  left_colour = getColour(tcs, S_LEFT);
  right_colour = getColour(tcs, S_RIGHT);

  Serial.print("Left:\t");
  switch(left_colour) {
    case RED:
      Serial.print("RED");
      break;
    case BLUE:
      Serial.print("BLUE");
      break;
    case GREEN:
      Serial.print("GREEN");
      break;
    case WHITE:
      Serial.print("WHITE");
      break;
    case BLACK:
      Serial.print("BLACK");
      break;
  }
  Serial.print('\t');
  Serial.print("RIGHT:\t");
  switch(right_colour) {
    case RED:
      Serial.print("RED");
      break;
    case BLUE:
      Serial.print("BLUE");
      break;
    case GREEN:
      Serial.print("GREEN");
      break;
    case WHITE:
      Serial.print("WHITE");
      break;
    case BLACK:
      Serial.print("BLACK");
      break;
  }
  Serial.print('\n');
/*
  getRGB(tcs, &red, &green, &blue, S_LEFT);
  Serial.print("LEFT: \t");
  Serial.print("R:\t"); Serial.print(int(red)); 
  Serial.print("\tG:\t"); Serial.print(int(green)); 
  Serial.print("\tB:\t"); Serial.print(int(blue));
  Serial.print('\t');

  getRGB(tcs, &red, &green, &blue, S_RIGHT);
  Serial.print("RIGHT: \t");
  Serial.print("R:\t"); Serial.print(int(red)); 
  Serial.print("\tG:\t"); Serial.print(int(green)); 
  Serial.print("\tB:\t"); Serial.print(int(blue));
  Serial.print('\n');
*/
}



