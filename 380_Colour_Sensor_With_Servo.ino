#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include <QTRSensors.h>
#include <Servo.h>

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

// Servo
Servo grabber_servo;

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

enum Sense_Colours{
  RED = 0,
  BLUE = 1,
  GREEN = 2,
  BLACK = 3,
  WHITE = 4,
};

Sense_Colours readColour = RED;

Sense_Colours getColour(Adafruit_TCS34725 tcs) {
  uint16_t red, green, blue, c;
  tcs.setInterrupt(false);  // turn on LED

  delay(60);  // takes 50ms to read

  tcs.getRawData(&red, &green, &blue, &c);
  
  tcs.setInterrupt(true);  // turn off LED

  if(((float) red/c) > 0.43) { 
    return RED;
  } else if(((float) green/c) > 0.39) {
    return GREEN;
  } else if(((float) blue/c) > 0.34) {
    return BLUE;
  } else {
    return WHITE;
  }
}


void setup()
{
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  delay(100);
  // Onboard Components
  pinMode(PC13, INPUT); // Blue Button
  pinMode(PA5, OUTPUT); // LED
  digitalWrite(PA5, HIGH); // Indicates Calibration Mode

  // Servo Motors
  grabber_servo.attach(11);

  // Serial Monitor
  Serial.begin(9600);

  if (!tcs.begin()) {
    Serial.println("No TCS34725 found ... check your connections");
    while (1); // halt!
  }
}

void loop()
{
  readColour = getColour(tcs);
  switch(readColour) {
    case BLUE:
      Serial.println("BLUE");
      grabber_servo.write(0); 
      break;
    case GREEN:
      grabber_servo.write(180); 
      Serial.println("GREEN");
      break;
    case RED:
      Serial.println("RED");
      break;
    case WHITE:
      Serial.println("WHITE");
      break;
    case BLACK:
      Serial.println("BLACK");
      break;
    default:
      Serial.println("ERROR");
      break;
  }
}
