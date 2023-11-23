#include "colourSensor.h"

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

//will just be used to initialize the second sensor
Adafruit_TCS34725 tcs2 = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

float left_red_max, left_red_min;
float right_red_max, right_red_min;
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
  // Onboard Components
  pinMode(PC13, INPUT); // Blue Button
  pinMode(PA5, OUTPUT); // LED
  digitalWrite(PA5, HIGH); // Indicates Calibration Mode

}

void loop() {
  getRGB_Percent(tcs);
}
