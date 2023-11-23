#include <QTRSensors.h>
#include <Servo.h>
#include "colourSensor.h"
// note: check pwm limit

// Drive Motor Pins
#define enA 9 
#define in1 6
#define in2 7

#define enB 10
#define in3 4
#define in4 5

#define pwm_max 255

// Servo
Servo grabber_servo;

#define OPEN_VALUE 30
#define CLOSED_VALUE 130
// Program Logic


const int BASE_SPEED = 230;

const float MIN_RED = 0.45;
const float MAX_RED = 0.75;
const float MAX_ERROR = MAX_RED - MIN_RED;

const float P_VAL = 0.08 / MAX_ERROR;
const float I_VAL = 0.002 / MAX_ERROR;
const float D_VAL = 0.3 / MAX_ERROR;

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
  // Onboard Components
  pinMode(PC13, INPUT); // Blue Button
  pinMode(PA5, OUTPUT); // LED
  digitalWrite(PA5, HIGH); // Indicates Calibration Mode

  // Buttons (temp)
  pinMode(33, INPUT); // B
  pinMode(34, INPUT); // G
  pinMode(PIN_A10, INPUT); // R

  // DC Motors

  // Motor A
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  // Motor B
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  // FWD Direction to start
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  // Servo Motors
  grabber_servo.attach(11);
  grabber_servo.write(OPEN_VALUE);
  // Serial Monitor
  Serial.begin(9600);  
}

void loop() {
  Sense_Colours left_colour, right_colour;
  float old_error = 0.0;
  float error_sum = 0.0;
  do{

  }while(digitalRead(PC13));

  Serial.println("----- START PROGRAM -----");

  while (true) {

    do {
      PID_line_follow(tcs, &old_error, &error_sum, &left_colour, &right_colour);
    } while (left_colour != BLUE && right_colour != BLUE);

    // run grab function
    Serial.println("GRAB HER I DON'T EVEN KNOW HER");
    digitalWrite(in2, LOW);
    digitalWrite(in4, LOW);
    grabber_servo.write(CLOSED_VALUE);
    delay(10000);
    digitalWrite(in2, HIGH);
    digitalWrite(in4, HIGH);

    do{
      PID_line_follow(tcs, &old_error, &error_sum, &left_colour, &right_colour);
    } while(left_colour != GREEN && right_colour!= GREEN);

    // run release function
    Serial.println("RELEASE");
    digitalWrite(in2, LOW);
    digitalWrite(in4, LOW);
    grabber_servo.write(OPEN_VALUE);
    delay(10000);
    digitalWrite(in2, HIGH);
    digitalWrite(in4, HIGH);

    do {
      PID_line_follow(tcs, &old_error, &error_sum, &left_colour, &right_colour);
    } while (left_colour != RED || right_colour != RED );      
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
  }
  // line follow until target (loop that polls colour sensor)
  // grab
  // reverse line follow until safe zone (loop that polls colour sensor)
  // release
  // reverse line follow (loop)
}

// add colour sensor bang bang backup
// test if red detected

void PID_line_follow(Adafruit_TCS34725 tcs, float *old_error, float *error_sum, Sense_Colours *left_colour, Sense_Colours *right_colour) {
  float left_percent, right_percent;
  *left_colour = getColour(tcs, S_LEFT, &left_percent);
  *right_colour = getColour(tcs, S_RIGHT, &right_percent);
  // calculate the error as the difference in percent red between the left and right
  float error = left_percent - right_percent;
  // calculate the derivative of the error as the new error minus the old error
  float error_diff = error - *old_error;
  // calculate the integral of the error as the old sum of the errors plus the new error
  *error_sum = *error_sum + error;
  // multiply each PID error value by their respective constants and convert that into an int
  int total_error = (int) error*P_VAL + error_diff*D_VAL + (*error_sum)*I_VAL;

  //print it for testing
  Serial.println(total_error);

  //increase the right motor speed by the error (as a larger left value means more error means left is closer to red)
  analogWrite(enA, (BASE_SPEED + total_error));
  // decrease the left motor speed by the error
  analogWrite(enB, (BASE_SPEED - total_error));
  // set the old error to the new error
  *old_error = error;
}
