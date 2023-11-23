#include <Servo.h>
#include "colourSensor.h"

// Drive Motor Pins
#define enA 9 
#define in1 6
#define in2 7

#define enB 10
#define in3 4
#define in4 5

#define pwm_max 255

#define colour_line GREEN

// Servo
Servo grabber_servo;

#define OPEN_VALUE 30
#define CLOSED_VALUE 130


int left_motor_speed;
int right_motor_speed;

// bang bang speeds //

float straight_threshold = 0.04;
int full_speed = 255;
int reverse_left = -60;
int reverse_right = -60;

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
  digitalWrite(PA5, HIGH);

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

  do{

  }while(digitalRead(PC13));

  Serial.println("----- START PROGRAM -----");

  while (true) {

    do {
      PID_line_follow(tcs, &left_colour, &right_colour);
    } while (left_colour != BLUE && right_colour != BLUE);

    // GRAB MECHANISM //
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
    grabber_servo.write(CLOSED_VALUE);
    delay(10000);

    // DO A 180, then, run same line follow until both see red. (if unequal adjusts, inverse them)

    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);

    do{
      PID_line_follow(tcs, &left_colour, &right_colour);
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
      PID_line_follow(tcs, &left_colour, &right_colour);
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



void PID_line_follow(Adafruit_TCS34725 tcs, Sense_Colours *left_colour, Sense_Colours *right_colour) {
  float left_percent, right_percent;
  *left_colour = getColour(tcs, S_LEFT, &left_percent);
  *right_colour = getColour(tcs, S_RIGHT, &right_percent);

  if (colour_line == RED) {

    if (left_percent > right_percent) {             // red stronger on left
      left_motor_speed = full_speed;
      right_motor_speed = reverse_right;                      // negative
    } else if (left_percent > right_percent) {      // red stronger on right
      right_motor_speed = full_speed;
      left_motor_speed = reverse_left;
    }
    if (left_percent - right_percent < 0.02 && left_percent - right_percent > -0.02) {
      left_motor_speed = full_speed;
      right_motor_speed = full_speed;
    }

  } else if (colour_line == GREEN) {
    if (left_percent < right_percent) {             // green stronger on left
      left_motor_speed = full_speed;
      right_motor_speed = reverse_right;                      // negative
   } else if (left_percent > right_percent) {       // green stronger on right
      right_motor_speed = full_speed;
      left_motor_speed = reverse_left;
    }
    if (left_percent - right_percent < straight_threshold && left_percent - right_percent > straight_threshold*-1) {
      left_motor_speed = full_speed;
      right_motor_speed = full_speed;
    }
  }

  if (left_motor_speed > 0){
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
  } else {
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
      left_motor_speed *= -1;
  }

  if (right_motor_speed > 0){
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
  } else {
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      right_motor_speed *= -1;
  }

  //print it for testing
  Serial.print(left_percent);
  Serial.print("  ");
  Serial.print(right_percent);
  Serial.print("  ");
  // Serial.print(total_error);
  Serial.print(" | ");
  Serial.print(left_motor_speed); 
  Serial.print("  ");
  Serial.println(right_motor_speed);



  // RIGHT (facing grabber side)
  analogWrite(enA, right_motor_speed);
  // LEFT
  analogWrite(enB, left_motor_speed);

}
