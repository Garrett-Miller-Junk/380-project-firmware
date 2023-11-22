#include <Servo.h>
#include "colourSensor.h"

// Drive Motor Pins //
#define enA 9 
#define in1 6
#define in2 7

#define enB 10
#define in3 4
#define in4 5

#define pwm_max 255

#define servo_open 30
#define servo_closed 130

// Servo //
Servo grabber_servo;

// Program Logic //
bool start_program = false;

bool start_grab = false;
bool start_release = false;
bool stop_run = false;

// BANG BANG Values //
#define DIFFERENTIAL 70;
int left_motor_speed;
int right_motor_speed;
int base_speed_left = 90;
int base_speed_right = 110; // natural curve


// Colour Sensors //
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
// will just be used to initialize the second sensor
Adafruit_TCS34725 tcs2 = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);


void setup()
{

  // Colour Sensor Init //
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


  // Onboard Components //
  pinMode(PC13, INPUT);       // Blue Button
  pinMode(PA5, OUTPUT);       // LED
  digitalWrite(PA5, HIGH);


  // DC Motors // 

  // Motor A - Right
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  // Motor B - Left
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // FWD Direction to start
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  // Servo Motor // 
  grabber_servo.attach(11);

  // Serial Monitor //
  Serial.begin(9600);
  
}

void loop() {

  Sense_Colours left_colour, right_colour = RED;

  grabber_servo.write(servo_open);

  while (!start_program) {
    start_program = !digitalRead(PC13);
  }

  Serial.println("----- START PROGRAM -----");

  while (start_program) {

    // Line Follow FWD until target //
    while (!start_grab) {
      Bang_Bang_line_follow(tcs);
      left_colour = getColour(tcs, S_LEFT);
      right_colour = getColour(tcs, S_RIGHT);
      // maybe add an estimated time elapsed before we start polling for target
      if (left_colour == BLUE || right_colour == BLUE) {
        start_grab = true;
      }
    }

    // Grab mechanism //
    Serial.println("GRAB HER I DON'T EVEN KNOW HER");
    digitalWrite(in2, LOW);
    digitalWrite(in4, LOW);
    grabber_servo.write(servo_closed);
    delay(80000); // long pause for now
    digitalWrite(in2, HIGH);
    digitalWrite(in4, HIGH);

    // Line Follow BKWD until safe zone //
    while (!start_release) {
      Bang_Bang_line_follow(tcs); // make a reverse function
      left_colour = getColour(tcs, S_LEFT);
      right_colour = getColour(tcs, S_RIGHT);
      // maybe add an estimated time elapsed before we start polling for target
      if (left_colour == GREEN || right_colour == GREEN) {
        start_release = true;
      }
    }

    // Release mechanism //
    Serial.println("RELEASE");
    digitalWrite(in2, LOW);
    digitalWrite(in4, LOW);
    grabber_servo.write(servo_open);
    delay(8000);
    digitalWrite(in2, HIGH);
    digitalWrite(in4, HIGH);

    // Line Follow BKWD until start zone //
    while (!stop_run) {
      Bang_Bang_line_follow(tcs); // make a reverse function
      // maybe add an estimated time elapsed before we start polling for target
      if (digitalRead(PIN_A10)) {
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
        digitalWrite(in3, LOW);
        digitalWrite(in4, LOW);
      }
    }

    // Go BKWD a bit to cross line //

    


  }
}



void Bang_Bang_line_follow(Adafruit_TCS34725 tcs) {
  
  left_motor_speed = base_speed_left;
	right_motor_speed = base_speed_right;

  if(getColour(tcs, S_LEFT) == RED) {
    left_motor_speed -= DIFFERENTIAL;
    right_motor_speed += DIFFERENTIAL;
  }
  if(getColour(tcs, S_RIGHT) == RED) {
    left_motor_speed += DIFFERENTIAL;
    right_motor_speed -= DIFFERENTIAL;
  }

  // Serial.print(left_motor_speed);
  // Serial.print(" / ");
  // Serial.print(right_motor_speed);
  // Serial.print(" | ");
  // Serial.print(getColour(tcs, S_LEFT));
  // Serial.print(" / ");
  // Serial.println(getColour(tcs, S_RIGHT));

  analogWrite(enA, right_motor_speed);
  analogWrite(enB, left_motor_speed);

  // delay(50);

}
