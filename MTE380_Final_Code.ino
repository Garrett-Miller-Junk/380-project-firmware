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

// IR Sensor Array
QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

// Servo
Servo grabber_servo;

// Program Logic
bool start_program = false;
bool start_calibrate = false;

bool start_grab = false;
bool start_release = false;
bool stop_run = false;

// BANG BANG Values
#define DIFFERENTIAL 50;
int left_motor_speed;
int right_motor_speed;
int base_speed = 120;

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

void setup()
{

  // Colour Sensor Toggle Pins
  pinMode(pinSensorR, OUTPUT);
  pinMode(pinSensorL, OUTPUT);
  digitalWrite(pinSensorL, HIGH);
  digitalWrite(pinSensorR, HIGH);
  delay(100);
  tcs.begin();
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

  // Serial Monitor
  Serial.begin(9600);

  // IR Sensor Array
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){30, 16, 17, 19, 20, 21, PIN_A8, PIN_A9}, SensorCount);
  qtr.setEmitterPin(31);
  
}

void loop() {
  Sense_Colours left_colour, right_colour = RED;

  while (!start_calibrate) {
    start_calibrate = !digitalRead(PC13);
    delay(100);
  }

  Serial.println("----- START CALIBRATE -----");
  calibrate();
  Serial.println("----- END CALIBRATE -----");

  while (!start_program) {
    start_program = !digitalRead(PC13);
  }

  Serial.println("----- START PROGRAM -----");

  while (start_program) {

    while (!start_grab) {
      Bang_Bang_line_follow(tcs);
      left_colour = getColour(tcs, S_LEFT);
      right_colour = getColour(tcs, S_RIGHT);
      // maybe add an estimated time elapsed before we start polling for target
      if (left_colour == BLUE || right_colour == BLUE) {
        start_grab = true;
      }
    }

    // run grab function
    Serial.println("GRAB HER I DON'T EVEN KNOW HER");
    digitalWrite(in2, LOW);
    digitalWrite(in4, LOW);
    grabber_servo.write(0);
    delay(8000);
    digitalWrite(in2, HIGH);
    digitalWrite(in4, HIGH);

    while (!start_release) {
      Bang_Bang_line_follow(tcs); // make a reverse function
      left_colour = getColour(tcs, S_LEFT);
      right_colour = getColour(tcs, S_RIGHT);
      // maybe add an estimated time elapsed before we start polling for target
      if (left_colour == GREEN || right_colour == GREEN) {
        start_release = true;
      }
    }
    // run release function
    Serial.println("RELEASE");
    digitalWrite(in2, LOW);
    digitalWrite(in4, LOW);
    grabber_servo.write(180);
    delay(8000);
    digitalWrite(in2, HIGH);
    digitalWrite(in4, HIGH);

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

    


  }

  // line follow until target (loop that polls colour sensor)
  // grab
  // reverse line follow until safe zone (loop that polls colour sensor)
  // release
  // reverse line follow (loop)
  


}

// add colour sensor bang bang backup
// test if red detected

void Bang_Bang_line_follow(Adafruit_TCS34725 tcs) {
  
  left_motor_speed = base_speed;
	right_motor_speed = base_speed;

  if(getColour(tcs, S_LEFT) == RED) {
    left_motor_speed -= DIFFERENTIAL;
    right_motor_speed += DIFFERENTIAL;
  }
  if(getColour(tcs, S_RIGHT) == RED) {
    left_motor_speed += DIFFERENTIAL;
    right_motor_speed -= DIFFERENTIAL;
  }

  Serial.print(left_motor_speed);
  Serial.print(" / ");
  Serial.println(right_motor_speed);

  analogWrite(enA, right_motor_speed);
  analogWrite(enB, left_motor_speed);

  delay(250);

}

void calibrate() {
  // 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
  // = ~25 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on
  Serial.begin(9600);
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);
}
