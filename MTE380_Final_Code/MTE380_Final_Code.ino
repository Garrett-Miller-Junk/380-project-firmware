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
#define DIFFERENTIAL 60;
int left_motor_speed;
int right_motor_speed;
int base_speed = 90;

float right_history_factor = 0.0;
float left_history_factor = 0.0;
float diff_increase = 60;

int time_index = 0;

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_4X);

//will just be used to initialize the second sensor
Adafruit_TCS34725 tcs2 = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_4X);

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

  // Serial Monitor
  Serial.begin(9600);

  // IR Sensor Array
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){30, 16, 17, 19, 20, 21, PIN_A8, PIN_A9}, SensorCount);
  qtr.setEmitterPin(31);
  
}

void loop() {
  Sense_Colours left_colour, right_colour = RED;

  // while (!start_calibrate) {
  //   start_calibrate = !digitalRead(PC13);
  //   delay(100);
  // }

  Serial.println("----- START CALIBRATE -----");
  // calibrate();
  Serial.println("----- END CALIBRATE -----");

  while (!start_program) {
    start_program = !digitalRead(PC13);
  }

  Serial.println("----- START PROGRAM -----");

  while (start_program) {

    while (!start_grab) {
      Bang_Bang_line_follow(tcs, right_history_factor, left_history_factor, left_colour, right_colour);
      // left_colour = getColour(tcs, S_LEFT);
      // right_colour = getColour(tcs, S_RIGHT);
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
      Bang_Bang_line_follow(tcs, right_history_factor, left_history_factor, left_colour, right_colour); // make a reverse function
      // left_colour = getColour(tcs, S_LEFT);
      // right_colour = getColour(tcs, S_RIGHT);
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
      Bang_Bang_line_follow(tcs, right_history_factor, left_history_factor, left_colour, right_colour); // make a reverse function
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

void Bang_Bang_line_follow(Adafruit_TCS34725 tcs, float &rhf, float &lhf, Sense_Colours &rclr, Sense_Colours &lclr) {
  time_index += 1;
  // left_motor_speed = base_speed - 20;
	// right_motor_speed = base_speed + 30;
  left_motor_speed = base_speed;
  right_motor_speed = base_speed;

  lclr = getColour(tcs, S_LEFT);
  rclr = getColour(tcs, S_RIGHT);

  // Left sensor detects line, right doesn't, gradually ramp right up & left down
  // if (lclr == GREEN) {
  //   rhf += 0.2;
  //   if (rhf >= 1.0) rhf = 1.0;

  //   lhf -= 0.2;
  //   if (lhf <= -1.0) lhf = -1.0;
  // }

  // // Right sensor detects line, left doesn't, gradually ramp left up & right down
  // else if (rclr == GREEN) {
  //   rhf -= 0.2;
  //   if (rhf <= -1.0) rhf = -1.0;

  //   lhf += 0.2;
  //   if (lhf >= 1.0) lhf = 1.0;
  // }

  // // Both sensors or no sensors detect line (default case), gradually equalize motors
  // else {
  //   if (rhf > 0) rhf -= 0.2;
  //   else if (rhf < 0) rhf += 0.2;

  //   if (lhf > 0) lhf -= 0.2;
  //   else if (lhf < 0) lhf += 0.2;
  // }

  // Adjusting motor speeds based on history values
  // left_motor_speed += diff_increase*lhf;
  // right_motor_speed += diff_increase*rhf;

  // if(getColour(tcs, S_LEFT) == GREEN) {
  // if (lclr == GREEN) {
  //   left_motor_speed -= DIFFERENTIAL;
  //   right_motor_speed += DIFFERENTIAL;
  // }
  // // if(getColour(tcs, S_RIGHT) == GREEN) {
  // if (rclr == GREEN) {
  //   left_motor_speed += DIFFERENTIAL;
  //   right_motor_speed -= DIFFERENTIAL;
  // }
  if (lclr == GREEN) {
    rhf = 50.0;
    lhf = 0.0;
  }
  if (rclr == GREEN) {
    rhf = 0.0;
    lhf = 50.0;
  }

  // Adjust speed based on history
  if (rhf > 0) {
    rhf += -1;
    left_motor_speed = base_speed - 30;
    right_motor_speed = base_speed + 30; 
  }
  else if (lhf > 0) {
    lhf += -1;
    left_motor_speed = base_speed + 30;
    right_motor_speed = base_speed - 30;
  }

  // Serial.print(str(leftSense) + " / " + str(rightSense) + "\t\t" + str(left_motor_speed) + " / " + str(right_motor_speed) + "\t\t" + str(lhf) + " / " + str(rhf));
  // Serial.print(time_index);
  // Serial.print("\t");
  // Serial.print(lclr);
  // Serial.print(" / ");
  // Serial.print(rclr);
  // Serial.print("\t");
  Serial.print(left_motor_speed);
  Serial.print(" / ");
  Serial.println(right_motor_speed);
  // Serial.print("\t");
  // Serial.print(lhf);
  // Serial.print(" / ");
  // Serial.println(rhf);

  analogWrite(enA, right_motor_speed);
  analogWrite(enB, left_motor_speed);

  // delay(50);

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
