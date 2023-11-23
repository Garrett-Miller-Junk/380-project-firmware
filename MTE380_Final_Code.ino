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

#define OPEN_VALUE 30
#define CLOSED_VALUE 130
// Program Logic
bool start_program = false;
bool start_calibrate = false;

bool start_grab = false;
bool start_release = false;
bool stop_run = false;

// BANG BANG Values
const int DIFFERENTIAL = 20;
const int TURN_TIME = 12;
int left_motor_speed;
int right_motor_speed;
int base_speed = 90;

int counter = TURN_TIME;


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

  // IR Sensor Array
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){30, 16, 17, 19, 20, 21, PIN_A8, PIN_A9}, SensorCount);
  qtr.setEmitterPin(31);

  
}

void loop() {
  Sense_Colours left_colour, right_colour = RED;

  while (!start_program) {
    start_program = !digitalRead(PC13);
  }

  Serial.println("----- START PROGRAM -----");

  analogWrite(enA, base_speed);
  analogWrite(enB, base_speed);

  while (start_program) {

    do {
      Bang_Bang_line_follow(tcs, &left_colour, &right_colour, &counter);
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
      Bang_Bang_line_follow(tcs, &left_colour, &right_colour, &counter);
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
      Bang_Bang_line_follow(tcs, &left_colour, &right_colour, &counter);
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

void Bang_Bang_line_follow(Adafruit_TCS34725 tcs, Sense_Colours *left_colour, Sense_Colours *right_colour, int *counter) {
  if(*counter == 0){
    analogWrite(enA, base_speed);
    analogWrite(enB, base_speed);
    *counter = -1;
  } else if (*counter > 0){
    *counter = *counter - 1;
  }
  
  *left_colour = getColour(tcs, S_LEFT);
  *right_colour = getColour(tcs, S_RIGHT);
  if(*left_colour == GREEN) {
    analogWrite(enA, (base_speed + DIFFERENTIAL));
    analogWrite(enB, (base_speed - DIFFERENTIAL));
    *counter = TURN_TIME;
  }
  if(*right_colour == GREEN) {
    analogWrite(enA, (base_speed - DIFFERENTIAL));
    analogWrite(enB, (base_speed + DIFFERENTIAL));
    *counter = TURN_TIME;
  }

  
/*
  Serial.print(left_motor_speed);
  Serial.print(" / ");
  Serial.print(right_motor_speed);
  Serial.print("\t");
  Serial.print(*left_colour);
  Serial.print(" / ");
  Serial.println(*right_colour);
*/
/*
  analogWrite(enA, right_motor_speed);
  analogWrite(enB, left_motor_speed);
*/
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
