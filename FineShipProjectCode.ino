#include <Servo.h>
Servo myservo;

// Rocker button switch module connections
#define pinRockerSwitch 2

// Servo motor connections
#define pinServoMotor 37

// 4-LED module connections
#define LED_GREEN 39  //IO-1
#define LED_ROSE 38   //IO-2
#define LED_AMBER 41  //IO-3
#define LED_BLUE 40   //IO-4

// LED on Breadboard connections
#define LED_WARNING 11  //Warning
#define LED_ON 10       //On-State

// Ultrasonic sensor connections
#define ECHO_FRONT 50  //Echo
#define TRIG_FRONT 51  //Trigger
#define ECHO_LEFT 46   //Echo
#define TRIG_LEFT 47   //Trigger
#define ECHO_RIGHT 42  //Echo
#define TRIG_RIGHT 43  //Trigger

// MotorLeft connections
// MotorRight connections
#define pwmDCMotorRight 9  //ENA
#define in1DCMotorRight 8  //IN1
#define in2DCMotorRight 7  //IN2
#define in1DCMotorLeft 5   //IN3
#define in2DCMotorLeft 4   //IN4
#define pwmDCMotorLeft 3   //ENB

#define DELAY_TIME 500

// distanceDangerous declaration
#define distanceDangerous 50

// variable declarations
float distanceFront;
float distanceLeft;
float distanceRight;

int rockerSwitchValue;

int angleStraightServoMotor = 130;
int angleLeftServoMotor = 170;
int angleRightServoMotor = 90;

void setup() {
  // Set rocker button switch module pin to INPUT
  pinMode(pinRockerSwitch, INPUT);

  // Set servo motor attach pin
  myservo.attach(pinServoMotor);

  // Set LED pin to OUTPUT
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_ROSE, OUTPUT);
  pinMode(LED_AMBER, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_WARNING, OUTPUT);
  pinMode(LED_ON, OUTPUT);

  // Set Ultrasonic Sensor pin
  // ECHO pin for INPUT and TRIG pin to OUTPUT
  pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT);

  // Set all 2 motors control pins to output
  pinMode(pwmDCMotorLeft, OUTPUT);
  pinMode(pwmDCMotorRight, OUTPUT);
  pinMode(in1DCMotorLeft, OUTPUT);
  pinMode(in1DCMotorRight, OUTPUT);
  pinMode(in2DCMotorLeft, OUTPUT);
  pinMode(in2DCMotorRight, OUTPUT);

  // Turn off 2 motors - Initial state
  digitalWrite(in1DCMotorLeft, LOW);
  digitalWrite(in2DCMotorLeft, LOW);
  digitalWrite(in1DCMotorRight, LOW);
  digitalWrite(in2DCMotorRight, LOW);

  // Establish serial communication
  Serial.begin(115200);
}

// Main Controlled
// Ship Operations
void loop() {
  // Turn off all of LED before operations - Initial state
  digitalWrite(LED_WARNING, LOW);  // Active HIGH
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_ROSE, HIGH);
  digitalWrite(LED_AMBER, HIGH);
  digitalWrite(LED_BLUE, HIGH);

  // Let rocket switch working and wait for turn ON
  rockerSwitch();

  if (rockerSwitchValue == LOW) {
    // LED_ON turn OFF (Green LED on Breadboard)
    digitalWrite(LED_ON, LOW);
    // Turn off 2 motors - Initial state
    dcMotorStop();

  } else if (rockerSwitchValue == HIGH) {
    // LED_ON turn ON (Green LED on Breadboard)
    digitalWrite(LED_ON, HIGH);
    // start to operations in above function
    checkAllOfUltrasonicSensor();
  }
  delay(DELAY_TIME);
}

// Check distanceDangerous in each side function
bool isDangerous(float distance) {
  if (distance < distanceDangerous) return true;
  else return false;
}

// Check all of ultrasonic sensor function
// Will check each ultrasonic sensor according to conditions on state chart.
void checkAllOfUltrasonicSensor() {
  ultrasonicSensorFront();
  ultrasonicSensorLeft();
  ultrasonicSensorRight();
  Serial.println(".");

  if (!isDangerous(distanceFront)) {
    // ALL SAFE (Not DANGER) --> LED_GREEN blynk (1st-LED on 4-LED Module) --> Could go Straight in Increased speed!
    digitalWrite(LED_GREEN, LOW);
    servoMotorStraight();
    dcMotorIncreasedSpeed();
  } else if (isDangerous(distanceFront) && !isDangerous(distanceRight) && !isDangerous(distanceLeft)) {
    // FRONT DANGER --> LED_ROSE blynk (2nd-LED on 4-LED Module) --> Could turn Left (turn Right got the same) in Decreased speed!
    digitalWrite(LED_ROSE, LOW);
    servoMotorTurnLeft();
    dcMotorDecreasedSpeed();
  } else if (isDangerous(distanceFront) && !isDangerous(distanceRight) && isDangerous(distanceLeft)) {
    // FRONT & LEFT DANGER --> LED_AMBER blynk (3rd-LED on 4-LED Module) --> Could turn Right in Decreased speed!
    digitalWrite(LED_AMBER, LOW);
    servoMotorTurnRight();
    dcMotorDecreasedSpeed();
  } else if (isDangerous(distanceFront) && isDangerous(distanceRight) && !isDangerous(distanceLeft)) {
    // FRONT & RIGHT DANGER --> LED_BLUE blynk (4th-LED on 4-LED Module) --> Could turn Left in Decreased speed!
    digitalWrite(LED_BLUE, LOW);
    servoMotorTurnLeft();
    dcMotorDecreasedSpeed();
  } else if ((isDangerous(distanceFront) && isDangerous(distanceRight) && isDangerous(distanceLeft))) {
    // FRONT & RIGHT & LEFT DANGER --> LED_WARNING blynk (Red LED on Breadboard) --> Could stop and move Backward in Decreased speed!
    digitalWrite(LED_WARNING, HIGH);
    dcMotorStop();
    delay(DELAY_TIME);
    servoMotorStraight();
    dcMotorBackward();
  }
}

// Rocker button switch module checking function
void rockerSwitch() {
  rockerSwitchValue = digitalRead(pinRockerSwitch);
}

// Ultrasonic Sensor Front side checking function
void ultrasonicSensorFront() {
  long roundTripTimeFront;

  digitalWrite(TRIG_FRONT, LOW);
  delayMicroseconds(2);

  // Emit ultrasonic wave
  digitalWrite(TRIG_FRONT, HIGH);
  delayMicroseconds(20);

  digitalWrite(TRIG_FRONT, LOW);

  // detect the round trip
  roundTripTimeFront = pulseIn(ECHO_FRONT, HIGH);

  // calculate the distance from the sensor
  distanceFront = roundTripTimeFront / 2.0 * 0.034;

  Serial.print("DistanceFront = ");
  Serial.print(distanceFront);
  Serial.println(" cm");
}

// Ultrasonic Sensor Left side checking function
void ultrasonicSensorLeft() {
  long roundTripTimeLeft;

  digitalWrite(TRIG_LEFT, LOW);
  delayMicroseconds(2);

  // Emit ultrasonic wave
  digitalWrite(TRIG_LEFT, HIGH);
  delayMicroseconds(20);

  digitalWrite(TRIG_LEFT, LOW);

  // detect the round trip
  roundTripTimeLeft = pulseIn(ECHO_LEFT, HIGH);

  // calculate the distance from the sensor
  distanceLeft = roundTripTimeLeft / 2.0 * 0.034;

  Serial.print("DistanceLeft = ");
  Serial.print(distanceLeft);
  Serial.println(" cm");
}

// Ultrasonic Sensor Right side checking function
void ultrasonicSensorRight() {
  long roundTripTimeRight;

  digitalWrite(TRIG_RIGHT, LOW);
  delayMicroseconds(2);

  // Emit ultrasonic wave
  digitalWrite(TRIG_RIGHT, HIGH);
  delayMicroseconds(20);

  digitalWrite(TRIG_RIGHT, LOW);

  // detect the round trip
  roundTripTimeRight = pulseIn(ECHO_RIGHT, HIGH);

  // calculate the distance from the sensor
  distanceRight = roundTripTimeRight / 2.0 * 0.034;

  Serial.print("DistanceRight = ");
  Serial.print(distanceRight);
  Serial.println(" cm");
}

// 1-Servo Motor controlled function
void servoMotorStraight() {
  myservo.write(angleStraightServoMotor);
  delay(DELAY_TIME);
}

void servoMotorTurnLeft() {
  myservo.write(angleLeftServoMotor);
  delay(DELAY_TIME);
}

void servoMotorTurnRight() {
  myservo.write(angleRightServoMotor);
  delay(DELAY_TIME);
}

// 2-DC Motor controlled function (Same Operations in 2-DC Motor)
void dcMotorBackward() {
  digitalWrite(in1DCMotorLeft, LOW);
  digitalWrite(in2DCMotorLeft, HIGH);
  digitalWrite(in1DCMotorRight, LOW);
  digitalWrite(in2DCMotorRight, HIGH);
  analogWrite(pwmDCMotorLeft, 102);
  analogWrite(pwmDCMotorRight, 102);
}

void dcMotorDecreasedSpeed() {
  digitalWrite(in1DCMotorLeft, HIGH);
  digitalWrite(in2DCMotorLeft, LOW);
  digitalWrite(in1DCMotorRight, HIGH);
  digitalWrite(in2DCMotorRight, LOW);
  analogWrite(pwmDCMotorLeft, 102);
  analogWrite(pwmDCMotorRight, 102);
}

void dcMotorIncreasedSpeed() {
  digitalWrite(in1DCMotorLeft, HIGH);
  digitalWrite(in2DCMotorLeft, LOW);
  digitalWrite(in1DCMotorRight, HIGH);
  digitalWrite(in2DCMotorRight, LOW);
  analogWrite(pwmDCMotorLeft, 178);
  analogWrite(pwmDCMotorRight, 178);
}

void dcMotorNormalSpeed() {
  digitalWrite(in1DCMotorLeft, HIGH);
  digitalWrite(in2DCMotorLeft, LOW);
  digitalWrite(in1DCMotorRight, HIGH);
  digitalWrite(in2DCMotorRight, LOW);
  analogWrite(pwmDCMotorLeft, 127);
  analogWrite(pwmDCMotorRight, 127);
}

void dcMotorStop() {
  digitalWrite(in1DCMotorLeft, HIGH);
  digitalWrite(in2DCMotorLeft, HIGH);
  digitalWrite(in1DCMotorRight, HIGH);
  digitalWrite(in2DCMotorRight, HIGH);
}