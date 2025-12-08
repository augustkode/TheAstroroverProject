#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>
#include "UltrasonicSensor.h"

// init av pwm driver og ultrasonisk sensor
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
UltrasonicSensor ultrasonicSensorRight(1, 2);
UltrasonicSensor ultrasonicSensorLeft(3, 4);

//hjul aliaser
#define FRONT_LEFT_IN1    6
#define FRONT_LEFT_IN2    7
#define FRONT_RIGHT_IN1   4
#define FRONT_RIGHT_IN2   5
#define BACK_LEFT_IN1     2
#define BACK_LEFT_IN2     3
#define BACK_RIGHT_IN1    0
#define BACK_RIGHT_IN2    1

//minimum PWM for å unngå piping, dødsone.
#define MIN_PWM 100

// Forenkla motorstyring
void setMotor(int in1, int in2, int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    pwm.setPWM(in1, 0, speed * 16);
    pwm.setPWM(in2, 0, 0);
  } else if (speed < 0) {
    pwm.setPWM(in1, 0, 0);
    pwm.setPWM(in2, 0, (-speed) * 16);
  } else {
    pwm.setPWM(in1, 0, 0);
    pwm.setPWM(in2, 0, 0);
  }
}

// retningsfunksjoner

void stopAll() {
  setMotor(FRONT_LEFT_IN1,  FRONT_LEFT_IN2,  0);
  setMotor(FRONT_RIGHT_IN1, FRONT_RIGHT_IN2, 0);
  setMotor(BACK_LEFT_IN1,   BACK_LEFT_IN2,   0);
  setMotor(BACK_RIGHT_IN1,  BACK_RIGHT_IN2,  0);
}


void forward(int speed) {
  setMotor(FRONT_LEFT_IN1,  FRONT_LEFT_IN2,  speed);
  setMotor(FRONT_RIGHT_IN1, FRONT_RIGHT_IN2, speed);
  setMotor(BACK_LEFT_IN1,   BACK_LEFT_IN2,   speed);
  setMotor(BACK_RIGHT_IN1,  BACK_RIGHT_IN2,  speed);
}

void backward(int speed) { forward(-speed); }

void left(int speed) {
  setMotor(FRONT_LEFT_IN1,  FRONT_LEFT_IN2,  speed);
  setMotor(FRONT_RIGHT_IN1, FRONT_RIGHT_IN2, -speed);
  setMotor(BACK_LEFT_IN1,   BACK_LEFT_IN2,   -speed);
  setMotor(BACK_RIGHT_IN1,  BACK_RIGHT_IN2,  speed);
}

void right(int speed) {
  setMotor(FRONT_LEFT_IN1,  FRONT_LEFT_IN2,  -speed);
  setMotor(FRONT_RIGHT_IN1, FRONT_RIGHT_IN2,  speed);
  setMotor(BACK_LEFT_IN1,   BACK_LEFT_IN2,   speed);
  setMotor(BACK_RIGHT_IN1,  BACK_RIGHT_IN2, -speed);
}

// diagonaler
void forwardRight(int speed) {
  setMotor(FRONT_RIGHT_IN1, FRONT_RIGHT_IN2, speed);
  setMotor(BACK_LEFT_IN1,   BACK_LEFT_IN2,   speed);
}

void forwardLeft(int speed) {
  setMotor(FRONT_LEFT_IN1,  FRONT_LEFT_IN2,  speed);
  setMotor(BACK_RIGHT_IN1,  BACK_RIGHT_IN2,  speed);
}

void backwardRight(int speed) {
  setMotor(FRONT_LEFT_IN1,  FRONT_LEFT_IN2, -speed);
  setMotor(BACK_RIGHT_IN1,  BACK_RIGHT_IN2, -speed);
}

void backwardLeft(int speed) {
  setMotor(FRONT_RIGHT_IN1, FRONT_RIGHT_IN2, -speed);
  setMotor(BACK_LEFT_IN1,   BACK_LEFT_IN2,   -speed);
}

//rotasjon
void rotateCW(int speed) {
  setMotor(FRONT_LEFT_IN1,  FRONT_LEFT_IN2,  speed);
  setMotor(FRONT_RIGHT_IN1, FRONT_RIGHT_IN2, speed);
  setMotor(BACK_LEFT_IN1,   BACK_LEFT_IN2,  -speed);
  setMotor(BACK_RIGHT_IN1,  BACK_RIGHT_IN2, -speed);
}

void rotateCCW(int speed) { rotateCW(-speed); }

//hjelpefunksjon for joystickverdier, kartlegger 0-3000 til 0-255 med dødsone
int toSpeed(int raw) {
  int s = map(abs(raw), 0, 3000, 0, 255);
  if (s > 0 && s < MIN_PWM) s = MIN_PWM;
  if (raw < 0) s = -s;
  return s;
}


//hovedprogram
void setup() {
  Serial.begin(115200);
  pwm.begin();
  pwm.setPWMFreq(800);
  stopAll();
}

void loop() {
  if (!Serial.available()) return;

  float distanceRight = ultrasonicSensorRight.measureDistanceUltrasonicCM();
  float distanceLeft = ultrasonicSensorLeft.measureDistanceUltrasonicCM();

  // debug ultrasonisk.
  Serial.print("Distance Right: ");
  Serial.print(distanceRight);
  Serial.print(" cm, ");


  // Parser kjørekommando fra motor_driver node, i formatet KJØRERETNING-HASTIGHET, F, B, L, R, FR, FL, BR, BL, CW, CCW, S. For eksempel F-1500, for halv fart fremover.

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();

  int sep = cmd.indexOf('-');
  if (sep == -1) {
    stopAll();
    return;
  }

  String action = cmd.substring(0, sep);
  int value = cmd.substring(sep + 1).toInt();
  int speed = toSpeed(value);

  // debug ultrasonisk
  Serial.print(" Distance Right: ");
  Serial.println(distanceRight);
  Serial.print(" Distance Left: ");
  Serial.print(distanceLeft);

  // debug kodee
  Serial.print("Action=");
  Serial.print(action);
  Serial.print(" Speed=");
  Serial.println(speed);

  // utfører handling basert på kommando parset, forward og forward left/right sjekker avstand først for å hindre kollisjon.
  if      (action == "F"  && (distanceRight > 5 || distanceLeft > 5)) forward(speed);
  else if (action == "B")   backward(speed);
  else if (action == "L")   left(speed);
  else if (action == "R")   right(speed);
  else if (action == "FR" && (distanceRight > 5 || distanceLeft > 5)) forwardRight(speed);
  else if (action == "FL" && (distanceRight > 5 || distanceLeft > 5)) forwardLeft(speed);
  else if (action == "BR")  backwardRight(speed);
  else if (action == "BL")  backwardLeft(speed);
  else if (action == "CW")  rotateCW(speed);
  else if (action == "CCW") rotateCCW(speed);
  else if (action == "S")   stopAll();
  else                      stopAll();
}
