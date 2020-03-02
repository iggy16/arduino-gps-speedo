/*
  GPS speedometer.
  (c) 1st November 2017 A.G.Doswell  Web Page http://andydoz.blogspot.co.uk

  Available to download from https://github.com/andydoswell/GPS-Speedo

  License: The MIT License (See full license at the bottom of this file)

  The sketch uses a U-Blox 6M GPS satellite module connected to the hardware Serial interface,
  a 128x32 SDD1306 OLED display module connected as an I2C device to pins A4 (SDA)& A5 (SCL) and a Swtech stepper motor connected to pins 4 -7. The arduino used is a Nano with 5v ATMEGA328P.

  The OzOled library is not quite "right" for this application, but works, and is lightweight and fast enough. Thanks Oscar!
  It is available here : http://blog.oscarliang.net/arduino-oled-display-library/
  TinyGPSPlus is available here :https://github.com/mikalhart/TinyGPSPlus
  Switec X25 stepper motor library is availble here: https://github.com/clearwater/SwitecX25

*/
#include <SoftwareSerial.h>
#include <Wire.h>
#include <TinyGPS++.h>
/*#include <EEPROM.h>*/
#include <SwitecX25.h>

TinyGPSPlus gps; // Feed gps data to TinySGPSPlus
SoftwareSerial ss(2, 3);
SwitecX25 motor1(945, 7, 5, 6, 4); // set up stepper motor, 945 steps, pins 4,5,6 & 7

int MPH; // Speed in mph
unsigned int motorPosition;
/*int milesUnit = 0;
  int milesTen = 0;
  int milesHundred = 0;
  int milesThousand = 0;
  int milesTenThousand = 0;
  char milesUnitA[2];
  char milesTenA[2];
  char milesHundredA[2];
  char milesThousandA[2];
  char milesTenThousandA[2];*/
double oldLat;
double oldLong;
float distanceMeters;
boolean fixFlag = false;
const int maxSpeed = 120;
const int maxStep = 784;
int delayCounter = 0;
int oldSpeed;
int acceleration;
long currentMotorPosition;
long motorDifference;
float currentDistance;
int led = 13;

void setup()
{
  pinMode(led, OUTPUT);

  // Set display to Normal mode
  motor1.zero();         // zero the stepper motor
  motor1.setPosition(maxStep); // move to the other end
  motor1.updateBlocking();
  motor1.zero(); // and back to zero

  Serial.begin(115200);
  delay(6000);  // allow the u-blox receiver to come up
  ss.begin(9600); // start the comms with the GPS Rx
  
}

void loop()
{
  while (ss.available()) { //While GPS message received,
    if (gps.encode(ss.read())) { // if the GPS messaged received is ready
      processGPS();
      break;
    }
  }
  //Serial.print("MPH: ");
   
  if (MPH >= 3) { // if speed is more than 3 MPH, set the target position on the motor
    motorPosition = map(MPH, 0, maxSpeed, 1, maxStep);
    motor1.setPosition(motorPosition);
  } else {// if the speed is ess than 3 MPH, go straight to zero.
    motorPosition = 1; // don't go quite to zero... prevents potential issue damaging the motor.
    motor1.setPosition(motorPosition);
    motor1.updateBlocking();
    currentMotorPosition = motorPosition; // reset target vs. current position.
  }
  //Serial.print("satellites:");
  Serial.print(MPH);
Serial.print(" "); 
  Serial.println(gps.satellites.value());
  if (gps.satellites.value() > 3) {
    digitalWrite(led, HIGH);
  } else {
    digitalWrite(led, LOW);
  }

  if (delayCounter <= 0) { // if delay counter is 0, update the motor position.
    updateMotor();
  }

  delayCounter--;
}

void processGPS()
{
  if (gps.location.isValid()){
    MPH = gps.speed.mph();
    /*Serial.println(MPH);
    /*if (!fixFlag) { // if this is the first time we've had a fix, the update the position
      oldLat = gps.location.lat();
      oldLong = gps.location.lng();
      //getMileage (); // re-read the EEPROM, just in case it was corrupted during cranking.
      fixFlag = true;
    } else {
      if (MPH >= 3) { // MPH must be greater than 3 to update the odometer, this prevents jitter adding to the odo.
        currentDistance = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), oldLat, oldLong); // distance between current position and the last pposition
        distanceMeters += currentDistance;
        oldLat = gps.location.lat(); // set last position to current position
        oldLong = gps.location.lng();
      }*/
    /*}
    if (distanceMeters >= 1609.34) {  // 1609.34m in a mile
      distanceMeters -= 1609.34; // subtract a mile
      if (distanceMeters < 10)
      { // this prevents a rare error which screws the mileage. (the error is the distance reported being from 0 deg lat, 0 deg long, which is fine if you're just off the coast of Africa...GPS still reports position valid, even though it isn't!))
        //incrementMileage (); // add a mile to the odo.
      }
    }*/
  }
}

void updateMotor()
{
  if (currentMotorPosition < motorPosition) {
    currentMotorPosition++;
  } else if (currentMotorPosition > motorPosition) {
    currentMotorPosition--;
  }

  motorDifference = motorPosition - currentMotorPosition; // calculates the difference between the current and target position, and modifies the delay to suit.
  motorDifference = abs(motorDifference);

  if (motorDifference > 40) {
    delayCounter = 0;
  } else if (motorDifference <= 40) {
    delayCounter = 40;
  } else if (motorDifference <= 30) {
    delayCounter = 133;
  } else if (motorDifference <= 20) {
    delayCounter = 300;
  } else if (motorDifference <= 10) {
    delayCounter = 600;
  } else if (motorDifference <= 6) {
    delayCounter = 1000;
  }
  motor1.update();
}
