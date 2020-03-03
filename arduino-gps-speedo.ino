/*
  Arduino GPS speedometer using Switec X25 stepper motor.
  Available to download from https://github.com/iggy16/arduino-gps-speedo

  License: The MIT License (See full license at the bottom of this file)
  TinyGPSPlus is available here :https://github.com/mikalhart/TinyGPSPlus
  Switec X25 stepper motor library is availble here: https://github.com/clearwater/SwitecX25
*/

#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <SwitecX25.h>

#define STEPS (315*3) // standard X25.168 range 315 degrees at 1/3 degree steps
#define LED 13 // Satellite indicator LED
#define MAXSPEED 120 // upper limit of speedometer (in MPH)
#define MAXSTEP 784 // upper limit of gauge
#define RXPIN 2
#define TXPIN 3
#define GPSBAUD 9600
#define GPSDELAY 6000

TinyGPSPlus gps; 
SoftwareSerial ss(RXPIN, TXPIN); // The serial connection to the GPS device
SwitecX25 motor1(STEPS, 7, 5, 6, 4); // set up stepper motor, pins 7, 5, 6, 4

int MPH; //current speed in MPH 
int motorPosition; //current stepper motor position
int delayCounter = 0; // slows the stepper motor to prevent jerky needle motion
long currentMotorPosition; // the current motor position
long motorDifference; // stores the difference of the current motor position and target motor position

void setup()
{
  pinMode(LED, OUTPUT);

  motor1.zero(); // zero the stepper motor
  motor1.setPosition(MAXSTEP); // move to the other end
  motor1.updateBlocking(); // program waits until the update is complete
  motor1.zero(); // and back to zero

  Serial.begin(115200); // for debugging
  delay(GPSDELAY);  // allow the u-blox receiver to come up
  ss.begin(GPSBAUD); // start the comms with the GPS Rx  
}

void loop()
{
  while (ss.available()) { //While GPS message received,
    if (gps.encode(ss.read())) { // if the GPS messaged received is ready
      processGPS();
      break;
    }
  }
   
  if (MPH >= 5) { // if speed is more than 3 MPH, set the target position on the motor
    motorPosition = map(MPH, 0, MAXSPEED, 1, MAXSTEP);
    motor1.setPosition(motorPosition);
  } else { // go straight to zero.
    motorPosition = 1; // don't go quite to zero... prevents potential issue damaging the motor.
    motor1.setPosition(motorPosition);
    motor1.updateBlocking();
    currentMotorPosition = motorPosition; // reset target vs. current position.
  }

  /* Debug using Serial plotter *
  Serial.print(MPH);
  Serial.print(" "); 
  Serial.println(gps.satellites.value());
  /* *** */

  /* Debug using Serial Monitor */
  Serial.print("Motor Position:");
  Serial.print(motorPosition);
  Serial.print(" MPH:");
  Serial.print(MPH);
  Serial.print(" Satellites:"); 
  Serial.println(gps.satellites.value());
  /* *** */

  /* Satellite indicator  
   * TODO - add variable brightness depending on number of satellites
   */
  if (gps.satellites.value() > 3) { 
    digitalWrite(LED, HIGH);
  } else {
    digitalWrite(LED, LOW); // no satellites found
  }

  if (delayCounter <= 0) { // if delay counter is 0, update the motor position.
    updateMotor();
  }

  delayCounter--;
}

// get information from GPS module
void processGPS() 
{
  if (gps.speed.isValid()){ // is there valid data to query
    MPH = gps.speed.mph(); // get the latest speed
  }
}

// update the motor's position
void updateMotor()
{
  if (currentMotorPosition < motorPosition) {
    currentMotorPosition++;
  } else if (currentMotorPosition > motorPosition) {
    currentMotorPosition--;
  }

  motorDifference = motorPosition - currentMotorPosition; // calculates the difference between the current and target position, and modifies the delay to suit.
  motorDifference = abs(motorDifference);

  //vary the delay based on the change of motor position
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
