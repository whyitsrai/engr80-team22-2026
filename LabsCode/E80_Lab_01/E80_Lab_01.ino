/********
Default E80 Code
Current Author:
    Wilson Ives (wives@g.hmc.edu) '20 (contributed in 2018)
Previous Contributors:
    Christopher McElroy (cmcelroy@g.hmc.edu) '19 (contributed in 2017)  
    Josephine Wong (jowong@hmc.edu) '18 (contributed in 2016)
    Apoorva Sharma (asharma@hmc.edu) '17 (contributed in 2016)                    
*/

// Set only one of them to be 1, the other one to be 0
#define MOTORTESTING 1
#define TANKTESTING 0
#define OBSTACLECOURSE 0

/* Libraries */

// general
#include <Arduino.h>
#include <Wire.h>
#include <Pinouts.h>

// E80-specific
#include <SensorIMU.h>
#include <MotorDriver.h>
#include <Logger.h>
#include <Printer.h>


/* Global Variables */

// period in ms of logger and printer
#define LOOP_PERIOD 100

// Motors
MotorDriver motorDriver;
// We set power values between -255 and 255 (with - being reversed)
// motors are [x, y, z]

// IMU
SensorIMU imu;

// Logger
Logger logger;
bool keepLogging = true;

// Printer
Printer printer;

// loop start recorder
int loopStartTime;

void setup() {
  printer.init();

  /* Initialize the Logger */
  logger.include(&imu);
  logger.include(&motorDriver);
  logger.init();

  /* Initialise the sensors */
  imu.init();

  /* Initialize motor pins */
  motorDriver.init();

  /* Keep track of time */
  printer.printMessage("Starting main loop",10);
  loopStartTime = millis();
}


void loop() {

  int currentTime = millis() - loopStartTime;
  
  ///////////  Don't change code above here! ////////////////////
  // write code here to make the robot fire its motors in the sequence specified in the lab manual 
  // the currentTime variable contains the number of ms since the robot was turned on 
  // The motorDriver.drive function takes in 3 inputs arguments motorA_power, motorB_power, motorC_power: 
  //       void motorDriver.drive(int motorA_power,int motorB_power,int motorC_power); 
  // the value of motorX_power can range from -255 to 255, and sets the PWM applied to the motor 
  // The following example will turn on motor B for four seconds between seconds 4 and 8 
  //Motor Testing
#ifdef MOTORTESTING
   if (currentTime > 4000 && currentTime <8000) {
    motorDriver.drive(120,0,0);
    Serial.println("Testing Motor 1 at speed 120");
  } else if (currentTime > 12000 && currentTime < 16000) {
    motorDriver.drive(0,120,0);
    Serial.println("Testing Motor 2 at speed 120");
  } else if (currentTime > 20000 && currentTime < 24000) {
    motorDriver.drive(0,0,120);
    Serial.println("Testing Motor 3 at speed 120");
  }
  else {
    motorDriver.drive(0,0,0);
    Serial.println("Done Testing Motors. All off.");
  }
#endif

#ifdef TANKTESTING
  // Plot motor acceleration/velocity/distance profile (extract tau)
  // Can we graph these?
  int testingStartTime = 15000; // delay before running program
  int timeMultiples = 5000; // how long to spin motors and/or coast
  int testingMotorPowerMag = 200; // what absolute amout of power to spin the motor to
  Serial.print("testingMotorPowerMag is");
  Serial.println(testingMotorPowerMag);

  if (currentTime < testingStartTime) {
    Serial.println("Waiting for 15 seconds before running all motors for a few seconds at testingMotorPowerMag duty");
    motorDriver.drive(0,0,testingMotorPowerMag);
  } else if (currentTime < testingStartTime + 1*timeMultiples) {
    Serial.println("Testing Motor 1 at speed testingMotorPowerMag for 5 seconds.");
    motorDriver.drive(testingMotorPowerMag,0,0);
  } else if (currentTime < testingStartTime + 2*timeMultiples) {
    Serial.println("All Motors off for 5 seconds.");
    motorDriver.drive(0,0,0);
  } else if (currentTime < testingStartTime + 3*timeMultiples) {
    Serial.println("Testing Motor 2 at speed testingMotorPowerMag for 5 seconds.");
    motorDriver.drive(0,testingMotorPowerMag,0);
  } else if (currentTime < testingStartTime + 4*timeMultiples) {
    Serial.println("All Motors off for 5 seconds.");
    motorDriver.drive(0,0,0);
  } else if (currentTime < testingStartTime + 5*timeMultiples) {
    Serial.println("Testing Motor 3 at speed testingMotorPowerMag for 5 seconds.");
    motorDriver.drive(0,0,testingMotorPowerMag);
  } else if (currentTime < testingStartTime + 6*timeMultiples) {
    Serial.println("All Motors off for 5 seconds.");
    motorDriver.drive(0,0,0);
  } else if (currentTime < testingStartTime + 7*timeMultiples) {
    Serial.println("Testing Motor 1 at speed testingMotorPowerMag for 5 seconds.");
    motorDriver.drive(-testingMotorPowerMag,0,0);
  } else if (currentTime < testingStartTime + 8*timeMultiples) {
    Serial.println("All Motors off for 5 seconds.");
    motorDriver.drive(0,0,0);
  } else if (currentTime < testingStartTime + 9*timeMultiples) {
    Serial.println("Testing Motor 2 at speed testingMotorPowerMag for 5 seconds.");
    motorDriver.drive(0,-testingMotorPowerMag,0);
  } else if (currentTime < testingStartTime + 10*timeMultiples) {
    Serial.println("All Motors off for 5 seconds.");
    motorDriver.drive(0,0,0);
  } else if (currentTime < testingStartTime + 11*timeMultiples) {
    Serial.println("Testing Motor 3 at speed testingMotorPowerMag for 5 seconds.");
    motorDriver.drive(0,0,-testingMotorPowerMag);
  } else if (currentTime < testingStartTime + 12*timeMultiples) {
    Serial.println("All Motors off for 5 seconds.");
    motorDriver.drive(0,0,0);
  } else if (currentTime < testingStartTime + 13*timeMultiples) {
    Serial.println("Surface");
    motorDriver.drive(0,0,255);
  } else {
    Serial.println("All Motors off, program done");
    motorDriver.drive(0,0,0);
  }
#endif

<<<<<<< HEAD
#ifdef OBSTACLECOURSE
//const int diveDepth = 2; // in feet (actually ideally 2.5)
//const int diveDistance = 6; // also in feet
// TODO tune times based on first tank test results
int courseStartTime = 15000; // delay before running program
int holdTime = 5000; // how long to hold at the surface before diving down
int diveTime = 5000; // how long do we dive down for before we traverse
int traverseTime = 5000; // how long do we traverse for before we surface
int surfaceTime = 5000; // how long do we surface for before cutting off our motors
int courseMotorPowerMag = 200; // what absolute amout of power to spin the motor to

  if (currentTime < courseStartTime) {
    Serial.println("Waiting for 15 seconds before running obstacle course program");
  } else if (currentTime < courseStartTime + holdTime) {
    Serial.println("Ensuring that we are surfaced");
    motorDriver.drive(0,0,courseMotorPowerMag);
  } else if (currentTime < courseStartTime + holdTime + diveTime) {
    Serial.println("Diving Down");
    motorDriver.drive(0,0,-courseMotorPowerMag);
  } else if (currentTime < courseStartTime + holdTime + diveTime + traverseTime) {
    Serial.println("Traversing to new hoop");
    motorDriver.drive(0,courseMotorPowerMag,0); // forward is y
  } else if (currentTime < courseStartTime + holdTime + diveTime + traverseTime + surfaceTime) {
    Serial.println("Surfacing");
    motorDriver.drive(0,0,courseMotorPowerMag);
  } else {
    Serial.println("Done. Motors Off");
    motorDriver.drive(0,0,0);
  }
//Possible Course Plan (Please make changes where you see fit!)
//if (currentTime > 10000 && currentTime <15000) {
//    motorDriver.drive(0,0,255); // 5 seconds of thrusting downward
//  } else if (currentTime > 16000 && currentTime <23000){ 
//    motorDriver.drive(255, 255, 0); // 7 seconds of thrusting forward
//  } else if (currentTime > 24000 && currentTime < 30000) {
//    motorDriver.drive(0, 0, 255);// 6 seconds of thrusting upward (extra second to account for possible loss of altitude)
//  } else if (currentTime > 31000 && currentTime < 36000) {
//    motorDriver.drive(255, -255, 0); // 5 seconds of perfoming Victory Spin manuver 
//  }
//  else {
//    motorDriver.drive(0, 0, 0);
//  }
#endif

  // DONT CHANGE CODE BELOW THIS LINE 
  // --------------------------------------------------------------------------

  
  if ( currentTime-printer.lastExecutionTime > LOOP_PERIOD ) {
    printer.lastExecutionTime = currentTime;
    printer.printValue(0,imu.printAccels());
    printer.printValue(1,imu.printRollPitchHeading());
    printer.printValue(2,motorDriver.printState());
    printer.printToSerial();  // To stop printing, just comment this line out
  }

  if ( currentTime-imu.lastExecutionTime > LOOP_PERIOD ) {
    imu.lastExecutionTime = currentTime;
    imu.read(); // this is a sequence of blocking I2C read calls
  }

  if ( currentTime-logger.lastExecutionTime > LOOP_PERIOD && logger.keepLogging) {
    logger.lastExecutionTime = currentTime;
    logger.log();
  }

}
