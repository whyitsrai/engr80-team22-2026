// In summer 2021 the motherboard had to be updated with new H-bridges. The new H-bridges
// needed to be driven by an H-bridge driver IN1 - Direction, IN2 - SPEED, rather than
// the Half-bridge driver for the old ones, switch IN1 & IN2 for PWM and LOW to change
// direction. The code below, along with the .h file and the Pinouts.h file were changed.
// Erik Spjut August 2021.
#include "MotorDriver.h"
#include "Printer.h"
extern Printer printer;

MotorDriver::MotorDriver()
: DataSource("motorA,motorB,motorC,motorD","int,int,int,int")
{
  // Only zero the software state here. Hardware writes are deferred to init()
  // because this constructor runs at static init before setup() — pins are not
  // configured as OUTPUT yet and calling analogWrite/digitalWrite here is undefined
  // behavior on Teensy 4.x.
  for (int m = 0; m < NUM_MOTORS; m++) {
    motorValues[m] = 0;
    pwmValues[m] = 0;
    pwmDir[m] = 0;
  }
}

void MotorDriver::init(void) {
  for (int m = 0; m < NUM_MOTORS; m++) {
    pinMode(motorPins[m][SPEED_PIN], OUTPUT);
    pinMode(motorPins[m][DIRECTION_PIN], OUTPUT);
    // Now that pins are OUTPUT, drive them to a known-safe zero state.
    digitalWrite(motorPins[m][DIRECTION_PIN], 0);
    analogWrite(motorPins[m][SPEED_PIN], 0);
  }
}

void MotorDriver::apply(void)
{
  // determine direction and magnitude of spin required:
  for (int m = 0; m < NUM_MOTORS; m++) {
    int signedCmd = motorValues[m] * motorPolarity[m];
    pwmDir[m] = (signedCmd >= 0);
    pwmValues[m] = (signedCmd < 0) ? -signedCmd : signedCmd;
    if (signedCmd) { // correct for deadzone if not zero
      pwmValues[m] = pwmValues[m] - MOTOR_DEADZONE*pwmValues[m]/255 + MOTOR_DEADZONE;
    }
  }

  // write this information to motors
  for (int m = 0; m < NUM_MOTORS; m++) { // using pwmDir as 0 or 1
    digitalWrite(motorPins[m][DIRECTION_PIN], pwmDir[m]);
    analogWrite(motorPins[m][SPEED_PIN], pwmValues[m]);
  }
}

void MotorDriver::drive(int motorA_power,int motorB_power,int motorC_power,int motorD_power) {
  motorValues[MOTOR_A_INDEX] = motorA_power;
  motorValues[MOTOR_B_INDEX] = motorB_power;
  motorValues[MOTOR_C_INDEX] = motorC_power;
  motorValues[MOTOR_D_INDEX] = motorD_power;
  apply();
  // removed: printState() return value was always discarded, wasting a String allocation per drive call
}

String MotorDriver::printState(void) {
  String printString =
    "Motors: PWMA: "  + String(pwmDir[MOTOR_A_INDEX] ? " " : "-") + String( pwmValues[MOTOR_A_INDEX] ) +  
           " PWMB: "  + String(pwmDir[MOTOR_B_INDEX] ? " " : "-") + String( pwmValues[MOTOR_B_INDEX] ) +  
           " PWMC: "  + String(pwmDir[MOTOR_C_INDEX] ? " " : "-") + String( pwmValues[MOTOR_C_INDEX] ) +  
           " PWMD: "  + String(pwmDir[MOTOR_D_INDEX] ? " " : "-") + String( pwmValues[MOTOR_D_INDEX] )  ; 
  return printString;
}

size_t MotorDriver::writeDataBytes(unsigned char * buffer, size_t idx) {
  int * data_slot = (int *) &buffer[idx];
  for (int i = 0; i < NUM_MOTORS; i++) {
    data_slot[i] = (pwmDir[i] ? 1 : -1) * pwmValues[i]; // pwmDir[i]*  
  }
  return idx + NUM_MOTORS*sizeof(int);
}


