/********
E80 2026 Team 22 Submersible Code
Default Code Authors:
    Wilson Ives (wives@g.hmc.edu) '20 (contributed in 2018)
    Christopher McElroy (cmcelroy@g.hmc.edu) '19 (contributed in 2017)
    Josephine Wong (jowong@hmc.edu) '18 (contributed in 2016)
    Apoorva Sharma (asharma@hmc.edu) '17 (contributed in 2016)
This Iteration's Authors:
    Rai Wandeler (rwandeler@hmc.edu) '28 (contributed in 2026)
    Alejandro Tellez (atellez@g.hmc.edu) '28 (contributed in 2026)
*/
#include <Arduino.h>
#include <Wire.h>
#include <avr/io.h>
#include <avr/interrupt.h>
//Adding watchdog timer library to use the watchdog timer later 
#include <Adafruit_SleepyDog.h>
//Adding watch 
#include <Pinouts.h>
#include <TimingOffsets.h>
#include <SensorGPS.h>
#include <SensorIMU.h>
#include <XYStateEstimator.h>
#include <ZStateEstimator.h>
#include <ADCSampler.h>
#include <ErrorFlagSampler.h>
#include <ButtonSampler.h> // A template of a data source library
#include <MotorDriver.h>
#include <Logger.h>
#include <Printer.h>
//#include <SurfaceControl.h>
#include <DepthControl.h>
#define UartSerial Serial1
#define DELAY 0
#include <GPSLockLED.h>
#include <BurstADCSampler.h>
#include <Adafruit_AS726x.h>

/////////////////////////* Global Variables *////////////////////////

MotorDriver motor_driver;
XYStateEstimator xy_state_estimator;
ZStateEstimator z_state_estimator;
//SurfaceControl surface_control;
DepthControl depth_control;
SensorGPS gps;
Adafruit_GPS GPS(&UartSerial);
ADCSampler adc;
ErrorFlagSampler ef;
ButtonSampler button_sampler;
SensorIMU imu;
Logger logger;
Printer printer;
GPSLockLED led;
BurstADCSampler burst_adc;
Adafruit_AS726x ams;

// loop start recorder
int loopStartTime;
int currentTime;
int current_way_point = 0;
volatile bool EF_States[NUM_FLAGS] = {1,1,1};

// GPS Waypoints
const int number_of_waypoints = 2;
const int waypoint_dimensions = 2;       // waypoints are set to have two pieces of information, x then y.
double waypoints [] = { 0, 10, 0, 0 };   // listed as x0,y0,x1,y1, ... etc.

// Color Sensor Channels
uint16_t sensorValues[AS726x_NUM_CHANNELS];
String print_as7262_status(uint16_t* as7262Values);

////////////////////////* Setup *////////////////////////////////

void setup() {
  pinMode(THERMOCOUPLE_PIN, INPUT);
  pinMode(THERMISTOR_PIN, INPUT);
  pinMode(PRESSURE_PIN, INPUT);

  Watchdog.enable(5000); //Watchdog timer essentially resets the system. Initiate after 5 sec

  AS726X_BUS.begin(); // ensure that i2c is in a known state
  AS726X_BUS.setClock(100000);

  logger.include(&imu);
  //logger.include(&gps);
  //logger.include(&surface_control);
  logger.include(&depth_control);
  logger.include(&motor_driver);
  logger.include(&adc);
  logger.include(&ef);
  logger.include(&button_sampler);
//  logger.include((int) &sensorValues[AS726x_VIOLET]); // there is some type BS to debug here
//  logger.include((int) &sensorValues[AS726x_BLUE]);
//  logger.include((int) &sensorValues[AS726x_GREEN]);
//  logger.include((int) &sensorValues[AS726x_YELLOW]);
//  logger.include((int) &sensorValues[AS726x_ORANGE]);
//  logger.include((int) &sensorValues[AS726x_RED]);
  logger.init();
  burst_adc.init();
  

  printer.init();
  ef.init();
  button_sampler.init();
  imu.init();
  UartSerial.begin(9600);
  gps.init(&GPS);
  motor_driver.init();
  led.init();

  //Old loop would run infinitely if something fails, so added changes below:
  unsigned long amsStart = millis(); //Keeps track of how long the light sensor has been running in ms. i.e. current time
  bool amsConnected = false; //Initially sets connected status to false to avoid getting false positive and crashing the sensor

  while(millis() - amsStart < 4000) {
    if (ams.begin((&AS726X_BUS))) {
      amsConnected = true;
      Serial.println("Successfully connected to color sensor :)");
      break;  //tries to connect. if connection sucessful, then breaks loop
    }
    Serial.println("waiting for AMS begin");
    delay(100);
  }
  // checking if sensor never connected in the first place
  if (!amsConnected) {
    Serial.println("AMS color sensor failed to connect. Skipping..."); 
  }
  
  int diveDelay = 10000; // how long robot will stay at depth waypoint before continuing (ms)

  const int num_depth_waypoints = 4;
  double depth_waypoints [] = { 0.5, 1 };  // listed as z0,z1,... etc.
  //double depth_waypoints [] = { 1.4, 1.0, 0.5, 0.8 };  // listed as z0,z1,... etc.
  depth_control.init(num_depth_waypoints, depth_waypoints, diveDelay);


  //surface_control.init(number_of_waypoints, waypoints, DELAY);
  
  xy_state_estimator.init(); 
  z_state_estimator.init();

  printer.printMessage("Starting main loop",10);
  loopStartTime = millis();
  printer.lastExecutionTime         = loopStartTime - LOOP_PERIOD + PRINTER_LOOP_OFFSET ;
  imu.lastExecutionTime             = loopStartTime - LOOP_PERIOD + IMU_LOOP_OFFSET;
  gps.lastExecutionTime             = loopStartTime - LOOP_PERIOD + GPS_LOOP_OFFSET;
  adc.lastExecutionTime             = loopStartTime - LOOP_PERIOD + ADC_LOOP_OFFSET;
  ef.lastExecutionTime              = loopStartTime - LOOP_PERIOD + ERROR_FLAG_LOOP_OFFSET;
  button_sampler.lastExecutionTime  = loopStartTime - LOOP_PERIOD + BUTTON_LOOP_OFFSET;
  //state_estimator.lastExecutionTime = loopStartTime - LOOP_PERIOD + XY_STATE_ESTIMATOR_LOOP_OFFSET;
  //surface_control.lastExecutionTime = loopStartTime - LOOP_PERIOD + SURFACE_CONTROL_LOOP_OFFSET;
  depth_control.lastExecutionTime      = loopStartTime - LOOP_PERIOD + DEPTH_CONTROL_LOOP_OFFSET;
  logger.lastExecutionTime          = loopStartTime - LOOP_PERIOD + LOGGER_LOOP_OFFSET;
  burst_adc.lastExecutionTime       = loopStartTime;
  //Petting the dog 
  Watchdog.reset();

}



//////////////////////////////* Loop */////////////////////////

void loop() {
  currentTime=millis();
  
  if ( currentTime-printer.lastExecutionTime > LOOP_PERIOD ) {
    printer.lastExecutionTime = currentTime;
    printer.printValue(0,adc.printSample());
    printer.printValue(1,ef.printStates());
    printer.printValue(2,logger.printState());
    printer.printValue(3,gps.printState());   
    printer.printValue(4,xy_state_estimator.printState());  
    printer.printValue(5,z_state_estimator.printState());  
    printer.printValue(6, print_as7262_status(sensorValues)); // Color Information
    printer.printValue(7,motor_driver.printState());
    printer.printValue(8,imu.printRollPitchHeading());        
    printer.printValue(9,imu.printAccels());
    printer.printValue(10,print_temperature_status(analogRead(THERMISTOR_PIN), analogRead(THERMOCOUPLE_PIN)));
    printer.printValue(11,print_pressure_status(analogRead(PRESSURE_PIN))); // TODO change pin
    printer.printToSerial();  // To stop printing, just comment this line out
  }


  if ( currentTime-adc.lastExecutionTime > LOOP_PERIOD ) {
    adc.lastExecutionTime = currentTime;
    adc.updateSample(); 
  }

  if ( currentTime-ef.lastExecutionTime > LOOP_PERIOD ) {
    ef.lastExecutionTime = currentTime;
    attachInterrupt(digitalPinToInterrupt(ERROR_FLAG_A), EFA_Detected, LOW);
    attachInterrupt(digitalPinToInterrupt(ERROR_FLAG_B), EFB_Detected, LOW);
    attachInterrupt(digitalPinToInterrupt(ERROR_FLAG_C), EFC_Detected, LOW);
    delay(5);
    detachInterrupt(digitalPinToInterrupt(ERROR_FLAG_A));
    detachInterrupt(digitalPinToInterrupt(ERROR_FLAG_B));
    detachInterrupt(digitalPinToInterrupt(ERROR_FLAG_C));
    ef.updateStates(EF_States[0],EF_States[1],EF_States[2]);
    EF_States[0] = 1;
    EF_States[1] = 1;
    EF_States[2] = 1;
  }

 // uses the ButtonSampler library to read a button -- use this as a template for new libraries!
  if ( currentTime-button_sampler.lastExecutionTime > LOOP_PERIOD ) {
    button_sampler.lastExecutionTime = currentTime;
    button_sampler.updateState();
  }

  if ( currentTime-imu.lastExecutionTime > LOOP_PERIOD ) {
    imu.lastExecutionTime = currentTime;
    imu.read();     // blocking I2C calls
    if (ams.dataReady()) { // add the logger stuff and make sure that this does not run too often
  // Log current colors
      ams.readRawValues(sensorValues); // blocking i2c call
      ams.startMeasurement();
    }
  }
 
  gps.read(&GPS); // blocking UART calls, need to check for UART every cycle

  if ( currentTime-led.lastExecutionTime > LOOP_PERIOD ) {
    led.lastExecutionTime = currentTime;
    led.flashLED(&gps.state);
  }

  if ( currentTime- logger.lastExecutionTime > LOOP_PERIOD && logger.keepLogging ) {
    logger.lastExecutionTime = currentTime;
    logger.log();
  }
  //"Petting the dog" - Reset the watchdog timer after ensuring everything in the loop is working
  Watchdog.reset(); 
}

void EFA_Detected(void){
  EF_States[0] = 0;
}

void EFB_Detected(void){
  EF_States[1] = 0;
}

void EFC_Detected(void){
  EF_States[2] = 0;
}

String print_as7262_status(uint16_t* as7262Values) {
  String status = "";
  status += "Violet: ";
  status += String(as7262Values[AS726x_VIOLET]);
  status += "   ";
  status += "Blue: ";
  status += String(as7262Values[AS726x_BLUE]);
  status += "   ";
  status += "Green: ";
  status += String(as7262Values[AS726x_GREEN]);
  status += "   ";
  status += "Yellow: ";
  status += String(as7262Values[AS726x_YELLOW]);
  status += "   ";
  status += "Orange: ";
  status += String(as7262Values[AS726x_ORANGE]);
  status += "   ";
  status += "Red: ";
  status += String(as7262Values[AS726x_RED]);
  status += "   ";
  return status;
}

String print_temperature_status(int thermistor, int thermocouple) {
  String status = "";
  status += "Thermocouple Value: ";
  status += String(thermocouple);
  status += "/1023   ";
  status += "Thermistor Value: ";
  status += String(thermistor);
  status += "/1023";
  return status;
}
String print_pressure_status(int pressure) {
  String status = "";
  status += "Pressure Value: ";
  status += String(pressure);
  status += "/1023   ";
  return status;
}
