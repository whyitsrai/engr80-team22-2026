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
#include <ADC.h>
#include <AnalogBufferDMA.h>
//#include <Watchdog_t4.h>
#include <Pinouts.h>
#include <TimingOffsets.h>
#include <SensorGPS.h>
#include <SensorIMU.h>
#include <XYStateEstimator.h>
#include <ZStateEstimator.h>
#include <ErrorFlagSampler.h>
#include <MotorDriver.h>
#include <Logger.h>
#include <Printer.h>
//#include <SurfaceControl.h>
#include <DepthControl.h>
#include <AttitudeControl.h>
#define UartSerial Serial1
#include <GPSLockLED.h>
#include <Adafruit_AS726x.h>
#include <ADCSampler.h>

/////////////////////////* Global Variables *////////////////////////

//WDT_T4<WDT1> watchdog;

MotorDriver motor_driver;
XYStateEstimator xy_state_estimator;
ZStateEstimator z_state_estimator;
//SurfaceControl surface_control;
DepthControl depth_control;
AttitudeControl attitude_control;
SensorGPS gps;
Adafruit_GPS GPS(&UartSerial);
ErrorFlagSampler ef;
SensorIMU imu;
Logger logger;
ADCSampler adcvals;
Printer printer;
GPSLockLED led;
Adafruit_AS726x ams;
ADC adc;

// loop start recorder
int loopStartTime;
int currentTime;
int current_way_point = 0;
volatile bool EF_States[NUM_FLAGS] = { 1, 1, 1 };

// GPS Waypoints
//const int number_of_waypoints = 2;
//const int waypoint_dimensions = 2;       // waypoints are set to have two pieces of information, x then y.
//double waypoints [] = { 0, 10, 0, 0 };   // listed as x0,y0,x1,y1, ... etc.

int diveDelay = 10000; // how long robot will stay at depth waypoint before continuing (ms)
const int num_depth_waypoints = 4;
double depth_waypoints [] = { 0.25, 1, 0.5, 0 };  // listed as z0,z1,... etc.

// Color Sensor Channels

class AS7262Sampler : public DataSource {
public:
  AS7262Sampler() : DataSource("AS7262Violet,AS7262Blue,AS7262Green,AS7262Yellow,AS7262Orange,AS7262Red", 
                               "uint16,uint16,uint16,uint16,uint16,uint16") {}
  String printState();
  size_t writeDataBytes(unsigned char * buffer, size_t idx);
  uint16_t sensorValues[AS726x_NUM_CHANNELS]; // Vio, Blu, Gre, Yel, Ora, Red

//private:
  //uint16_t sensorValues[AS726x_NUM_CHANNELS]; // Vio, Blu, Gre, Yel, Ora, Red
  // add interfaces later
};

AS7262Sampler as7262_sampler;
bool amsConnected = false; //Initially sets connected status to false to avoid getting false positive and crashing the sensor


// TODO currently not using DMA. Either remove or fix
DMAMEM static uint16_t pressureBuf[2]; // For Direct Memory Access
DMAMEM static uint16_t temperatureBuf[2];

AnalogBufferDMA dmaPressure(pressureBuf, 1);
AnalogBufferDMA dmaTemperature(temperatureBuf, 1);

float filteredPressure = 0;
const float alpha = 0.05;

////////////////////////* Setup *////////////////////////////////

void setup() {
  //WDT_timings_t config;
  //config.timeout=100; //Setting timeout to 100 seconds
  //watchdog.begin(config);

  AS726X_BUS.begin();  // ensure that i2c is in a known state
  AS726X_BUS.setClock(100000);

  logger.include(&imu);
  //logger.include(&gps);
  //logger.include(&surface_control);
  logger.include(&depth_control);
  logger.include(&attitude_control);
  logger.include(&motor_driver);
  logger.include(&ef);
  logger.include(&imu);
  logger.include(&imu);
  logger.include(&adcvals);
  logger.include(&as7262_sampler); // there is some type BS to debug here
  logger.init();


  printer.init();
  Serial.println("Successfully initiated printer :)");
  ef.init();
  imu.init();
  Serial.println("Successfully initiated IMU :)");
  UartSerial.begin(9600);
  gps.init(&GPS);
  motor_driver.init();
  led.init();

  unsigned long amsStart = millis(); //Keeps track of how long the light sensor has been running in ms. i.e. current time

  while (millis() - amsStart < 2000) {
    if (ams.begin((&AS726X_BUS))) {
      amsConnected = true;
      Serial.println("Successfully connected to color sensor :)");
      break;  //tries to connect. if connection sucessful, then breaks loop
    }
    Serial.println("waiting for AMS begin");
    delay(1000); // blocking
  }
  // checking if sensor never connected in the first place
  if (!amsConnected) {
    Serial.println("AMS color sensor failed to connect. Skipping...");
  }

  adc.adc0->calibrate();
  adc.adc1->calibrate();

  // adc0 for pressure only
  adc.adc0->setAveraging(32);
  adc.adc0->setResolution(12);
  adc.adc0->setConversionSpeed(ADC_CONVERSION_SPEED::LOW_SPEED);
  adc.adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::LOW_SPEED);
  //adc.adc0->startContinuous(PRESSURE_PIN);
  //dmaPressure.init(&adc, ADC_0);

  // adc1 for temperature and cold junction if needed
  adc.adc1->setAveraging(16);
  adc.adc1->setResolution(12);
  adc.adc1->setConversionSpeed(ADC_CONVERSION_SPEED::LOW_SPEED);
  adc.adc1->setSamplingSpeed(ADC_SAMPLING_SPEED::LOW_SPEED);
  //adc.adc1->startContinuous(THERMISTOR_PIN);
  //dmaTemperature.init(&adc, ADC_1);

  //surface_control.init(number_of_waypoints, waypoints, DELAY);

  xy_state_estimator.init();
  z_state_estimator.init();

  Serial.println("Waiting Before Dive");
  delay(10000);
  //delay(70000);
  Serial.println("Initializing Depth Control");
  depth_control.init(num_depth_waypoints, depth_waypoints, diveDelay);

  printer.printMessage("Starting main loop", 10);
  loopStartTime = millis();
  printer.lastExecutionTime = loopStartTime - LOOP_PERIOD + PRINTER_LOOP_OFFSET;
  imu.lastExecutionTime = loopStartTime - LOOP_PERIOD + IMU_LOOP_OFFSET;
  gps.lastExecutionTime = loopStartTime - LOOP_PERIOD + GPS_LOOP_OFFSET;
  ef.lastExecutionTime = loopStartTime - LOOP_PERIOD + ERROR_FLAG_LOOP_OFFSET;
  adcvals.lastExecutionTime                = loopStartTime - LOOP_PERIOD + ADC_LOOP_OFFSET;
  //state_estimator.lastExecutionTime = loopStartTime - LOOP_PERIOD + XY_STATE_ESTIMATOR_LOOP_OFFSET;
  //surface_control.lastExecutionTime = loopStartTime - LOOP_PERIOD + SURFACE_CONTROL_LOOP_OFFSET;
  //attitude_control.init(0.0f);
  //attitude_control.lastExecutionTime = loopStartTime - LOOP_PERIOD + DEPTH_CONTROL_LOOP_OFFSET;
  depth_control.lastExecutionTime = loopStartTime - LOOP_PERIOD + DEPTH_CONTROL_LOOP_OFFSET;
  logger.lastExecutionTime = loopStartTime - LOOP_PERIOD + LOGGER_LOOP_OFFSET;
  //Petting the dog
  //watchdog.feed();
}



//////////////////////////////* Loop */////////////////////////

void loop() {
  currentTime = millis();

  if (currentTime - printer.lastExecutionTime > LOOP_PERIOD) {
    printer.lastExecutionTime = currentTime;
    printer.printValue(0,depth_control.printString());
    printer.printValue(1,ef.printStates());
    printer.printValue(2,logger.printState());
    printer.printValue(3,gps.printState());   
    printer.printValue(4,depth_control.printWaypointUpdate());
    printer.printValue(5,depth_control.printString());
    //printer.printValue(5,z_state_estimator.printState());  
    printer.printValue(6,as7262_sampler.printState());
    printer.printValue(7,motor_driver.printState());
    printer.printValue(8,imu.printRollPitchHeading());        
    printer.printValue(9,imu.printAccels());
    //printer.printValue(10,print_temperature_status(temperatureBuf[0], adc.adc1->analogRead(THERMOCOUPLE_PIN)));
    printer.printValue(10,print_temperature_status(adc.adc1->readSingle(), 0));
    //printer.printValue(11,print_pressure_status(pressureBuf[0]));
    //printer.printValue(11,print_pressure_status(adc.adc0->analogRead(PRESSURE_PIN)));
    printer.printValue(11,print_pressure_status(filteredPressure));
    printer.printToSerial();  // To stop printing, just comment this line out
  }

  /* ROBOT CONTROL Finite State Machine Taken from dive code */
  if ( currentTime-depth_control.lastExecutionTime > LOOP_PERIOD ) {
    depth_control.lastExecutionTime = currentTime;
    if ( depth_control.diveState ) {      // DIVE STATE //
      depth_control.complete = false;
      if ( !depth_control.atDepth ) {
        depth_control.dive(&z_state_estimator.state, currentTime);
      }
      else {
        depth_control.diveState = false; 
        depth_control.surfaceState = true;
      }
      motor_driver.drive(0,0,depth_control.uV,depth_control.uV);
    }
    else if ( depth_control.surfaceState ) {     // SURFACE STATE //
      if ( !depth_control.atSurface ) { 
        depth_control.surface(&z_state_estimator.state);
      }
      else if ( depth_control.complete ) { 
        delete[] depth_control.wayPoints;   // destroy depth waypoint array from the Heap
      }
      motor_driver.drive(0,0,depth_control.uV,depth_control.uV);
    }
  }

//  int courseStartTime = 10000; // delay before running program
//  int holdTime = 1000; // how long to hold at the surface before diving down
//  int diveTime = 3000; // how long do we dive down for before we traverse
//  int traverseTime = 5000; // how long do we traverse for before we surface
//  int surfaceTime = 15000; // how long do we surface for before cutting off our motors
//  int courseMotorPowerMag = 60; // what absolute amount of power to spin the motor to
//
//  if (currentTime < courseStartTime) {
//  } else if (currentTime < courseStartTime + holdTime) {
//    motor_driver.drive(0,0,courseMotorPowerMag,courseMotorPowerMag);
//  } else if (currentTime < courseStartTime + holdTime + diveTime) {
//    motor_driver.drive(0,0,-courseMotorPowerMag,-courseMotorPowerMag);
//  } else if (currentTime < courseStartTime + holdTime + diveTime + traverseTime) {
//    motor_driver.drive(courseMotorPowerMag,courseMotorPowerMag,0,0);
//  } else if (currentTime < courseStartTime + holdTime + diveTime + traverseTime + surfaceTime) {
//    motor_driver.drive(0,0,courseMotorPowerMag,courseMotorPowerMag);
//  } else if (currentTime < courseStartTime + holdTime + diveTime) {
//  } else {
//    motor_driver.drive(0,0,0,0);
//  }


  if (currentTime - ef.lastExecutionTime > LOOP_PERIOD) {
    ef.lastExecutionTime = currentTime;
    attachInterrupt(digitalPinToInterrupt(ERROR_FLAG_A), EFA_Detected, LOW);
    attachInterrupt(digitalPinToInterrupt(ERROR_FLAG_B), EFB_Detected, LOW);
    attachInterrupt(digitalPinToInterrupt(ERROR_FLAG_C), EFC_Detected, LOW);
    //delay(5); // no idea why this is here
    detachInterrupt(digitalPinToInterrupt(ERROR_FLAG_A));
    detachInterrupt(digitalPinToInterrupt(ERROR_FLAG_B));
    detachInterrupt(digitalPinToInterrupt(ERROR_FLAG_C));
    ef.updateStates(EF_States[0], EF_States[1], EF_States[2]);
    EF_States[0] = 1;
    EF_States[1] = 1;
    EF_States[2] = 1;
  }

  if (currentTime - imu.lastExecutionTime > LOOP_PERIOD) {
    imu.lastExecutionTime = currentTime;
    imu.read();     // blocking I2C calls
    if (amsConnected == true && ams.dataReady()) { // add the logger stuff and make sure that this does not run too often
    // Log current colors
      ams.readRawValues(as7262_sampler.sensorValues); // blocking i2c call
      ams.startMeasurement(); // blocking
    }
  }

  gps.read(&GPS);  // blocking UART calls, need to check for UART every cycle

  if (currentTime - led.lastExecutionTime > LOOP_PERIOD) {
    led.lastExecutionTime = currentTime;
    led.flashLED(&gps.state);
  }

  if (currentTime - logger.lastExecutionTime > LOOP_PERIOD && logger.keepLogging) {
    logger.lastExecutionTime = currentTime;
    logger.log();
  }

  if ( currentTime-z_state_estimator.lastExecutionTime > LOOP_PERIOD ) {
    z_state_estimator.lastExecutionTime = currentTime;
    z_state_estimator.updateState(filteredPressure);
  }

  if ( currentTime-adcvals.lastExecutionTime > LOOP_PERIOD ) {
    adcvals.lastExecutionTime = currentTime;
    adcvals.updateSample(); 
  }


  updatePressure(adc.adc0->readSingle());
  adc.adc0->startSingleRead(PRESSURE_PIN);
  adc.adc1->startSingleRead(THERMISTOR_PIN);
  //watchdog.feed();
}

void updatePressure(uint16_t sample) {
  filteredPressure = filteredPressure * (1.0 - alpha) + sample * alpha;
}

void EFA_Detected(void) {
  EF_States[0] = 0;
}

void EFB_Detected(void) {
  EF_States[1] = 0;
}

void EFC_Detected(void) {
  EF_States[2] = 0;
}

String print_temperature_status(int thermistor, int thermocouple) {
  String status = "";
  status += "Thermocouple Value: ";
  status += String(thermocouple);
  status += "/4095   ";
  status += "Thermistor Value: ";
  status += String(thermistor);
  status += "/4095";
  return status;
}
String print_pressure_status(int pressure) {
  String status = "";
  status += "Pressure Value: ";
  status += String(pressure);
  status += "/4095   ";
  return status;
}

String AS7262Sampler::printState() {
  String status = "";
  status += "Violet: ";
  status += String(sensorValues[AS726x_VIOLET]);
  status += "  ";
  status += "Blue: ";
  status += String(sensorValues[AS726x_BLUE]);
  status += "  ";
  status += "Green: ";
  status += String(sensorValues[AS726x_GREEN]);
  status += "  ";
  status += "Yellow: ";
  status += String(sensorValues[AS726x_YELLOW]);
  status += "  ";
  status += "Orange: ";
  status += String(sensorValues[AS726x_ORANGE]);
  status += "  ";
  status += "Red: ";
  status += String(sensorValues[AS726x_RED]);
  return status;
}

size_t AS7262Sampler::writeDataBytes(unsigned char * buffer, size_t idx) {
  uint16_t * data_slot = (uint16_t *) &buffer[idx];

  for (int i = 0; i < AS726x_NUM_CHANNELS; i++) {
    data_slot[i] = sensorValues[i];
  }

  return idx + AS726x_NUM_CHANNELS * sizeof(uint16_t);
}
