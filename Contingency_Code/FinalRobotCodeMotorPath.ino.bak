/********
E80 2026 Team 22 Submersible Code
Default Code Authors:
    Wilson Ives (wives@g.hmc.edu) '20 (contributed in 2018)
    Christopher McElroy (cmcelroy@g.hmc.edu) '19 (contributed in 2017)
    Josephine Wong (jowong@hmc.edu) '18 (contributed in 2016)
    Apoorva Sharma (asharma@hmc.edu) '17 (contributed in 2016)
This Iteration's Authors:
    Rai Wandeler (rwandeler@hmc.edu) '28 (contributed in 2026)
    Brave Yongphiphatwong (byongphiphatwong@hmc.edu) '28 (contributed in 2026)
*/

#include <Arduino.h>              // Core Arduino functions
#include <Wire.h>                 // I2C communication
#include <avr/io.h>               // AVR low-level IO
#include <avr/interrupt.h>        // Enable/handle interrupts

#include <Pinouts.h>              // Pins for motors, sensors, LEDs
#include <TimingOffsets.h>        // Timing offsets for loop scheduling
#include <SensorGPS.h>            // GPS library
#include <SensorIMU.h>            // IMU (orientation) library
#include <XYStateEstimator.h>     // XY state estimation library
#include <ZStateEstimator.h>      // Depth/Z state estimation library
#include <ADCSampler.h>           // ADC sensor library
#include <ErrorFlagSampler.h>     // Error flag library
#include <ButtonSampler.h>        // Button input library
#include <MotorDriver.h>          // Motor driver library
#include <Logger.h>               // Logging library
#include <Printer.h>              // Print sensor states to serial
#include <DepthControl.h>         // Depth control library
#include <GPSLockLED.h>           // LED indicator for GPS lock
#include <BurstADCSampler.h>      // Burst ADC sampling library
#include <Adafruit_AS726x.h>      // Color sensor library

#define UartSerial Serial1        // Teensy UART used for GPS
#define DELAY 0                   // Default delay (not used in this code)

///////////////////////// Global Variables ////////////////////////

MotorDriver motor_driver;          // Controls thrusters
XYStateEstimator xy_state_estimator; // Position estimation in XY plane
ZStateEstimator z_state_estimator;   // Depth estimation
DepthControl depth_control;          // Depth waypoint control
SensorGPS gps;                       // GPS sensor wrapper
Adafruit_GPS GPS(&UartSerial);       // GPS object
ADCSampler adc;                       // Analog sensors sampler
ErrorFlagSampler ef;                  // Error flag sampler
ButtonSampler button_sampler;         // User button sampler
SensorIMU imu;                        // IMU object
Logger logger;                         // SD card logger
Printer printer;                       // Prints to serial
GPSLockLED led;                        // Blinking LED for GPS lock
BurstADCSampler burst_adc;             // Burst ADC sampler
Adafruit_AS726x ams;                   // Color sensor

// loop timing
int loopStartTime;                     // Tracks millis() at loop start
int currentTime;                       // Current millis() each loop
volatile bool EF_States[NUM_FLAGS] = {1,1,1}; // Error flag states

// GPS Waypoints (x,y)
const int number_of_waypoints = 2;     // Number of GPS points
const int waypoint_dimensions = 2;     // x,y only
double waypoints [] = {0,10,0,0};      // Waypoint coordinates

// Color Sensor
uint16_t sensorValues[AS726x_NUM_CHANNELS]; // Stores raw color readings

// Sensor timeouts (ms)
#define IMU_TIMEOUT      50             // Max time to wait for IMU
#define COLOR_TIMEOUT    50             // Max time to wait for color sensor

////////////////////////// Setup ////////////////////////////////

void setup() {
    // Configure analog sensor pins
    pinMode(THERMOCOUPLE_PIN, INPUT);
    pinMode(THERMISTOR_PIN, INPUT);
    pinMode(PRESSURE_PIN, INPUT);

    // Initialize I2C for color sensor
    AS726X_BUS.begin();
    AS726X_BUS.setClock(100000);

    // Include sensors and actuators in logger
    logger.include(&imu);
    logger.include(&depth_control);
    logger.include(&motor_driver);
    logger.include(&adc);
    logger.include(&ef);
    logger.include(&button_sampler);
    logger.init();                     // Initialize SD logging

    burst_adc.init();                  // Initialize burst ADC
    printer.init();                    // Initialize serial printer
    ef.init();                         // Initialize error flags
    button_sampler.init();             // Initialize button
    imu.init();                        // Initialize IMU
    UartSerial.begin(9600);            // Start serial for GPS
    gps.init(&GPS);                    // Initialize GPS
    motor_driver.init();               // Initialize motors
    led.init();                        // Initialize GPS LED

    // Wait until color sensor is detected
    while(!ams.begin(&AS726X_BUS)){
        Serial.println("waiting for AMS color sensor...");
        delay(100);
    }

    // Depth waypoints setup
    const int num_depth_waypoints = 4; 
    double depth_waypoints [] = {0.5,1}; // Example depth waypoints
    int diveDelay = 10000;               // Delay at depth
    depth_control.init(num_depth_waypoints, depth_waypoints, diveDelay);

    // Initialize state estimators
    xy_state_estimator.init(); 
    z_state_estimator.init();

    printer.printMessage("Starting main loop",10); // Print message
    loopStartTime = millis();                      // Record loop start

    // Set last execution times to offset loops correctly
    printer.lastExecutionTime         = loopStartTime - LOOP_PERIOD + PRINTER_LOOP_OFFSET ;
    imu.lastExecutionTime             = loopStartTime - LOOP_PERIOD + IMU_LOOP_OFFSET;
    gps.lastExecutionTime             = loopStartTime - LOOP_PERIOD + GPS_LOOP_OFFSET;
    adc.lastExecutionTime             = loopStartTime - LOOP_PERIOD + ADC_LOOP_OFFSET;
    ef.lastExecutionTime              = loopStartTime - LOOP_PERIOD + ERROR_FLAG_LOOP_OFFSET;
    button_sampler.lastExecutionTime  = loopStartTime - LOOP_PERIOD + BUTTON_LOOP_OFFSET;
    depth_control.lastExecutionTime   = loopStartTime - LOOP_PERIOD + DEPTH_CONTROL_LOOP_OFFSET;
    logger.lastExecutionTime          = loopStartTime - LOOP_PERIOD + LOGGER_LOOP_OFFSET;
    burst_adc.lastExecutionTime       = loopStartTime;
}

///////////////////////////// Loop ///////////////////////////////

void loop() {
    currentTime = millis();  // Update current time

    // ====================== SAWTOOTH NAVIGATION =====================
    static unsigned long navStart = 0; // Start time of current stage
    static int stage = 0;               // Current stage of sawtooth
    static int repeatCount = 0;         // How many times the pattern repeated
    unsigned long t = currentTime - navStart; // Time elapsed in stage

    // Stage durations in ms
    const int waitTime = 10000; 
    const int downTime = 4000;
    const int upDiagTime = 5000;
    const int upTime = 8000;

    if (navStart == 0) navStart = currentTime; // Initialize navStart

    // Sawtooth navigation sequence
    switch(stage){
        case 0: motor_driver.drive(0,0,0); // Wait
                if (t >= waitTime) { stage=1; navStart=currentTime; }
                break;
        case 1: motor_driver.drive(0,0,-255); // Dive
                if (t >= downTime) { stage=2; navStart=currentTime; }
                break;
        case 2: motor_driver.drive(255,255,0); // Up diagonal
                if (t >= upDiagTime) { stage=3; navStart=currentTime; }
                break;
        case 3: motor_driver.drive(0,0,-255); // Dive
                if (t >= downTime) { stage=4; navStart=currentTime; }
                break;
        case 4: motor_driver.drive(255,255,0); // Up diagonal
                if (t >= upDiagTime){
                    repeatCount++;
                    if (repeatCount<2) { stage=1; } // Repeat once more
                    else { stage=5; }               // Go to final ascent
                    navStart=currentTime;
                }
                break;
        case 5: motor_driver.drive(0,0,255); // Up straight
                if (t>=upTime){ stage=6; navStart=currentTime; }
                break;
        case 6: motor_driver.drive(0,0,0);   // Stop
                break;
    }

    // ====================== PRINT & LOG ==============================
    if (currentTime - printer.lastExecutionTime > LOOP_PERIOD) {
        printer.lastExecutionTime = currentTime;

        // Print all sensor & actuator data
        printer.printValue(0,adc.printSample());
        printer.printValue(1,ef.printStates());
        printer.printValue(2,logger.printState());
        printer.printValue(3,gps.printState());
        printer.printValue(4,xy_state_estimator.printState());
        printer.printValue(5,z_state_estimator.printState());
        printer.printValue(6, print_as7262_status(sensorValues));
        printer.printValue(7,motor_driver.printState());
        printer.printValue(8,imu.printRollPitchHeading());
        printer.printValue(9,imu.printAccels());
        printer.printValue(10,print_temperature_status(analogRead(THERMISTOR_PIN), analogRead(THERMOCOUPLE_PIN)));
        printer.printValue(11,print_pressure_status(analogRead(PRESSURE_PIN)));
        printer.printToSerial();
    }

    // ====================== ADC, Error Flags, Button =================
    if (currentTime - adc.lastExecutionTime > LOOP_PERIOD) { adc.lastExecutionTime = currentTime; adc.updateSample(); }
    if (currentTime - ef.lastExecutionTime > LOOP_PERIOD) {
        ef.lastExecutionTime = currentTime;
        attachInterrupt(digitalPinToInterrupt(ERROR_FLAG_A), EFA_Detected, LOW);
        attachInterrupt(digitalPinToInterrupt(ERROR_FLAG_B), EFB_Detected, LOW);
        attachInterrupt(digitalPinToInterrupt(ERROR_FLAG_C), EFC_Detected, LOW);
        delay(5);
        detachInterrupt(digitalPinToInterrupt(ERROR_FLAG_A));
        detachInterrupt(digitalPinToInterrupt(ERROR_FLAG_B));
        detachInterrupt(digitalPinToInterrupt(ERROR_FLAG_C));
        ef.updateStates(EF_States[0],EF_States[1],EF_States[2]);
        EF_States[0] = EF_States[1] = EF_States[2] = 1;
    }

    if (currentTime - button_sampler.lastExecutionTime > LOOP_PERIOD) { button_sampler.lastExecutionTime=currentTime; button_sampler.updateState(); }

    // ====================== IMU & COLOR SENSOR =======================
    static unsigned long imuStartTime = 0;
    static unsigned long colorStartTime = 0;

    if (currentTime - imu.lastExecutionTime > LOOP_PERIOD) {
        imu.lastExecutionTime = currentTime;

        // IMU read with timeout
        imuStartTime = millis();
        bool imuReadSuccess = false;
        while (!imuReadSuccess) {
            if (millis() - imuStartTime > IMU_TIMEOUT) { imuReadSuccess = true; printer.printMessage("IMU timeout",10); break; }
            imuReadSuccess = imu.readNonBlocking();
        }

        // Color sensor read with timeout
        if (ams.dataReady()) {
            colorStartTime = millis();
            bool colorReadSuccess = false;
            while (!colorReadSuccess) {
                if (millis() - colorStartTime > COLOR_TIMEOUT) {
                    for (int i=0;i<AS726x_NUM_CHANNELS;i++) sensorValues[i]=0;
                    colorReadSuccess=true;
                    printer.printMessage("Color sensor timeout",10);
                    break;
                }
                colorReadSuccess = ams.readRawValuesNonBlocking(sensorValues);
            }
            ams.startMeasurement(); // start next measurement
        }
    }

    gps.read(&GPS); // Update GPS

    if (currentTime - led.lastExecutionTime > LOOP_PERIOD) { led.lastExecutionTime = currentTime; led.flashLED(&gps.state); }

    if (currentTime - logger.lastExecutionTime > LOOP_PERIOD && logger.keepLogging) { logger.lastExecutionTime=currentTime; logger.log(); }
}

// ==================== ERROR FLAG INTERRUPTS ====================
void EFA_Detected(void){ EF_States[0] = 0; } // Error A detected
void EFB_Detected(void){ EF_States[1] = 0; } // Error B detected
void EFC_Detected(void){ EF_States[2] = 0; } // Error C detected

// ==================== STATUS PRINT FUNCTIONS ====================
String print_as7262_status(uint16_t* as7262Values){
    String status = "";
    const char* names[] = {"Violet","Blue","Green","Yellow","Orange","Red"};
    for(int i=0;i<AS726x_NUM_CHANNELS;i++){
        status += names[i];
        status += ": ";
        status += String(as7262Values[i]);
        status += "   ";
    }
    return status;
}

String print_temperature_status(int thermistor,int thermocouple){
    String status = "";
    status += "Thermocouple: "; status += String(thermocouple); status += "/1023   ";
    status += "Thermistor: "; status += String(thermistor); status += "/1023";
    return status;
}

String print_pressure_status(int pressure){
    String status = "";
    status += "Pressure: "; status += String(pressure); status += "/1023";
    return status;
}