/********
E80 2026 Team 22 - Attitude Controller Test Sketch
Standalone test for AttitudeControl module.

What this does:
  - Reads IMU
  - Runs AttitudeControl with uV = 0 (no net vertical thrust)
  - Drives only vertical motors C (rear) and D (front); horizontal A, B are off
  - Prints pitch, pitch_des, uPitch, and motor commands so you can tune Kp_pitch

Test stages (see code comments for toggles):
  Stage 1 - DRY BENCH: leave MOTORS_ENABLED = false. Tilt the AUV by hand,
            watch the printed commands. Verify signs before powering motors.
  Stage 2 - DRY BENCH w/ motors: set MOTORS_ENABLED = true with props OFF or
            propellers removed. Confirm motor direction/magnitude response.
  Stage 3 - IN WATER: props on, tether the AUV, hold underwater, release with
            a tilt, observe restoring behavior. Tune KP_PITCH_TEST.
  Stage 4 - Add constant uV (UV_TEST) to simulate depth-control effort and
            confirm attitude still holds level while descending/ascending.
********/

#include <Arduino.h>
#include <Wire.h>
#include <Pinouts.h>
#include <SensorIMU.h>
#include <MotorDriver.h>
#include <AttitudeControl.h>
#include <Printer.h>

// ---------------- Test configuration ----------------
#define MOTORS_ENABLED true  // false = print only, don't command motors
#define PITCH_DES_DEG 0.0f   // target pitch angle [deg]
#define KP_PITCH_TEST 2.0f   // override of AttitudeControl::Kp_pitch
#define UV_TEST 0            // common-mode vertical effort (-255..255). \
                             // Keep 0 for pure attitude test. Try 40-80 \
                             // later to simulate descent.
#define LOOP_PERIOD_MS 20    // 50 Hz control loop
#define PRINT_PERIOD_MS 100  // 5 Hz serial print
// ----------------------------------------------------

MotorDriver motor_driver;
SensorIMU imu;
AttitudeControl attitude_control;
Printer printer;

unsigned long lastControlTime = 0;
unsigned long lastPrintTime = 0;

void setup() {
  printer.init();
  Serial.begin(115200);
  Serial.println("=== AttitudeControl Test Sketch ===");
  Serial.print("  MOTORS_ENABLED = ");
  Serial.println(MOTORS_ENABLED ? "true" : "false");
  Serial.print("  pitch_des      = ");
  Serial.println(PITCH_DES_DEG);
  Serial.print("  Kp_pitch       = ");
  Serial.println(KP_PITCH_TEST);
  Serial.print("  uV (constant)  = ");
  Serial.println(UV_TEST);

  motor_driver.init();
  imu.init();
  attitude_control.init(PITCH_DES_DEG);
  attitude_control.Kp_pitch = 2.0;  // override default gain for tuning

  // Safety: make sure motors are stopped before we start
  motor_driver.drive(0, 0, 0, 0);

  printer.printMessage("Starting attitude test loop", 10);
  lastControlTime = millis();
  lastPrintTime = millis();
}

void loop() {
  unsigned long now = millis();

  // ---------------- Control loop (50 Hz) ----------------
  if (now - lastControlTime >= LOOP_PERIOD_MS) {
    lastControlTime = now;

    // 1. Read IMU
    imu.read();

    // 2. Compute motor commands from attitude controller
    int motorC_cmd = 0;  // rear vertical
    int motorD_cmd = 0;  // front vertical
    attitude_control.computeMotorCommands(&imu.state,(float)UV_TEST,&motorC_cmd,&motorD_cmd);

    // 3. Drive motors (A, B horizontal kept at zero for this test)
    if (MOTORS_ENABLED) {
      motor_driver.drive(0, 0, motorC_cmd, motorD_cmd);
    } else {
      // Still update internal motor state so printState() reflects what
      // WOULD be sent, but don't actually write PWM.
      motor_driver.motorValues[0] = 0;
      motor_driver.motorValues[1] = 0;
      motor_driver.motorValues[2] = motorC_cmd;
      motor_driver.motorValues[3] = motorD_cmd;
    }
  }

  // ---------------- Print loop (5 Hz) ----------------
  if (now - lastPrintTime >= PRINT_PERIOD_MS) {
    lastPrintTime = now;

    printer.printValue(0, imu.printRollPitchHeading());
    printer.printValue(1, imu.printAccels());
    printer.printValue(2, attitude_control.printString());
    printer.printValue(3, motor_driver.printState());
    printer.printValue(4, String("Motors enabled: ") + (MOTORS_ENABLED ? "YES" : "NO"));
    printer.printToSerial();
  }
}
