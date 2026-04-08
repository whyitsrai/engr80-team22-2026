/********
 * E80 Motor Test — All 4 Motors (with Motor D focus)
 * 
 * Tests each motor individually (A, B, C, D) then all together.
 * Motor D uses pins 30 (speed) and 32 (direction) on the Teensy.
 * 
 * IMPORTANT: Before uploading, fix the drive() declaration in MotorDriver.h:
 *   void drive(int motorA_power, int motorB_power, int motorC_power, int motorD_power);
 *   (The .h currently declares 3 args but the .cpp defines 4.)
 * 
 * Sequence:
 *   Stage 0: All off (3s)
 *   Stage 1: Motor A forward at half speed (3s)
 *   Stage 2: Motor A reverse at half speed (3s)
 *   Stage 3: All off (2s pause)
 *   Stage 4: Motor B forward at half speed (3s)
 *   Stage 5: Motor B reverse at half speed (3s)
 *   Stage 6: All off (2s pause)
 *   Stage 7: Motor C forward at half speed (3s)
 *   Stage 8: Motor C reverse at half speed (3s)
 *   Stage 9: All off (2s pause)
 *   Stage 10: Motor D forward at half speed (3s)
 *   Stage 11: Motor D reverse at half speed (3s)
 *   Stage 12: Motor D forward FULL speed (3s)
 *   Stage 13: Motor D reverse FULL speed (3s)
 *   Stage 14: All off (2s pause)
 *   Stage 15: All motors forward at half speed (3s)
 *   Stage 16: All motors reverse at half speed (3s)
 *   Stage 17: All off — done
 * 
 * Authors: E80 Team 22, 2026
 ********/

#include <Arduino.h>
#include <Wire.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include <Pinouts.h>
#include <MotorDriver.h>
#include <Printer.h>

///////////////////////// Global Variables ////////////////////////

MotorDriver motor_driver;
Printer printer;

int loopStartTime;
int currentTime;

// Test configuration
const int HALF_SPEED = 127;
const int FULL_SPEED = 255;
const int RUN_TIME   = 3000;  // ms per motor test stage
const int PAUSE_TIME = 2000;  // ms pause between motors

////////////////////////// Setup ////////////////////////////////

void setup() {
  printer.init();
  motor_driver.init();

  Serial.println("=== E80 Motor Test: All 4 Motors ===");
  Serial.println("Motor A: pins " + String(MOTOR_A_DIRECTION) + " (dir), " + String(MOTOR_A_SPEED) + " (spd)");
  Serial.println("Motor B: pins " + String(MOTOR_B_DIRECTION) + " (dir), " + String(MOTOR_B_SPEED) + " (spd)");
  Serial.println("Motor C: pins " + String(MOTOR_C_DIRECTION) + " (dir), " + String(MOTOR_C_SPEED) + " (spd)");
  Serial.println("Motor D: pins " + String(MOTOR_D_DIRECTION) + " (dir), " + String(MOTOR_D_SPEED) + " (spd)");
  Serial.println("Starting in 3 seconds...");
  delay(3000);

  loopStartTime = millis();
}

///////////////////////////// Loop ///////////////////////////////

void loop() {
  currentTime = millis();

  static unsigned long stageStart = 0;
  static int stage = 0;
  static bool stagePrinted = false;

  if (stageStart == 0) stageStart = currentTime;

  unsigned long elapsed = currentTime - stageStart;

  // Determine what to do in each stage
  switch (stage) {
    case 0:  // Initial pause — all off
      if (!stagePrinted) { Serial.println("[Stage 0] All motors OFF — initial pause"); stagePrinted = true; }
      motor_driver.drive(0, 0, 0, 0);
      if (elapsed >= PAUSE_TIME) { stage++; stageStart = currentTime; stagePrinted = false; }
      break;

    // ---- Motor A ----
    case 1:
      if (!stagePrinted) { Serial.println("[Stage 1] Motor A FORWARD @ half speed"); stagePrinted = true; }
      motor_driver.drive(HALF_SPEED, 0, 0, 0);
      if (elapsed >= RUN_TIME) { stage++; stageStart = currentTime; stagePrinted = false; }
      break;
    case 2:
      if (!stagePrinted) { Serial.println("[Stage 2] Motor A REVERSE @ half speed"); stagePrinted = true; }
      motor_driver.drive(-HALF_SPEED, 0, 0, 0);
      if (elapsed >= RUN_TIME) { stage++; stageStart = currentTime; stagePrinted = false; }
      break;
    case 3:
      if (!stagePrinted) { Serial.println("[Stage 3] Pause"); stagePrinted = true; }
      motor_driver.drive(0, 0, 0, 0);
      if (elapsed >= PAUSE_TIME) { stage++; stageStart = currentTime; stagePrinted = false; }
      break;

    // ---- Motor B ----
    case 4:
      if (!stagePrinted) { Serial.println("[Stage 4] Motor B FORWARD @ half speed"); stagePrinted = true; }
      motor_driver.drive(0, HALF_SPEED, 0, 0);
      if (elapsed >= RUN_TIME) { stage++; stageStart = currentTime; stagePrinted = false; }
      break;
    case 5:
      if (!stagePrinted) { Serial.println("[Stage 5] Motor B REVERSE @ half speed"); stagePrinted = true; }
      motor_driver.drive(0, -HALF_SPEED, 0, 0);
      if (elapsed >= RUN_TIME) { stage++; stageStart = currentTime; stagePrinted = false; }
      break;
    case 6:
      if (!stagePrinted) { Serial.println("[Stage 6] Pause"); stagePrinted = true; }
      motor_driver.drive(0, 0, 0, 0);
      if (elapsed >= PAUSE_TIME) { stage++; stageStart = currentTime; stagePrinted = false; }
      break;

    // ---- Motor C ----
    case 7:
      if (!stagePrinted) { Serial.println("[Stage 7] Motor C FORWARD @ half speed"); stagePrinted = true; }
      motor_driver.drive(0, 0, HALF_SPEED, 0);
      if (elapsed >= RUN_TIME) { stage++; stageStart = currentTime; stagePrinted = false; }
      break;
    case 8:
      if (!stagePrinted) { Serial.println("[Stage 8] Motor C REVERSE @ half speed"); stagePrinted = true; }
      motor_driver.drive(0, 0, -HALF_SPEED, 0);
      if (elapsed >= RUN_TIME) { stage++; stageStart = currentTime; stagePrinted = false; }
      break;
    case 9:
      if (!stagePrinted) { Serial.println("[Stage 9] Pause"); stagePrinted = true; }
      motor_driver.drive(0, 0, 0, 0);
      if (elapsed >= PAUSE_TIME) { stage++; stageStart = currentTime; stagePrinted = false; }
      break;

    // ---- Motor D (extended test) ----
    case 10:
      if (!stagePrinted) { Serial.println("[Stage 10] Motor D FORWARD @ half speed"); stagePrinted = true; }
      motor_driver.drive(0, 0, 0, HALF_SPEED);
      if (elapsed >= RUN_TIME) { stage++; stageStart = currentTime; stagePrinted = false; }
      break;
    case 11:
      if (!stagePrinted) { Serial.println("[Stage 11] Motor D REVERSE @ half speed"); stagePrinted = true; }
      motor_driver.drive(0, 0, 0, -HALF_SPEED);
      if (elapsed >= RUN_TIME) { stage++; stageStart = currentTime; stagePrinted = false; }
      break;
    case 12:
      if (!stagePrinted) { Serial.println("[Stage 12] Motor D FORWARD @ FULL speed"); stagePrinted = true; }
      motor_driver.drive(0, 0, 0, FULL_SPEED);
      if (elapsed >= RUN_TIME) { stage++; stageStart = currentTime; stagePrinted = false; }
      break;
    case 13:
      if (!stagePrinted) { Serial.println("[Stage 13] Motor D REVERSE @ FULL speed"); stagePrinted = true; }
      motor_driver.drive(0, 0, 0, -FULL_SPEED);
      if (elapsed >= RUN_TIME) { stage++; stageStart = currentTime; stagePrinted = false; }
      break;
    case 14:
      if (!stagePrinted) { Serial.println("[Stage 14] Pause"); stagePrinted = true; }
      motor_driver.drive(0, 0, 0, 0);
      if (elapsed >= PAUSE_TIME) { stage++; stageStart = currentTime; stagePrinted = false; }
      break;

    // ---- All motors together ----
    case 15:
      if (!stagePrinted) { Serial.println("[Stage 15] ALL motors FORWARD @ half speed"); stagePrinted = true; }
      motor_driver.drive(HALF_SPEED, HALF_SPEED, HALF_SPEED, HALF_SPEED);
      if (elapsed >= RUN_TIME) { stage++; stageStart = currentTime; stagePrinted = false; }
      break;
    case 16:
      if (!stagePrinted) { Serial.println("[Stage 16] ALL motors REVERSE @ half speed"); stagePrinted = true; }
      motor_driver.drive(-HALF_SPEED, -HALF_SPEED, -HALF_SPEED, -HALF_SPEED);
      if (elapsed >= RUN_TIME) { stage++; stageStart = currentTime; stagePrinted = false; }
      break;

    // ---- Done ----
    case 17:
      if (!stagePrinted) {
        motor_driver.drive(0, 0, 0, 0);
        Serial.println("[DONE] All motors OFF. Test complete.");
        stagePrinted = true;
      }
      break;
  }

  // Print motor state to serial every loop period
  static unsigned long lastPrint = 0;
  if (currentTime - lastPrint > 500) {  // print every 500ms
    lastPrint = currentTime;
    Serial.println(motor_driver.printState());
  }
}
