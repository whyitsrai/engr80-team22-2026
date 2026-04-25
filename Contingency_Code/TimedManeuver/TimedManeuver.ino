/********
 * E80 2026 Team 22 — Contingency Timed Maneuver
 *
 * Open-loop, time-based mission sequence (no sensor feedback). Useful as a
 * fallback if depth/attitude control is misbehaving.
 *
 * Sequence (each duration is independently tunable below):
 *   1. Descent #1   — fire C & D downward for DESCENT_1_SEC seconds
 *   2. Forward      — fire A & B forward for FORWARD_SEC seconds
 *   3. Descent #2   — fire C & D downward for DESCENT_2_SEC seconds
 *   4. Reverse      — fire A & B in reverse for REVERSE_SEC seconds
 *   5. Terminal asc — fire C & D upward for ASCENT_SEC seconds
 *   6. All motors off
 *
 * Sign convention (matches MotorDriver polarity):
 *   A,B positive = forward       A,B negative = reverse
 *   C,D positive = descend       C,D negative = ascend
 ********/

#include <Arduino.h>
#include <Pinouts.h>
#include <MotorDriver.h>
#include <Printer.h>

///////////////////////// Tunable Parameters ////////////////////////

// Per-leg durations, in seconds
const float DESCENT_1_SEC = 10.0f;
const float FORWARD_SEC   = 10.0f;
const float DESCENT_2_SEC = 10.0f;
const float REVERSE_SEC   = 10.0f;
const float ASCENT_SEC    = 60.0f;

// Motor power magnitudes (0–255). Must exceed MOTOR_DEADZONE (34) to spin.
const int VERTICAL_POWER   = 250;  // for C & D during descent / ascent
const int HORIZONTAL_POWER = -250;  // for A & B during forward / reverse

// Pause between legs so transitions are visible / mechanically gentle
const unsigned long PAUSE_MS = 500;

// Delay before first motor command after boot
const unsigned long STARTUP_DELAY_MS = 3000;

///////////////////////// Globals ////////////////////////

MotorDriver motor_driver;
Printer printer;

enum Stage {
  STAGE_DESCENT_1 = 0,
  STAGE_PAUSE_1,
  STAGE_FORWARD,
  STAGE_PAUSE_2,
  STAGE_DESCENT_2,
  STAGE_PAUSE_3,
  STAGE_REVERSE,
  STAGE_PAUSE_4,
  STAGE_ASCENT,
  STAGE_DONE
};

int           stage          = STAGE_DESCENT_1;
unsigned long stageStartTime = 0;
bool          stageAnnounced = false;

////////////////////////// Setup ////////////////////////////////

void setup() {
  printer.init();
  motor_driver.init();
  motor_driver.drive(0, 0, 0, 0);

  Serial.println("=== Contingency Timed Maneuver ===");
  Serial.print("Starting in "); Serial.print(STARTUP_DELAY_MS / 1000);
  Serial.println(" s...");
  delay(STARTUP_DELAY_MS);

  stageStartTime = millis();
}

///////////////////////////// Loop ///////////////////////////////

void loop() {
  unsigned long now     = millis();
  unsigned long elapsed = now - stageStartTime;

  switch (stage) {

    case STAGE_DESCENT_1:
      announce("[1] Descent #1: C & D down");
      motor_driver.drive(0, 0, VERTICAL_POWER, VERTICAL_POWER);
      advanceAfter(elapsed, (unsigned long)(DESCENT_1_SEC * 1000));
      break;

    case STAGE_PAUSE_1:
      announce("    pause");
      motor_driver.drive(0, 0, 0, 0);
      advanceAfter(elapsed, PAUSE_MS);
      break;

    case STAGE_FORWARD:
      announce("[2] Forward: A & B forward");
      motor_driver.drive(HORIZONTAL_POWER, HORIZONTAL_POWER, 0, 0);
      advanceAfter(elapsed, (unsigned long)(FORWARD_SEC * 1000));
      break;

    case STAGE_PAUSE_2:
      announce("    pause");
      motor_driver.drive(0, 0, 0, 0);
      advanceAfter(elapsed, PAUSE_MS);
      break;

    case STAGE_DESCENT_2:
      announce("[3] Descent #2: C & D down");
      motor_driver.drive(0, 0, VERTICAL_POWER, VERTICAL_POWER);
      advanceAfter(elapsed, (unsigned long)(DESCENT_2_SEC * 1000));
      break;

    case STAGE_PAUSE_3:
      announce("    pause");
      motor_driver.drive(0, 0, 0, 0);
      advanceAfter(elapsed, PAUSE_MS);
      break;

    case STAGE_REVERSE:
      announce("[4] Reverse: A & B reverse");
      motor_driver.drive(-HORIZONTAL_POWER, -HORIZONTAL_POWER, 0, 0);
      advanceAfter(elapsed, (unsigned long)(REVERSE_SEC * 1000));
      break;

    case STAGE_PAUSE_4:
      announce("    pause");
      motor_driver.drive(0, 0, 0, 0);
      advanceAfter(elapsed, PAUSE_MS);
      break;

    case STAGE_ASCENT:
      announce("[5] Terminal ascent: C & D up");
      motor_driver.drive(0, 0, -VERTICAL_POWER, -VERTICAL_POWER);
      advanceAfter(elapsed, (unsigned long)(ASCENT_SEC * 1000));
      break;

    case STAGE_DONE:
      if (!stageAnnounced) {
        motor_driver.drive(0, 0, 0, 0);
        Serial.println("[DONE] Sequence complete. All motors OFF.");
        stageAnnounced = true;
      }
      break;
  }

  // Periodic status print
  static unsigned long lastPrint = 0;
  if (now - lastPrint > 500) {
    lastPrint = now;
    Serial.println(motor_driver.printState());
  }
}

///////////////////////// Helpers ////////////////////////

void announce(const char * msg) {
  if (!stageAnnounced) {
    Serial.println(msg);
    stageAnnounced = true;
  }
}

void advanceAfter(unsigned long elapsed, unsigned long duration) {
  if (elapsed >= duration) {
    stage++;
    stageStartTime = millis();
    stageAnnounced = false;
  }
}
