#include "DepthControl.h"
#include <math.h>
#include "Printer.h"
extern Printer printer;

DepthControl::DepthControl(void)
: DataSource("uV,depth,depth_des","float,float,float"){}


void DepthControl::init(const int totalWayPoints_in, double * wayPoints_in, int diveDelay_in) {
  totalWayPoints = totalWayPoints_in;
  // create wayPoints array on the Heap so that it isn't erased once the main Arduino loop starts
  wayPoints = new double[totalWayPoints];
  for (int i=0; i<totalWayPoints; i++) {
    wayPoints[i] = wayPoints_in[i];
  }
  diveDelay = diveDelay_in;
}

void DepthControl::dive(z_state_t * state, int currentTime_in) {
  currentTime = currentTime_in;

  updatePoint(state->z);
  if (atDepth || currentWayPoint == totalWayPoints) return; // at final depth point

  // compute control effort based on error between current depth and currently desired depth (P-control)
  depth_des = wayPoints[currentWayPoint];
  depth = state->z;
  depth_error = depth_des - depth;

  // deadband: suppress control effort when error is within the filtered noise floor
  if (fabs(depth_error) < DEPTH_DEADBAND) {
    uV = 0;
  } else {
    uV = depth_error * Kp;
  }

  // bound the control effort
  if (uV > MAX_CONTROL_EFFORT) uV = MAX_CONTROL_EFFORT;
  else if (uV < -1 * MAX_CONTROL_EFFORT) uV = -1 * MAX_CONTROL_EFFORT;
  
  ///////////////////////////////////////////////////////////////////////
  // don't change code past this point
  ///////////////////////////////////////////////////////////////////////

}

void DepthControl::surface(z_state_t * state) {
  depth_des = 0;
  depth = state->z;

  String surfaceMessage = "";
  int smTime = 20;
  if (fabs(depth - depth_des) < DEPTH_MARGIN || delayed) {
    atSurface = 1;
    complete = 1;
    uV = 0;
    surfaceMessage = "Got to surface. Finished Depth Control";
    smTime = 10;
  }
  else { // not at surface yet
    atSurface = 0;
    uV = -30; // go upward TODO fine-tune this value
  }
  printer.printMessage(surfaceMessage,smTime);
}

String DepthControl::printString(void) {
  String printString = "";
  if (!diveState && !surfaceState) {
    printString += "DepthControl: Not in dive or surface state";
  }
  else {
    printString += "DepthControl: ";
    printString += "Depth_Des: ";
    printString += String(depth_des);
    printString += "[m], ";
    printString += "Depth: ";
    printString += String(depth);
    printString += "[m], ";
    printString += "uV: ";
    printString += String(uV);
  }
  return printString;
}

String DepthControl::printWaypointUpdate(void) {
  String wayPointUpdate = "";
  if (!diveState && !surfaceState) {
    wayPointUpdate += "DepthControl: Not in dive or surface state";
  }
  else if (delayed) {
    wayPointUpdate += "DepthControl: Waiting for delay";
  }
  else {
    wayPointUpdate += "DepthControl: ";
    wayPointUpdate += "Current Waypoint: ";
    wayPointUpdate += String(currentWayPoint);
    wayPointUpdate += "; Distance from Waypoint: ";
    wayPointUpdate += String(depth_error);
    wayPointUpdate += "[m]";
  }
  return wayPointUpdate;
}

void DepthControl::updatePoint(float z) {
  if (currentWayPoint == totalWayPoints) return; // don't check if finished

  float z_des = wayPoints[currentWayPoint];
  depth_error = fabs((float)z_des - z); // was abs() — integer abs() truncates floats < 1 m to 0
  
  if ((depth_error < DEPTH_MARGIN && currentWayPoint < totalWayPoints) || delayed) {
    String changingWPMessage = "";
    int cwpmTime = 20;

    // dive delay
    if (delayStartTime == 0) delayStartTime = currentTime;
    if (currentTime < delayStartTime + diveDelay) {
      delayed = 1;
      changingWPMessage = "Got to depth waypoint " + String(currentWayPoint)
        + ", waiting until delay is over";
    }
    else {
      delayed = 0;
      delayStartTime = 0;
      changingWPMessage = "Got to depth waypoint " + String(currentWayPoint)
        + ", now directing to next point";
      currentWayPoint++;
    } 
    if (currentWayPoint == totalWayPoints) {
      changingWPMessage = "Got to final depth waypoint. Now surfacing";
      atDepth = 1;
      uV = 0;
      cwpmTime = 10;
      currentWayPoint = 0; // reset so index is valid if atDepth is ever cleared; atDepth is never cleared in normal operation
    }
    printer.printMessage(changingWPMessage,cwpmTime);
  }
}

size_t DepthControl::writeDataBytes(unsigned char * buffer, size_t idx) {
  float * data_slot = (float *) &buffer[idx];
  data_slot[0] = uV;
  data_slot[1] = depth;
  data_slot[2] = depth_des;
  return idx + 3*sizeof(float);
}
