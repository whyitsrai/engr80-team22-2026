#ifndef __DEPTHCONTROL_H__
#define __DEPTHCONTROL_H__

#define DEPTH_MARGIN 0.20 // depth margin in meters
// Noise budget: raw ADC noise ≈ 40 TU pk-pk → ±0.76 m unfiltered.
// ZStateEstimator applies N=16 moving average → filtered noise ≈ ±0.19 m.
// DEPTH_MARGIN must exceed the filtered half-amplitude; 0.20 m provides minimal margin.
#define MAX_CONTROL_EFFORT 180
#define DEPTH_DEADBAND 0.20 // deadband in meters — matches DEPTH_MARGIN (filtered noise floor ±0.19 m)

#include <Arduino.h>
#include "MotorDriver.h"
#include "ZStateEstimator.h"
extern MotorDriver motorDriver;

/* 
 * Note that the direction of the motors (as in which way is positive) ought to be defined somewhere
 * like here. In the case of surfacing, uV is hard-coded to be negative as of now. This would imply
 * that a negative voltage produces an upward movement (which is the case of the propellers facing
 * upwards is be a clockwise rotation). This is achieved by wiring the motors + to + and - to -
 *
 * As a rule: positive uV causes a downwards movement, negative uV causes a upwards movement
 * TODO consider changing this after Lab7. Remove these lines if we do so.
*/

class DepthControl : public DataSource
{
public:
  DepthControl(void);

  // defines the waypoints used for Depth Control
  void init(const int totalWayPoints_in, double * wayPoints_in, int diveDelay_in);

  // sets the vertical motor effort using P-Control when diving
  void dive(z_state_t * state, int currentTime_in);

  // sets the vertical motor effort to surface
  void surface(z_state_t * state);

  String printString(void);

  String printWaypointUpdate(void);

  // from DataSource
  size_t writeDataBytes(unsigned char * buffer, size_t idx);

  int lastExecutionTime = -1;

  // control fields
  float depth_des;   // desired depth
  float depth;       // current depth
  float depth_error; // distance to waypoint
  float Kp=500.0;     // proportional control gain — lowered from 1000 for gentler descent
  float uV;          // vertical motor effort

  bool diveState = 1;
  bool surfaceState = 0;
  bool atDepth = false;
  bool atSurface = false ;
  bool complete = false;

  int totalWayPoints;
  double * wayPoints;

private:

  // updates current waypoint if necessary
  void updatePoint(float z);

  int currentWayPoint = 0;
  
  int diveDelay;
  int delayStartTime = 0;
  int currentTime;
  bool delayed;
};

#endif
