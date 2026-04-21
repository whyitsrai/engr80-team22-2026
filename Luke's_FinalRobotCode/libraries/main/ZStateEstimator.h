#ifndef __Z_STATE_ESTIMATOR_H__
#define __Z_STATE_ESTIMATOR_H__

#include <Arduino.h>

#include "DataSource.h"

typedef struct {
    float z = 0; // z position (depth) in global frame [m]
} z_state_t;

/*
 * ZStateEstimator class keeps track of the robot's depth state,
 * incorperating measurements of the system outputs from the Pressure
 * Sensor, as well as the control inputs to the system
 */
class ZStateEstimator : public DataSource
{
public:
  ZStateEstimator(void);

  // init
  void init(void);

  // State Access
  z_state_t state;

  void updateState(int pressure_signal);
  String printState(void);

  // from DataSource
  size_t writeDataBytes(unsigned char * buffer, size_t idx);

  int lastExecutionTime = -1;

private:
  // Pressure sensor calibration: depth [m] = slope * V_pressure + intercept
  // 2-point bench cal, Team 22 unit, 2026:
  //   0.0 m → 2.867 V  (atmospheric, tube open)
  //   0.4 m → 879 TU / 2.8355 V
  //.  0.0 m = 2.75 V
  //   0.5 = 2.69 V
  //   1.0 m = 2.65 V

  //   nitial Calibatrion Curve using graduate cylinder in E80 room
  //   slope     = (0.4 − 0) / (2.8355 − 2.867) = −12.69 m/V
  //   intercept = 0 − (−12.69 × 2.867)          = 36.39 m

  //  Tank Room Calibration Cals
  //  slope = (0.5 − 0) / (2.69 − 2.75) = -8.33 m/V
  //  intercept = 0 − (−8.33 × 2.75)            = 22.91 m

  // Phake Lake & Dana Point Calibration Cals (preliminary, to be done prior to grabbing data)
  
  const float depthCal_slope     = -8.33f; // m/V  (negative: higher voltage → shallower)
  const float depthCal_intercept =  22.91f; // m

  // Moving-average filter to suppress ADC noise (~40 TU pk-pk → ±0.76 m raw)
  // N=16 at 99 ms/sample reduces noise to ±0.19 m (adds ~0.8 s lag)
  static const int PRESSURE_FILTER_N = 16;
  int  pressureBuffer[PRESSURE_FILTER_N];
  int  bufferIdx;
  bool bufferFull;

};

#endif
