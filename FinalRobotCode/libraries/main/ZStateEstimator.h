#ifndef __Z_STATE_ESTIMATOR_H__
#define __Z_STATE_ESTIMATOR_H__

#include <Arduino.h>

#include "DataSource.h"

typedef struct {
    float z = 0; // z position (depth) in globad frame [m]
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
  // Derived from Team 22 bench calibration (6-point, 0–1 m):
  //   TU-domain regression: depth = -0.03801 * TU + 32.18   (R² ≈ 1)
  //   Converting to voltage domain (V = TU * 3.3/1023):
  //     slope  = -0.03801 * (1023/3.3) = -11.78 m/V
  //     intercept = 32.18 m
  // Verification: 0 m → 846 TU / 2.730 V → 0.03 m ✓   1 m → 820 TU / 2.646 V → 1.01 m ✓
  const float depthCal_slope     = -11.78f; // m/V  (negative: higher voltage → shallower)
  const float depthCal_intercept =  32.18f; // m

  // Moving-average filter to suppress ADC noise (~40 TU pk-pk → ±0.76 m raw)
  // N=16 at 99 ms/sample reduces noise to ±0.19 m (adds ~0.8 s lag)
  static const int PRESSURE_FILTER_N = 16;
  int  pressureBuffer[PRESSURE_FILTER_N];
  int  bufferIdx;
  bool bufferFull;

};

#endif
