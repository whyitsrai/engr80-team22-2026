#include "ZStateEstimator.h"
#include <math.h>
#include "Printer.h"
extern Printer printer;

ZStateEstimator::ZStateEstimator(void)
  : DataSource("z","float") // from DataSource
{}

void ZStateEstimator::init(void) {
  state.z  = 0;
  bufferIdx  = 0;
  bufferFull = false;
  for (int i = 0; i < PRESSURE_FILTER_N; i++) pressureBuffer[i] = 0;
}

void ZStateEstimator::updateState(int pressure_signal) {
  // --- moving-average filter (N=16) to suppress ADC noise ---
  pressureBuffer[bufferIdx] = pressure_signal;
  bufferIdx = (bufferIdx + 1) % PRESSURE_FILTER_N;
  if (!bufferFull && bufferIdx == 0) bufferFull = true;

  int   count = bufferFull ? PRESSURE_FILTER_N : bufferIdx;
  long  sum   = 0;
  for (int i = 0; i < count; i++) sum += pressureBuffer[i];
  float filtered_signal = (float)sum / (float)count;

  // --- convert filtered TU → voltage → depth [m] ---
  float pressure_voltage = filtered_signal * (3.3f / 1023.0f);
  state.z = depthCal_slope * pressure_voltage + depthCal_intercept;

  String calibrationMessage = "Pressure (filtered): " + String(pressure_voltage, 4) + " V  →  z = " + String(state.z, 3) + " m";
  printer.printMessage(calibrationMessage, 20);
}

String ZStateEstimator::printState(void) {
  String currentState = "";
  int decimals = 2;
  currentState += "Z_State: z: ";
  currentState += String(state.z,decimals);
  currentState += "[m]";
  return currentState;
}

size_t ZStateEstimator::writeDataBytes(unsigned char * buffer, size_t idx) {
    float * data_slot = (float *) &buffer[idx];
    data_slot[0] = state.z;
    return idx + sizeof(float);
}
