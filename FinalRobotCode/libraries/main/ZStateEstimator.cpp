#include "ZStateEstimator.h"
#include <math.h>
#include "Printer.h"
extern Printer printer;

ZStateEstimator::ZStateEstimator(void)
  : DataSource("z","float") // from DataSource
{}

void ZStateEstimator::init(void) {
  state.z  = 0;
}

void ZStateEstimator::updateState(int pressure_signal) {
  state.z = depthCal_slope * pressure_signal + depthCal_intercept; // convert from Volts to depth [m]
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
