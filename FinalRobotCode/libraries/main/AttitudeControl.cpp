# include "AttitudeControl.h"
#include <math.h>
#include "Printer.h"
extern Printer printer;

float pitch = 0;
float pitch_des = 0;
float pitch_error = 0;
float uPitch = 0;

AttitudeControl::AttitudeControl(void)
    : DataSource("uPitch,pitch,pitch_des", "float,float,float") {}
void AttitudeControl::init(float pitch_des_in) {
  pitch_des = pitch_des_in; // should be ideallly close to zero for level attitude
}

void AttitudeControl::computeMotorCommands(imu_state_t * imu_state, float uV, int * motorC_out, int * motorD_out) {
    pitch = imu_state->pitch;
    pitch_error = pitch_des - pitch;
    //deadbands: if the error is small enough, consider it zero to avoid unnecessary motor effort and oscillation
    if (fabs(pitch_error) < PITCH_DEADBAND) {
        uPitch = 0;
    } else {
        uPitch = Kp_pitch * pitch_error;
    }
    // Saturate the control command to be within the limits of the motor commands
    if (uPitch > MAX_PITCH_EFFORT) uPitch = MAX_PITCH_EFFORT;
    else if (uPitch < -MAX_PITCH_EFFORT) uPitch = -MAX_PITCH_EFFORT;
    //Mix: fornt (D) gets uV + Pitch, rear (C) gets uV - uPitch
    float cmdD = uV + uPitch;
    float cmdC = uV - uPitch;
    // Clamp the motor commands to be within the valid range (0 to 255 for PWM)
    if (cmdC > 255) cmdC = 255; else if (cmdC < -255 ) cmdC = -255;
    if (cmdD > 255) cmdD = 255; else if (cmdD < -255 ) cmdD = -255;
    *motorD_out = (int) cmdD;
    *motorC_out = (int) cmdC;
}

String AttitudeControl::printString(void) {
    String s = "AttitudeControl: ";
    s += "pitch_des: " + String(pitch_des) + " [deg], ";
    s += "pitch: " + String(pitch) + " [deg], ";
    s += "error: " + String(pitch_error) + " [deg], ";
    s += "uPitch: " + String(uPitch);
    return s;
}

size_t AttitudeControl::writeDataBytes(unsigned char * buffer, size_t idx) {
float * data_slot = (float *) & buffer[idx];   
data_slot[0] = uPitch;
data_slot[1] = pitch;
data_slot[2] = pitch_des;
return idx + 3 * sizeof(float);
}