#ifndef __ATTITUDECONTROL_H__
#define __ATTITUDECONTROL_H__
#include <Arduino.h>
#include <SensorIMU.h>
#include "DataSource.h"

#define MAX_PITCH_EFFORT 180 // Max differential effort; tune later
#define PITCH_DEADBAND 2 // degrees of error that we consider "close enough" to the desired pitch; tune later

class AttitudeControl : public DataSource {
    public:
    AttitudeControl(void);

    void init(float pitch_des_in);

    //Computes per-motor commands by combining depth effort uV 
    // with. pitch correction. motorD in the front, motorC in the rear
    void computeMotorCommands(imu_state_t * imu_state, float uV, int * motorC_out, int *motorD_out);

    String printString(void);
    
    //from DataSource
    size_t writeDataBytes(unsigned char * buffer, size_t idx);

    int lastExecutionTime = -1;
    float Kp_pitch = 2.0; // pitch is in degrees, so gain can be modest. tune later (start small)

    // State for loggin
    float pitch;
    float pitch_des;
    float pitch_error;
    float uPitch;
};
#endif
