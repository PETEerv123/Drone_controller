#ifndef _PID_H
#define _PID_H
#include "Arduino.h"


class PID{
  public:
    float KpPitch = 0.1;
    float KiPitch = 0.01;
    float KdPitch = 0.05;
    const int PID_PITCH_RANGE = 500;

    float KpRoll = 0.1;
    float KiRoll = 0.01;
    float KdRoll = 0.05;
    const int PID_ROLL_RANGE = 500;

    float kpThrottle = 6; // cho mạnh chút
    float kdThrottle = 0;
    float kiThrottle = 0.0;
    const int PID_ALTITUDE_RANGE = 800;

    float KpYaw = 0.5;
    float KiYaw = 0.0;
    float KdYaw = 0.0;
    const int PID_HEADING_RANGE = 500;
    void altitude_hold(int targetAltitude, int current_Atitude,float* data_u) ;
    void heading_hold(float targethold, float current_angle,float* data_u);
    void target(int targetx, int targety,int current_x,int current_y,int* data_x,int* data_y );
};
#endif