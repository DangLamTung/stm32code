#ifndef PID_H_
#define PID_H_

double Kp_pitch, Ki_pitch, Kd_pitch;
double Kp_roll, Ki_roll, Kd_roll;

float dt;
float P_pitch,I_pitch,D_pitch;
volatile int PID_value;
volatile int PID_value_pre;

volatile int dir;
volatile float err_pitch;
volatile float err_1_pitch;
volatile float err_2_pitch;

int PID_pitch(float setpoint, float input);

void motor_mixing(int PID_pitch,int PID_roll);
#endif
