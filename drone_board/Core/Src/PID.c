#include "PID.h"
#include "sbus.h"
int PID_pitch(float setpoint, float input){
	Kp_pitch = 20;
	Ki_pitch = 0;
	Kd_pitch = 1;

	dt = 0.002472;
	err_pitch = -input;

	P_pitch = Kp_pitch*(err_pitch);
	I_pitch = 0.5*Ki_pitch*dt*(err_pitch + err_1_pitch);
	D_pitch = Kd_pitch/dt*(err_pitch - 2*err_1_pitch + err_2_pitch);

	PID_value = ( (int) P_pitch +(int) I_pitch +(int)D_pitch);
    err_2_pitch = err_1_pitch;
    err_1_pitch = err_pitch;
    PID_value_pre = PID_value;
    /*BO for PID control*/
    if(PID_value > 500){
    	PID_value = 500;
    }
    if(PID_value < -500){
      	PID_value = -500;
      }
    return PID_value;
}

void motor_mixing(int PID_pitch,int PID_roll){
	/*X-quad config*/
    esc_value1 = 1500 + PID_pitch;
    esc_value2 = 1500 - PID_roll;
    esc_value3 = 1500 - PID_pitch;
    esc_value4 = 1500 + PID_roll;
}
