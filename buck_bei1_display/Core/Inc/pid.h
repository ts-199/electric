#ifndef pid_h
#define pid_h
 
struct pid_var
{
	float kp;
	float ki;
	float kd;
	float ek2,ek1,ek0;
	float out;
	float set;
	float max;
	float min;
};


void pid_init(float kp,float ki,float kd,float set);
void pid_cal(float value);

#endif
