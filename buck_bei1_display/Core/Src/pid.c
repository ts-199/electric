#include "pid.h"
#include "main.h"
extern ADC_HandleTypeDef hadc1;
extern float U_out;
struct pid_var pid;
void pid_init(float kp,float ki,float kd,float set)
{
	pid.kp=kp;
	pid.ki=ki;
	pid.kd=kd;
	pid.ek2=0;
	pid.ek1=0;
	pid.ek0=0;
	pid.out=0;
	pid.set=set;
	pid.max=2570;
	pid.min=10;
}

void pid_cal(float value)
{
  value=((float)value/(float)4096*(float)3.3+0.0005)/0.0206;
	U_out=value;
	pid.ek2=pid.set-value;
	pid.out=(pid.kp+pid.ki+pid.kd)*pid.ek2-(2*pid.kd+pid.kp)*pid.ek1+pid.kd*pid.ek0;
	pid.ek1=pid.ek2;
	pid.ek0=pid.ek1;
}

