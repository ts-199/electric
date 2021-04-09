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
	pid.max=4100;
	pid.min=10;
	pid.sum=0;
}

void pid_cal(float value)
{
  value=value*0.01194638672+0.0387;   //检测到测量数据转化
	U_out=value;
	pid.ek2=pid.set-value;
	pid.out=pid.kp*pid.ek2+pid.ki*pid.sum;
	pid.sum=+pid.ek2;

}

