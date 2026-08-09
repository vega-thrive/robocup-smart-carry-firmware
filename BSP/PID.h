#ifndef __PID_H__
#define __PID_H__

typedef struct
{
	float kp;
	float ki;
	float kd;
	float error;
	float lasterror;
	float preerror;
	float output;
} PID;

void PID_Init(volatile PID *pid, float kp, float ki, float kd);
void PID_Calc(volatile PID *pid, float target, float actual);

extern volatile PID pid_m1;
extern volatile PID pid_m2;
extern volatile PID pid_m3;
extern volatile PID pid_m4;

#endif
