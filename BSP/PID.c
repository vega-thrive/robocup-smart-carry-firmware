#include "PID.h"

volatile PID pid_m1;
volatile PID pid_m2;
volatile PID pid_m3;
volatile PID pid_m4;

#define PID_OUTPUT_LIMIT 950

static float mabs(float a)
{
	return (a <= 0.0f) ? -a : a;
}

static void limit(float *value, float limitation)
{
	float lim = mabs(limitation);
	if (*value > lim)
		*value = lim;
	else if (*value < -lim)
		*value = -lim;
}

void PID_Init(volatile PID *pid, float kp, float ki, float kd)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->error = 0.0f;
	pid->lasterror = 0.0f;
	pid->preerror = 0.0f;
	pid->output = 0.0f;
}

void PID_Calc(volatile PID *pid, float target, float actual)
{
	if (target >= 600.0f) target = 600.0f;
	pid->error = target - actual;
	if (pid->error > -2.0f && pid->error < 2.0f) pid->error = 0.0f;

	pid->output += pid->kp * (pid->error - pid->lasterror)
	             + pid->ki * (pid->error)
	             + pid->kd * (pid->error - 2.0f * pid->lasterror + pid->preerror);

	pid->preerror = pid->lasterror;
	pid->lasterror = pid->error;
	limit((float *)&pid->output, (float)PID_OUTPUT_LIMIT);
}
