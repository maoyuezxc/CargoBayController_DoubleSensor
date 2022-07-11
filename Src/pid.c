#include "pid.h"

float dError, error;

float PIDCalc(PID *p, double measurement)
{
	error = p->Object - measurement; // 偏差
	p->SumError += error;			 // 积分
	dError = error - p->LastError;
	p->PrevError = p->LastError;
	p->LastError = error;
	return (p->P * error		 // 比例项
			+ p->I * p->SumError // 积分项
			+ p->D * dError);	 // 微分项
}

void PIDInit(PID *pp, double p, double i, double d)
{
	memset(pp, 0, sizeof(PID));
	pp->P = p;
	pp->D = d;
	pp->I = i;
}

void SetPIDObject(PID *p, double object)
{
	p->Object = object;
	p->PrevError = 0;
	p->LastError = 0;
	p->SumError = 0;
}
