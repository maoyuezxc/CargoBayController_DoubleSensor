#ifndef _PID_H
#define _PID_H

#include <string.h>

typedef struct
{
	double P;
	double I;
	double D;
	double Object;		// 控制目标
	double SumError;
	double LastError;	// Error[-1]
	double PrevError;	// Error[-2]
} PID;

float PIDCalc(PID *p, double measurement);
void PIDInit(PID *pp, double p, double i, double d);
void SetPIDObject(PID *p, double object);

#endif /* _PID_H */
