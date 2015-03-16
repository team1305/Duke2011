/*----------------------------------------------------------------------------*/
/* Basic PD Class for PD Control                                                                                        */
/*----------------------------------------------------------------------------*/
#ifndef METALPD_H_
#define METALPD_H_
#include "WPILib.h"
#include <math.h>
#define PI	3.1415926538

class MetalPD
{
public:
   //MetalPID(double pGain, double iGain, double dGain, double iMax, double iMin);
	MetalPD(float pGainIn, float dGainIn);
    float Update(float reference, float position);
    void Set(float pGainIn, float dGainIn);
    void Reset();
    float		modifiedReference;
	float		previousReference;
	float		actualPreviousReference;
private:
	DriverStationLCD *ds;
	Timer		*pidTimer;
	float 		dState; // Last position input
	float 		pGain, // proportional gain
	           	dGain; // derivative gain
	float		constantReference;
	float		constantError;
	bool		refChanged;
};

#endif
