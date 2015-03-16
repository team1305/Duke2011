/*----------------------------------------------------------------------------*/
/* Class for the arm of the robot, to be worked on later					  */
/*----------------------------------------------------------------------------*/
#ifndef GEKKOARM_H_
#define GEKKOARM_H_

#include "WPILib.h"
#include "MetalSensors.h"
#include <math.h>

/* PID VALUES */
#define PARM	0.0001
#define IARM 	0.0
#define DARM 	0.0

#define MAXANGLE 		180
#define MINANGLE 		0.0
#define TOPVOLTAGE 		4.14
#define BOTTOMVOLTAGE 	2.40
#define ARMLENGTH		61.0			// real length 59.625, old 61
#define EXTENSIONLENGTH	12.0
#define TOWERHEIGHT		54.625			// 54.875, old 54.625
#define PI				3.1415926538
#define HEIGHTOFFSET	2.5

#define TIME_INTERVAL_2 	10			// Increase in time interval increases period of acceleration
#define TIME_CONSTANT_CLAW 	2500		// Increase in time constant decreases rate of change of acceleration
#define TIME_CONSTANT_ARM 	10000		// Increase in time constant decreases rate of change of acceleration

#define COUNTEROVERFLOW		25

#define TOPLIMIT			115.0
#define BOTTOMLIMIT			0.25

class GekkoArm
{
public:
	// This is purely based on speculation of the final configuration of the arm
	/* TODO : Wait until final arm design and rework some of these methods */
	GekkoArm(MetalSensors *sensors);
	void Extend();
	void SetHeight(float height);
	float GetHeight();
	void Rotate(float output);
	float signSquare(float input);
	void Claw(float output);
	bool Drop(void);
	float lowpassFilter(float input, float previousOutput, float timeConstant);
	MetalSensors *io;
	Timer *pidUpdateTimer;
	Timer *dropTimer;
	// Motors
	//Victor 			*primaryMotor;
	//Victor 				*clawMotor;
	// Solenoids
	//Solenoid 		*extension;
	//Relay			*extension;
	// Sensors (Probably potentiometers)
	//AnalogChannel	*armPot;
	// Driver Station interfaces
	//DriverStation 		*dsInterface;
	//DriverStationLCD	*dsLCD;

	/* TODO : Might use a custom PID class, but will need to evaluate this one. */
	/* TODO : On a side note, also going to look into making it a single PID Controller for both arm/wrist */

	/* Variables that are used and may be useful elsewhere */
	float currentAngle;
	int counter;
	float reference;
	float armOutput;
	bool autoEnabled;
	float previousArmOut;
	bool dropping;
	bool extended;
private:
	/* TODO : Tune these values at a further date */
	double pArmValue, iArmValue, dArmValue;
	double pWristValue, iWristValue, dWristValue;
	float maxAngle;
	float clawOutput;

	int counterMax;
};

#endif
