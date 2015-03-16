/*----------------------------------------------------------------------------*/
/* Basic drive class featuring joystick smoothing							  */
/*----------------------------------------------------------------------------*/
#ifndef METALDRIVE_H_
#define METALDRIVE_H_
/* Include the required header file for basic library functions */
#include "WPILib.h"
#include "MetalSensors.h"
#include <math.h>

/*----------------------------------------------------------------------------*/
/* Configuration settings for shifting, drive, and PID control of the drive	  */
/*----------------------------------------------------------------------------*/

/*
 * The time interval and time constants for the low pass filter
 * used for soft-start/joystick smoothing
 */
#define TIME_INTERVAL 		8			// Increase in time interval increases period of acceleration
#define TIME_CONSTANT 		8000		// Increase in time constant decreases rate of change of acceleration

/* Various drive base configuration / customization */
#define TURN_CONSTANT		0.25		// A value from ideally 0.0 - 0.5 to scale the effect of the software differential
#define WHEEL_DIAMETER 		8.0			// Wheel diameter in inches

/* Encoder Configuration */
#define ENCODER_PULSES 		250			// Number of pulses the encoder records per revolution of the wheel

/* Line Tracker Configuration */
#define LINE_SPEED			0.25		// line speed
#define LINE_LOW_1			0.15		// Other line speed
#define LINE_LOW_2			0.25		// Faster line speed
#define LINE_HIGH_3			0.25		// Other line speed
#define LINE_HIGH_4			0.35		// Faster line speed
#define SCALER_VALUE		0.025		// The scaling value for adjusting the motor turn output based on the gyro angle
#define LEFTSIDE			true
#define RIGHTSIDE			false
#define TOHIGH				-1
#define TOLOW				1

/* Gyro Configuration */
#define GYRO_INTERVAL 		100
#define GYRO_TIME_CONSTANT	1000
#define GYRO_SCALE			1.0			// The scaling value to scale the value of the gyro to 360 degrees (default 9.0)

/* Settings used for shift timing, and the "automatic" transmission */
#define RPM_CONSTANT		1909.482758	// The number of RPM per volt
#define SHIFT_HIGHRPM		1000		// The target RPM range for shifting to high gear - 4000
#define	SHIFT_LOWRPM		500			// The target RPM range for shifting to low gear - 1100
#define SHIFTING_SPEED		1500		// The intermediate shifting speed for high gear
#define SHIFT_DELAY			0.1			// The delay before the driver can begin driving after a shift
#define SHIFT_WAIT_TIME		1			// The wait time before another automatic shift will occur
#define MAX_VOLTAGE			2.784		// The maximum voltage that can be read from the rpm sensors
#define HIGH_GEAR			1			// The numeral defintion of high gear
#define LOW_GEAR			0			// The numeral definition of low gear
#define AUTOMATIC			true		// The boolean defintion of automatic shifting
#define MANUAL				false		// the boolean definition of manual shifting

class MetalDrive
{
public:
	MetalDrive(MetalSensors *sensors);
	MetalSensors			*io;

	// Function Constructs
	void Drive(float joyX, float joyY, bool squareInputs);
	void DriveMotors(float leftOutput, float rightOutput);
	void Shift(int gear, int direction=0);
	//void TrackLine(void);
	void SetShifting(bool mode);
	void FinishShifting();
	bool ShiftingMode(void);
	bool GetGear(void);
	float signSquare(float input);
	float convertRPM(float input);
	float lowpassFilter(float input, float previousOutput, float timeConstant);
	int checkSigns(float input1, float input2);
	float 	leftOutput,rightOutput;
	float 	shiftMotorDirection;
	//float	lineLeft, lineRight;
	//float	lineLeftOutput, lineRightOutput;
	//float	fowardSpeed;
private:
	bool 	isDriving, canShift;
	bool 	autoShifting;
	float	shiftTime;
	float 	outputX,outputY;
	int 	currentGear;
	int		oldInput1, oldInput2;
	int		shift;
	int 	desiredShift;
};

#endif
