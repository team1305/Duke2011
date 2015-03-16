/*----------------------------------------------------------------------------*/
/* Basic PD Class for PD Control                                                                                        */
/*----------------------------------------------------------------------------*/
#ifndef ARCADEPAD_H_
#define ARCADEPAD_H_
#include "WPILib.h"
//using namespace std;

#define ANALOG_SWITCH_POINT 1.0
/*
#define kFloor		= 1;	// Digital
#define kDump		= 2;	// Digital
#define kCloseClaw	= 3;	// Digital
#define kOpenClaw	= 4;	// Digital
#define kTop		= 5;	// Digital
#define kMiddle		= 6;	// Digital
#define kBottom 	= 7;	// Digital
#define kBump		= 8;	// Digital

#define kClaw		= 1;	// Analog
#define kTrim		= 2;	// Analog
#define kHuman		= 3;	// Analog
#define kExtend		= 4;	// Analog
#define kStow		= 5;	// Analog
#define kSquare		= 6;	// Analog
#define kCircle 	= 7;	// Analog
#define kTriange	= 8;	// Analog

#define kHigh		= 1;	// LED out
#define kLow		= 2;	// LED out
#define kManual		= 3;	// LED out
#define kAuto		= 4;	// LED out
*/
class ArcadePad
{
public:
	ArcadePad(DriverStation *sensors);
	bool GetRawButton(UINT32 button);
	bool GetRawAnalogButton(UINT32 button);
	float GetAnalog(UINT32 button);
	void SetLED(UINT32 led, bool state);

	UINT32 kFloor;// Digital
	UINT32 kDump;// Digital
	UINT32 kCloseClaw;// Digital
	UINT32 kOpenClaw;// Digital
	UINT32 kTop;// Digital
	UINT32 kMiddle;// Digital
	UINT32 kBottom ;// Digital
	UINT32 kBump;// Digital

	UINT32 kClaw;// Analog
	UINT32 kTrim;// Analog
	UINT32 kHuman;// Analog
	UINT32 kExtend;// Analog
	UINT32 kStow;// Analog
	UINT32 kSquare;// Analog
	UINT32 kCircle ;// Analog
	UINT32 kTriange;// Analog

	UINT32 kHigh;// LED out
	UINT32 kLow;// LED out
	UINT32 kManual;// LED out
	UINT32 kAuto;// LED out

	bool kFloorDown;// Digital
	bool kDumpDown;// Digital
	bool kCloseClawDown;// Digital
	bool kOpenClawDown;// Digital
	bool kTopDown;// Digital
	bool kMiddleDown;// Digital
	bool kBottomDown;// Digital
	bool kBumpDown;// Digital

	bool kClawDown;// Analog
	bool kTrimDown;// Analog
	bool kHumanDown;// Analog
	bool kExtendDown;// Analog
	bool kStowDown;// Analog
	bool kSquareDown;// Analog
	bool kCircleDown;// Analog
	bool kTriangeDown;// Analog
private:
	DriverStation *io;
};

#endif
