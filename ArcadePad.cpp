#include "ArcadePad.h"

ArcadePad::ArcadePad(DriverStation *sensors)
{
	io = sensors;
	kFloor		= 1;	// Digital
	kDump		= 2;	// Digital
	kCloseClaw	= 3;	// Digital
	kOpenClaw	= 4;	// Digital
	kTop		= 5;	// Digital
	kMiddle		= 6;	// Digital
	kBottom 	= 7;	// Digital
	kBump		= 8;	// Digital

	kClaw		= 1;	// Analog
	kTrim		= 2;	// Analog
	kHuman		= 3;	// Analog
	kExtend		= 4;	// Analog
	kStow		= 5;	// Analog
	kSquare		= 6;	// Analog
	kCircle 	= 7;	// Analog
	kTriange	= 8;	// Analog

	kHigh		= 1;	// LED out
	kLow		= 2;	// LED out
	kManual		= 3;	// LED out
	kAuto		= 4;	// LED out

	// Initialize all button states
	kFloorDown 		= false;// Digital
	kDumpDown 		= false;// Digital
	kCloseClawDown 	= false;// Digital
	kOpenClawDown 	= false;// Digital
	kTopDown 		= false;// Digital
	kMiddleDown 	= false;// Digital
	kBottomDown 	= false;// Digital
	kBumpDown 		= false;// Digital

	kClawDown 		= false;// Analog
	kTrimDown 		= false;// Analog
	kHumanDown 		= false;// Analog
	kExtendDown 	= false;// Analog
	kStowDown 		= false;// Analog
	kSquareDown 	= false;// Analog
	kCircleDown 	= false;// Analog
	kTriangeDown 	= false;// Analog
}

bool ArcadePad::GetRawButton(UINT32 button)
{
	// Assumes button is digital
	bool state = io->GetDigitalIn(button);
	if(state == false)
		state = true;
	else
		state = false;
	return state;
}

bool  ArcadePad::GetRawAnalogButton(UINT32 button)
{
	bool state;

	float value = io->GetAnalogIn(button);
	if(value > 1.5)
		state = false;
	else
		state = true;

	return state;
}

float  ArcadePad::GetAnalog(UINT32 button)
{
	return io->GetAnalogIn(button);
}

void ArcadePad::SetLED(UINT32 led, bool state)
{
	io->SetDigitalOut(led, state);
}
