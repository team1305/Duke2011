#include "MetalDrive.h"

/*----------------------------------------------------------------------------*/
/* Class initialization														  */
/*----------------------------------------------------------------------------*/
MetalDrive::MetalDrive(MetalSensors *sensors)
{
	io					= sensors;
	oldInput1			= 0;
	oldInput2			= 0;
	isDriving		 	= true;
	canShift			= true;
	shiftTime			= 0;
	outputX				= 0;
	outputY 			= 0;
	shiftMotorDirection	= 0;
	autoShifting		= false;
	shift				= 0;
}

/*----------------------------------------------------------------------------*/
/* Primary Drive / Automatic Shifting Function								  */
/*----------------------------------------------------------------------------*/
void MetalDrive::Drive(float joyX, float joyY, bool squareInputs)
{
	/* If the user is driving run the low pass filter on the joystick input */
	if(isDriving)
	{
		// Check if we want to square the inputs
		if(squareInputs)
		{
			// Get the x and y joystick values and square them
			joyY = signSquare(joyY);
			joyX = signSquare(joyX);
		}

		/* Run the low pass filter on the joystick input */
		outputX = lowpassFilter(joyX,outputX,TIME_CONSTANT);
		outputY = lowpassFilter(joyY,outputY,TIME_CONSTANT);
	}else{
		/* Check the current gear and set the virtual outputs accordingly */
		if(currentGear == LOW_GEAR)
			/* Run the low pass filter on the desired high gear transitionary speed */
			outputY = lowpassFilter(convertRPM(SHIFTING_SPEED)*shiftMotorDirection,outputY,TIME_CONSTANT);
		else if(currentGear == HIGH_GEAR)
			/* The motors are running slowly since we are down shifting anyways, so it is alright to set them to 0.0*/
			outputY = lowpassFilter(0.0,outputY,TIME_CONSTANT);

		/* Turning is probably not suitable for this situation, so set the turn amount to 0.0 */
		outputX = lowpassFilter(0.0,outputX,TIME_CONSTANT);
	}

	/* Calculate the left and right side outputs */
	leftOutput 	= (outputY-outputX)+(TURN_CONSTANT*fabs(outputX)*outputY);
	rightOutput = (outputY+outputX)+(TURN_CONSTANT*fabs(outputX)*outputY);


	/* Check if the shifting finished based on the magnetic sensors, and in case it doesn't shift it also cross checks against the timeout timer */
	if(currentGear == LOW_GEAR && io->leftLowMagnet->Get() == 0 && io->leftHighMagnet->Get() == 1){
		FinishShifting();
	}else if(currentGear == HIGH_GEAR && io->leftLowMagnet->Get() == 1 && io->leftHighMagnet->Get() == 0){
		FinishShifting();
	}else if(io->shiftingTimeout->Get() > SHIFT_DELAY){
		FinishShifting();
		io->shiftingTimeout->Stop();
		io->shiftingTimeout->Reset();
		isDriving		= true;
	}

	/* Check if automatic shifting is enabled and if we are not in a refactory period */
	if(autoShifting == true && canShift == true)
	{
		// Check which gear we need to shift and determine if we are the shift range
		// Check the signs on the left and right output to ensure they are the same before shifting

		if((shiftMotorDirection = checkSigns(leftOutput,rightOutput)) != 0)
		{
			float leftRPM	= ((io->leftFrontPositiveRPM->GetAverageVoltage()*RPM_CONSTANT+io->leftFrontNegativeRPM->GetAverageVoltage()*RPM_CONSTANT)+(io->leftRearPositiveRPM->GetAverageVoltage()*RPM_CONSTANT+io->leftRearNegativeRPM->GetAverageVoltage()*RPM_CONSTANT))/2;
			float rightRPM	= ((io->rightFrontPositiveRPM->GetAverageVoltage()*RPM_CONSTANT+io->rightFrontNegativeRPM->GetAverageVoltage()*RPM_CONSTANT)+(io->rightRearPositiveRPM->GetAverageVoltage()*RPM_CONSTANT+io->rightRearNegativeRPM->GetAverageVoltage()*RPM_CONSTANT))/2;

			if(currentGear == LOW_GEAR && leftRPM >= SHIFT_HIGHRPM && rightRPM >= SHIFT_HIGHRPM)
				Shift(HIGH_GEAR);
			else if(currentGear == HIGH_GEAR && leftRPM <= SHIFT_LOWRPM && rightRPM <= SHIFT_LOWRPM)
				Shift(LOW_GEAR);

		}
	}else if(io->shiftingWait->Get()>SHIFT_WAIT_TIME) {
			io->shiftingWait->Stop();
			io->shiftingWait->Reset();
			canShift	= true;
	}

	if(io->shiftingTimeout->Get() > SHIFT_DELAY/2)
		shift = desiredShift;

	// SHIFTING code
	if(shift == LOW_GEAR)
		io->shiftingSolenoid->Set(false);//originally true
	else
		io->shiftingSolenoid->Set(true);// originally false

	DriveMotors(leftOutput,rightOutput);
}

void MetalDrive::FinishShifting(void)
{
	/* Stop the shift timer and start the refractory period */
	io->shiftingTimeout->Stop();
	io->shiftingTimeout->Reset();
	io->shiftingWait->Start();

	if(currentGear == LOW_GEAR)
		currentGear = HIGH_GEAR;
	else if(currentGear == HIGH_GEAR)
		currentGear = LOW_GEAR;

	/* Since shifting has finished, the driver can continue driving */
	isDriving=true;
}

// Drives the motors
void MetalDrive::DriveMotors(float leftOutput, float rightOutput)
{
	/* Drive the motors according to the profile selected */
	io->leftFrontMotor->Set(leftOutput*LEFT_SIDE_INVERTED);
	io->leftRearMotor->Set(leftOutput*LEFT_SIDE_INVERTED);
	io->rightFrontMotor->Set(rightOutput*RIGHT_SIDE_INVERTED);
	io->rightRearMotor->Set(rightOutput*RIGHT_SIDE_INVERTED);
}

/*----------------------------------------------------------------------------*/
/*	Begins the shift and starts a delay before the user can drive		 	  */
/*----------------------------------------------------------------------------*/
void MetalDrive::Shift(int gear, int direction)
{
	/* Start the shift delay timer to get the motors into the recommended range for shifting */
	io->shiftingTimeout->Start();

	desiredShift = gear;
	/* Disable driving and shifting functionality during this period */
	isDriving	= false;
	canShift	= false;

}

/*----------------------------------------------------------------------------*/
/*	Sets the shifting mode to automatic or manual based on true/false	 	  */
/*----------------------------------------------------------------------------*/
void MetalDrive::SetShifting(bool mode)
{
	/* Set the shifting mode to either automatic or manual */
	if(mode)
	{
		autoShifting = true;
		//driverStation->SetDigitalOut(8,1);
	}else{
		autoShifting = false;
		//driverStation->SetDigitalOut(8,0);
	}
}
/*----------------------------------------------------------------------------*/
/*	Gets the shifting mode and returns a true/false value for it    	 	  */
/*----------------------------------------------------------------------------*/
bool MetalDrive::ShiftingMode(void)
{
	return autoShifting;
}

/*-------------------------------------------------------------------------------------*/
/* Returns the current gear the robot is in											   */
/*-------------------------------------------------------------------------------------*/
bool MetalDrive::GetGear()
{
	// returns true if high, returns false if low
	if(currentGear == HIGH_GEAR)
		return true;
	else
		return false;
}

/*-------------------------------------------------------------------------------------*/
/* Squares an input while keeping the sign the same									   */
/*-------------------------------------------------------------------------------------*/
float MetalDrive::signSquare(float input)
{
	//(input > 0) ? input=input*input : input=(-1.0)*input*input;
	if(input > 0)
		input=input*input;
	else if(input < 0)
		input=(-1.0)*input*input;
	return input;
}

/*-----------------------------------------------------------------------------------------*/
/* A simple low pass filter that uses a time interval, time constant and a previous input  */
/*-----------------------------------------------------------------------------------------*/
float MetalDrive::lowpassFilter(float input, float previousOutput, float timeConstant)
{
	float output = 0;
	float alpha = TIME_INTERVAL / (timeConstant + TIME_INTERVAL);
	output = alpha * input + (1-alpha) * previousOutput;
	return output;
}

/*----------------------------------------------------------------------------------------------------------*/
/* Converts an rpm value to an input between 0 and 1 based on the maximum voltage the rpm sensors can read  */
/*----------------------------------------------------------------------------------------------------------*/
float MetalDrive::convertRPM(float input)
{
	return (input/RPM_CONSTANT) * (1/MAX_VOLTAGE);
}

/*-----------------------------------------------------------------------------------------------------------------------*/
/* Checks if the signs between two values and the same, and returns whether they are positive, negative, or not the same */
/*-----------------------------------------------------------------------------------------------------------------------*/
int MetalDrive::checkSigns(float input1, float input2)
{
	/* Ensure the signs are both the same */
	if(input1 >= 0 && input2 >= 0)
		return 1;
	else if(input1 < 0 && input2 < 0)
		return -1;
	else
		return 0;
}
