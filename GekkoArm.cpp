#include "GekkoArm.h"

/* TODO : Initialization for the motors and PID Controllers here */
GekkoArm::GekkoArm(MetalSensors *sensors)
{
	io = sensors;
	/* Initialize all our variables */ //the game
	pArmValue	 = PARM;
	iArmValue	 = IARM;
	dArmValue	 = DARM;
	maxAngle	 = MAXANGLE;

	/* Initialize Motrs and Actuators */
	//primaryMotor = new Victor(primaryMotorPort);
	//clawMotor 	 = new Victor(CLAW_PORT);
	//extension	 = new Solenoid(2);
	//armPot		 	= new AnalogChannel(ARM_POT_PORT);
	//dsInterface 	= DriverStation::GetInstance();
	//dsLCD			= DriverStationLCD::GetInstance();
	//extension		= new Relay(2);
	extended		= false;
	clawOutput		= 0.0;
	armOutput		= 0.0;
	counter			= 0.0;
	previousArmOut	= 0.0;
	counterMax		= 25;
	reference		= 0.0;
	autoEnabled		= false;
	dropping		= false;
	pidUpdateTimer	= new Timer();
	dropTimer		= new Timer();
}

void GekkoArm::SetHeight(float height)
{
	if(autoEnabled == false)
	{
		io->armPDController->modifiedReference = GetHeight();
		autoEnabled = true;
	}
	float output = 0;

	io->armPDController->Set(0.145,0.145);
	if(pidUpdateTimer->Get() > 0.005 && height < TOPLIMIT && height > BOTTOMLIMIT)
	{
		output	= io->armPDController->Update(height, GetHeight());

		pidUpdateTimer->Reset();
	}else{
		pidUpdateTimer->Start();
		output = previousArmOut;
	}

	if(height > TOPLIMIT || height < BOTTOMLIMIT)
	{
		output = 0.0;
	}

	previousArmOut	= output;
	Rotate(output*-1.0);
}

float GekkoArm::GetHeight(void)
{
	float height, length;

	if(extended)
		length = ARMLENGTH;
	else
		length = ARMLENGTH-EXTENSIONLENGTH;

	//height	=	length*sin(((io->armPotentiometer->GetVoltage()*((MAXANGLE-MINANGLE)/MAXVOLTAGE))*(PI/180))-(PI/2));
	//height	=	height+TOWERHEIGHT;
	height		=	length*cos(-1.5*(io->armPotentiometer->GetAverageVoltage() - 2.4)) + TOWERHEIGHT - HEIGHTOFFSET;
	//dsLCD->PrintfLine(DriverStationLCD::kUser_Line6,"POT Voltage : %0.1f",height);
	//dsLCD->UpdateLCD();.
	io->driverStationDisplay->PrintfLine(DriverStationLCD::kUser_Line6,"Height : %f",height);
	io->driverStationDisplay->UpdateLCD();
	return height;
}

/* TODO : Arm rotation code (probably PID) */
void GekkoArm::Rotate(float output)
{
	if(armOutput > 0.05 && io->armPotentiometer->GetVoltage() > 4.05)
		output = 0.0;

	armOutput	= output;
	//if(io->armPotentiometer->GetVoltage() > 3.7 && (armOutput > -0.05 || armOutput < 0.05))
		//output = 0.1;

	if(armOutput > 0.15)
		armOutput = 0.15;

	if(armOutput < -0.75)
		armOutput = -0.75;

	if(armOutput > 0 && GetHeight() < BOTTOMLIMIT)
		armOutput = 0;

	if(armOutput < 0 && GetHeight() > TOPLIMIT)
			armOutput = 0;

	if(armOutput < 0 && GetHeight() > TOPLIMIT-15.0 && extended == false)
		armOutput = 0;

	//if(armOutput < 0.0 && io->armPotentiometer->GetVoltage() < 2.4)
		//armOutput = 0.05;

	io->driverStationDisplay->PrintfLine(DriverStationLCD::kUser_Line2,"Output : %f",armOutput);
	io->driverStationDisplay->UpdateLCD();

	/*if(armOutput > 0.05 && armOutput < 0.09)
	{
		io->armMotor->Disable();
	}else{
	*/io->armMotor->Set(armOutput);
	//}
}

void GekkoArm::Extend()
{
	if(extended == false)
	{
		io->extensionSolenoid->Set(true);
		extended = true;
	}else{
		io->extensionSolenoid->Set(false);
		extended = false;
	}
}

/*-----------------------------------------------------------------------------------------*/
/* A simple low pass filter that uses a time interval, time constant and a previous input  */
/*-----------------------------------------------------------------------------------------*/
float GekkoArm::lowpassFilter(float input, float previousOutput, float timeConstant)
{
	float output = 0;
	float alpha = TIME_INTERVAL_2 / (timeConstant + TIME_INTERVAL_2);
	output = alpha * input + (1-alpha) * previousOutput;
	return output;
}

/*-------------------------------------------------------------------------------------*/
/* Squares an input while keeping the sign the same									   */
/*-------------------------------------------------------------------------------------*/
float GekkoArm::signSquare(float input)
{
	//(input > 0) ? input=input*input : input=(-1.0)*input*input;
	if(input > 0)
		input=input*input;
	else if(input < 0)
		input=(-1.0)*input*input;
	return input;
}

void GekkoArm::Claw(float output)
{
//	if(io->upperClawLimit->Get() == 0 && output < 0.0)
	//{
	//	io->clawMotor->Set(0.0);
	//}else{
		clawOutput	= lowpassFilter(signSquare(output),clawOutput,TIME_CONSTANT_CLAW);
		io->clawMotor->Set(clawOutput);
	//}
}

bool GekkoArm::Drop(void)
{
	if(dropping == false)
	{
		dropTimer->Reset();
		dropTimer->Start();
		dropping = true;
	}

	if(dropTimer->Get() < 0.5)
	{
		if(dropTimer->Get() > 0.25 && dropTimer->Get() < 0.5)
			Claw(-0.9);
		else
			Claw(0.0);

		/*if(dropTimer->Get() < 1.5)
			SetHeight(io->reference-14.0);
		else
			SetHeight(io->reference)*;*/
		if(dropTimer->Get() < 0.4){
			Rotate(0.75*0.75);
			io->reference = GetHeight();
		}else if(dropTimer->Get() > 0.4 && dropTimer->Get() < 0.6){
			io->extensionSolenoid->Set(false);
			extended = false;
			io->reference = GetHeight();
		}else{
			SetHeight(io->reference);
		}
	}else{
		return false;
	}

	return true;
}


