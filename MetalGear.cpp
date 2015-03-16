#include "WPILib.h"
#include "MetalSensors.h"
#include "MetalDrive.h"
#include "GekkoArm.h"
//#include "ArcadePad.h"
#include "SolidEye.h"

// Misc. Variables for joystick button states
bool button1Down = false;
bool button2Down = false;
bool button3Down = false;
bool button4Down = false;
bool button5Down = false;
bool button6Down = false;
bool button7Down = false;
bool button8Down = false;
bool button9Down = false;
bool button10Down = false;
bool pad1Down	 	= false;
bool pad2Down		= false;
bool pad3Down		= false;
bool pad4Down		= false;
bool pad5Down		= false;
bool pad6Down		= false;
bool pad7Down		= false;
bool pad8Down		= false;
bool pad9Down		= false;
bool pad10Down		= false;
bool pad11Down		= false;

bool dropDone	= false;
bool deployed = false;
bool guideDown = false;
int dropState	= 0;
int autoState 	= 0;

/* The main robot class, codenamed MetalGear  */
class MetalGear : public SimpleRobot
{
	// 	Interface Constructors
	Joystick 			*stick;
	Joystick			*pad;
	MetalSensors		*io;
	MetalDrive			*drive;
	GekkoArm			*arm;
	Timer				*autoTimer;
	Timer				*autoTimer2;
	//Timer				*matchTimer;

public:
	/* Metal Gear constructor */
	MetalGear(void)
	{
		/* Interface Constructors */
		stick 				= new Joystick(1);
		pad					= new Joystick(2);
		io					= new MetalSensors();

		/* Initialize all inputs and outputs on the robot */
		io->InitializeSensors();

		/* Primary Component Construtors */
		drive				= new MetalDrive(io);
		arm					= new GekkoArm(io);
		autoTimer			= new Timer();
		autoTimer2			= new Timer();
		//matchTimer			= new Timer();
	}

	void Disabled(void)
	{
		/* Initialize the Connection to the Driver Station */
		/* NOTE : This is done during disabled mode to prevent
		 * The cRIO from hanging while trying to connect during immediate initialization */
		io->InitializeDriverStation();
		io->reference = arm->GetHeight();
		//drive->Drive(0.0,0.0,false);
		//arm->Rotate(0.0);
		//io->gyro->Reset();
	}

	void RobotInit(void)
	{
		/* 	Set the Watchdog expiration */
		GetWatchdog().SetExpiration(3.0);
	}

	/* The Autonomous function is only called once during an actual match */
	void Autonomous(void)
	{
		/* Reset Encoders, Timers, and States for Autonomous, and start them */
		io->leftEncoder->Reset();
		//io->gyro->Reset();
		autoState = 0;
		autoTimer2->Reset();
		autoTimer2->Start();
		autoTimer->Reset();
		autoTimer->Start();

		/* Continuously run autonomous until the 15 second period has expired */
		while(autoTimer2->Get() < 15.0)
		{
			/* Select which state of the autonomous to do */
			switch(autoState)
			{
			case 0:
				/* Reset the extension variable and make sure the arm is not extended */
				arm->extended = false;
				//io->extensionSolenoid->Set(false);

				/* Turn on the red light */
				//io->blueLight->SetDirection(Relay::kForwardOnly);
				//io->blueLight->Set(Relay::kForward);
				//io->redLight->SetDirection(Relay::kForwardOnly);
				//io->redLight->Set(Relay::kForward);

				/* Clamp the claw down on the tube for 0.75s, then go to the next state */
				if(autoTimer->Get() < 0.75)
				{
					/* Close the claw at full speed */
					arm->Claw(1.0);
				}else{
					/* Reset the timer and advance to the next state of the program */
					autoTimer->Reset();
					autoState++;
				}
				break;
			case 1:
				/* Begin driving forward at full speed */
				/* NOTE : Robot currently has a tendency to drift slightly left, so this needs to be fixed */
				if(autoTimer->Get() > 0.6)
					drive->Drive(0,-1.0,false);
				else
					drive->Drive(0.1,-1.0,false);
				/*if(autoTimer->Get() > 0.25)
				{
					/* NEW CODE YO
					if(io->leftEncoder->GetDistance()-io->rightEncoder->GetDistance() > 0.5)
					{
						drive->Drive(0.1,-1.0,false);
						//drive->DriveMotors(-0.875,-0.95);
					}else if(io->leftEncoder->GetDistance()-io->rightEncoder->GetDistance() < -0.5){
						//drive->DriveMotors(-0.95,-0.875);
						drive->Drive(-0.1,-1.0,false);
					}else{
						drive->Drive(0,-1.0,false);
					}
				}else{
					drive->Drive(0,-1.0,false);
				}*/

				/* Extend the arm */
				io->extensionSolenoid->Set(true);
				arm->extended = true;

				/* Set the arm to go to 106'' off the ground */
				io->reference = 104.0;

				/* Begin driving forward until the robot has passed 165'' (currently the encoder distance per pulse is slightly off, so this remains an estimate */
				if(io->leftEncoder->GetDistance() > 80.0)// 130
				{
					/* Reset the timer and advance to the next state of the program */
					autoTimer->Reset();
					autoState++;
				}
				break;
				case 2:
					/* Start slowing down as we approach the goal */
					drive->Drive(0.0,0.0,false);
					io->extensionSolenoid->Set(true);
					arm->extended = true;
					if(autoTimer->Get() > 0.75)
					{
						/* Reset the timer and advance to the next state of the program */
						autoTimer->Reset();
						autoState++;

						/* Reset the arm dropping state */
						arm->dropping = false;
					}
					break;
				case 3:
					/* Pull the arm down, and drop the tube */
					arm->Drop();

					/* Go to the next state after the drop function completes at about 2.5s */
					if(autoTimer->Get() > 1.00)
					{
						autoTimer->Reset();
						autoState++;
					}
					break;
				case 4:
					/* Continue driving backwards at full speed */
					drive->Drive(0,0.5,false);

					/* SHIFT TO HIGH */
					//drive->Shift(HIGH_GEAR,drive->shiftMotorDirection);

					/* Go backwards for about 2.0s before going to the next state */
					if(autoTimer->Get() > 2.5)
					{
						autoTimer->Reset();
						autoState++;
					}
					break;
				case 5:
					/* Do a 360 spin */
					drive->Drive(-1.0,0.0,false);

					if(autoTimer->Get() > 0.5)
					{
						autoTimer->Reset();
						autoState++;
					}
				case 6:
					/* Stop driving in order to avoid crossing center line in autonomous */
					drive->Drive(0,0.0,false);
					/* Set the arm to floor height */
					io->reference = 0.75;
					break;
				}

			/* This constantly sets the height of the arm to the reference points set during the states */
			arm->SetHeight(io->reference);
		}

		/* Cleanup and reset autonomous */
		autoState = 0;
		autoTimer->Stop();
		autoTimer2->Stop();
		autoTimer->Reset();
		autoTimer2->Reset();
	}

	/* Teleoperated Control  */
	void OperatorControl(void)
	{
		io->reference = arm->GetHeight();
		arm->armOutput = 0.0;
		io->armPDController->previousReference = arm->GetHeight();
		io->armPDController->actualPreviousReference = arm->GetHeight();
		io->armPDController->modifiedReference = arm->GetHeight();
		arm->previousArmOut = 0.0;
		io->armPDController->Reset();
		while (IsOperatorControl())
		{
			/* Check if the two primary shift buttons have been pressed and shift */
			if(stick->GetRawButton(5))
				drive->Shift(LOW_GEAR,drive->shiftMotorDirection);
			else if(stick->GetRawButton(7) == true || stick->GetRawButton(8) == true)
				drive->Shift(HIGH_GEAR,drive->shiftMotorDirection);

			/* Check if button 6 has been pressed */
			if(stick->GetRawButton(6) == true && button6Down == false)
			{
				/* Switch to low gear and enable slow driving mode */
				drive->Shift(LOW_GEAR,drive->shiftMotorDirection);
				drive->Drive(stick->GetZ()*0.45,stick->GetY()*0.45,true);
				button6Down = true;
			}else if(stick->GetRawButton(6) == true){
				/* Drive in slow mode */
				drive->Drive(stick->GetZ()*0.45,stick->GetY()*0.45,true);
				button6Down = true;
			}else{
				/* Drive normally */
				drive->Drive(stick->GetZ(),stick->GetY(),true);
				button6Down = false;
			}

			/* Check if button 9 is pressed, and deploy minibot if it is */
			if(stick->GetRawButton(9) == true && button9Down == false)
			{
				if(deployed == false)
				{
					io->minibotSolenoid->Set(true);
					deployed = true;
				}else{
					io->minibotSolenoid->Set(false);
					deployed = false;
				}

				button9Down = true;
			}else if(stick->GetRawButton(9) == false){
				button9Down = false;
			}

			/* Check if button 10 is pressed, and deploy the minibot guide if it is */
			if(stick->GetRawButton(10) == true && button10Down == false)
			{
				if(guideDown == false)
				{
					io->guideSolenoid->Set(true);
					guideDown = true;
				}else{
					io->guideSolenoid->Set(false);
					guideDown = false;
				}

				button10Down = true;
			}else if(stick->GetRawButton(10) == false){
				button10Down = false;
			}

			// Controls arm and claw automatically using PID and the rcade pad
			if(pad->GetRawButton(8) == true && pad8Down == false)
			{
				arm->dropping = false;
				arm->Drop();

				pad8Down = true;
			}else if(pad8Down == true){
				dropDone = arm->Drop();
				pad8Down = dropDone;
			}else{
				arm->Claw(pad->GetY()*0.9);
				/* Check if any of the presets were pressed */
				if(pad->GetRawButton(4))
				{
					io->reference = 102.0;
					pad1Down	= false;
					io->extensionSolenoid->Set(true);
					arm->extended = true;
				}else if(pad->GetRawButton(3)){
					io->reference = 64.5;
					pad1Down = false;
				}else if(pad->GetRawButton(2)){
					io->reference = 32.0;
					pad1Down = false;
				}else if(pad->GetRawButton(6)){
					io->reference = 0.3;
					io->extensionSolenoid->Set(true);
					arm->extended = true;
					pad1Down = false;
				}

				/* Controls arm extension */
				if(pad->GetRawButton(5) && pad5Down == false)
				{
					arm->Extend();
					pad5Down = true;
				}else if(pad->GetRawButton(5) == false){
					pad5Down = false;
				}

				// Check if we need to bump the reference point
				if(pad->GetRawButton(1) == true && pad1Down == false)
				{
					io->previousReference = io->reference;
					io->reference = io->reference + 8.5;
					pad1Down = true;
				}

				/* Check whether the secondary driver's controller is outside deadzone, and enable manual control if it is */
				if(pad->GetRawAxis(4) > 0.15 || pad->GetRawAxis(4) < -0.15)
				{
					//arm->Rotate(arm->lowpassFilter(pad->GetRawAxis(4)*0.5,armOut,10000));

					/* Control the arm manually, and take into account the deadzone */
					if(pad->GetRawAxis(4) > 0.0)
					{
						if(pad->GetRawButton(7))
							arm->Rotate(arm->signSquare(pad->GetRawAxis(4)*0.55));
						else
							arm->Rotate(arm->signSquare(pad->GetRawAxis(4)*0.9-0.100));
						io->reference = arm->GetHeight()+0.5;
					}else{
						if(pad->GetRawButton(7))
							arm->Rotate(arm->signSquare(pad->GetRawAxis(4)*0.1));
						else
							arm->Rotate(arm->signSquare(pad->GetRawAxis(4)*0.9+0.100));

						io->reference = arm->GetHeight()-1.0;
					}

				}else{
					/* Otherwise set arm using the PD controller */
					/*if(arm->GetHeight()-io->reference < 0.5 || arm->GetHeight()-io->reference > -0.5) */
					arm->SetHeight(io->reference);
					/*else
						arm->Rotate(0);*/
				}
			}

			/* Display current information about arm height and the desired reference point */
			arm->GetHeight();
			io->driverStationDisplay->PrintfLine(DriverStationLCD::kUser_Line4,"Reference : %f",io->reference);

			/* Control the tube request lights */
			if(stick->GetRawButton(3))
			{
				io->redLight->SetDirection(Relay::kForwardOnly);
				io->redLight->Set(Relay::kForward);
				io->whiteLight->SetDirection(Relay::kForwardOnly);
				io->whiteLight->Set(Relay::kOff);
				io->blueLight->SetDirection(Relay::kForwardOnly);
				io->blueLight->Set(Relay::kOff);
			}else if(stick->GetRawButton(4)){
				io->redLight->SetDirection(Relay::kForwardOnly);
				io->redLight->Set(Relay::kOff);
				io->whiteLight->SetDirection(Relay::kForwardOnly);
				io->whiteLight->Set(Relay::kForward);
				io->blueLight->SetDirection(Relay::kForwardOnly);
				io->blueLight->Set (Relay::kOff);
			}else if(stick->GetRawButton(1)){
				io->redLight->SetDirection(Relay::kForwardOnly);
				io->redLight->Set(Relay::kOff);
				io->whiteLight->SetDirection(Relay::kForwardOnly);
				io->whiteLight->Set(Relay::kOff);
				io->blueLight->SetDirection(Relay::kForwardOnly);
				io->blueLight->Set(Relay::kForward);
			}else if(stick->GetRawButton(2)){
				io->redLight->SetDirection(Relay::kForwardOnly);
				io->redLight->Set(Relay::kOff);
				io->whiteLight->SetDirection(Relay::kForwardOnly);
				io->whiteLight->Set(Relay::kOff);
				io->blueLight->SetDirection(Relay::kForwardOnly);
				io->blueLight->Set(Relay::kOff);
			}

			//io->driverStationDisplay->PrintfLine(DriverStationLCD::kUser_Line1,"LE : %f",stick->GetZ());
			//io->driverStationDisplay->UpdateLCD();
			/*if(io->tripSensor->Get() == 1)
			{
				io->redLight->SetDirection(Relay::kForwardOnly);
				io->redLight->Set(Relay::kForward);
				io->whiteLight->SetDirection(Relay::kForwardOnly);
				io->whiteLight->Set(Relay::kForward);
				io->blueLight->SetDirection(Relay::kForwardOnly);
				io->blueLight->Set(Relay::kForward);
			}*/
		}
	}
};

START_ROBOT_CLASS(MetalGear);
