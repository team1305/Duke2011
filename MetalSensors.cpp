#include "MetalSensors.h"

/*----------------------------------------------------------------------------*/
/* Reset the initialzation values											  */
/*----------------------------------------------------------------------------*/
MetalSensors::MetalSensors(void)
{
	sensorsInitialized			= false;
	cameraInitialized			= false;
	driverStationInitialized	= false;
}

void MetalSensors::InitializeSensors(void)
{
	if(!sensorsInitialized)
	{
		/* Drive Motor Constructs */
		leftFrontMotor			=	new Victor(LEFT_FRONT_MOTOR_PORT);
		leftRearMotor			=	new Victor(LEFT_REAR_MOTOR_PORT);
		rightFrontMotor			=	new Victor(RIGHT_FRONT_MOTOR_PORT);
		rightRearMotor			=	new Victor(RIGHT_REAR_MOTOR_PORT);

		/* Arm Motor Constructs */
		clawMotor				=	new Victor(CLAW_MOTOR_PORT);;
		armMotor				=	new Victor(ARM_MOTOR_PORT);

		/* Encoder Constructs */
		leftEncoder				=	new Encoder(LEFT_ENCODER_A_PORT,LEFT_ENCODER_B_PORT,(LEFT_SIDE_INVERTED==INVERTED),Encoder::k4X);
		rightEncoder			=	new Encoder(RIGHT_ENCODER_A_PORT,RIGHT_ENCODER_B_PORT,(RIGHT_SIDE_INVERTED==INVERTED),Encoder::k4X);
		leftEncoder->SetDistancePerPulse(PI*WHEEL/PULSES);
		leftEncoder->Reset();
		leftEncoder->Start();
		rightEncoder->SetDistancePerPulse(PI*WHEEL/360);
		rightEncoder->Reset();
		rightEncoder->Start();


		/* Gyro & Accelerometer Constructs */
		//gyro					= 	new Gyro(1);
		//accelerometer			= 	new Accelerometer(ACCELEROMETER_PORT);
		/* IR Line Sensor Constructs */
		//leftLineSensor			=	new DigitalInput(LEFT_LINE_SENSOR_PORT);
		//middleLineSensor		=	new DigitalInput(MIDDLE_LINE_SENSOR_PORT);
		//rightLineSensor			=	new DigitalInput(RIGHT_LINE_SENSOR_PORT);
		//tripSensor				=	new DigitalInput(TRIP_SENSOR_PORT);

		/* Limit Switches */
		//upperClawLimit			=	new DigitalInput(UPPER_CLAW_LIMIT_PORT);
		//lowerClawLimit		=	new DigitalInput(LOWER_CLAW_LIMIT_PORT);

		/* Piston Magnetic Sensors */
		leftLowMagnet			=	new DigitalInput(LEFT_LOW_MAGNET_PORT);
		leftHighMagnet			=	new DigitalInput(LEFT_HIGH_MAGNET_PORT);
		rightLowMagnet			=	new DigitalInput(RIGHT_LOW_MAGNET_PORT);
		rightHighMagnet			=	new DigitalInput(RIGHT_HIGH_MAGNET_PORT);

		/* Relays, Solenoids, Compressor Constructs */
		extensionSolenoid		=	new Solenoid(EXTENSION_SOLENOID_PORT);
		//clawSolenoid			=	new Solenoid(CLAW_SOLENOID_PORT);
		shiftingSolenoid		=	new Solenoid(SHIFTING_SOLENOID_PORT);
		minibotSolenoid			=	new Solenoid(MINIBOT_SOLENOID_PORT);
		guideSolenoid			= 	new Solenoid(GUIDE_SOLENOID_PORT);
		compressor				=	new Compressor(PRESSURE_SWITCH_PORT, COMPRESSOR_PORT);
		compressor->Start();

		whiteLight				=	new Relay(WHITE_LIGHT_PORT);
		redLight				=	new Relay(RED_LIGHT_PORT);
		blueLight				=	new Relay(BLUE_LIGHT_PORT);

		/* RPM Sensors */
		leftFrontPositiveRPM	=	new AnalogChannel(2,LEFT_FRONT_POSITIVE_RPM_PORT);
		leftFrontNegativeRPM	=	new AnalogChannel(2,LEFT_FRONT_NEGATIVE_RPM_PORT);
		leftRearPositiveRPM		=	new AnalogChannel(2,LEFT_REAR_POSITIVE_RPM_PORT);
		leftRearNegativeRPM		=	new AnalogChannel(2,LEFT_REAR_NEGATIVE_RPM_PORT);
		rightFrontPositiveRPM	=	new AnalogChannel(2,RIGHT_FRONT_POSITIVE_RPM_PORT);
		rightFrontNegativeRPM	=	new AnalogChannel(2,RIGHT_FRONT_NEGATIVE_RPM_PORT);
		rightRearPositiveRPM	=	new AnalogChannel(2,RIGHT_REAR_POSITIVE_RPM_PORT);
		rightRearNegativeRPM	=	new AnalogChannel(2,RIGHT_REAR_NEGATIVE_RPM_PORT);

		/* Potentiometers */
		armPotentiometer		=	new AnalogChannel(ARM_POTENTIOMETER_PORT);

		/* Timers */
		shiftingTimeout			= 	new Timer();
		shiftingWait			=	new Timer();
		cameraTimer				=	new Timer();

		/* PD controller Constructs */
		drivePDController		=	new MetalPD(DRIVE_PD_CONTROLLER_PGAIN,DRIVE_PD_CONTROLLER_DGAIN);
		armPDController			=	new MetalPD(ARM_PD_CONTROLLER_PGAIN,ARM_PD_CONTROLLER_DGAIN);

		/* Initialization of all sensors has completed! */
		sensorsInitialized		=	true;
		reference				=	0;
	}
}

void MetalSensors::InitializeDriverStation(void)
{
	if(!driverStationInitialized)
	{
		/* Driver Station Initalization */
		driverStation				= 	DriverStation::GetInstance();
		driverStationDisplay		=	DriverStationLCD::GetInstance();
		/* Initialization of driver station interfaces has completed! */
		driverStationInitialized	=	true;
	}
}

void MetalSensors::InitializeCamera(void)
{
	if(cameraInitialized == false) {
		Wait(7.0);
		/* Initialize the camera now since it isn't initialized */
		camera = &AxisCamera::GetInstance();

		/* Set resolution, compression, brightness */
		camera->WriteResolution(AxisCamera::kResolution_320x240);
		camera->WriteCompression(10);
		camera->WriteBrightness(0);
		camera->WriteExposureControl(AxisCameraParams::kExposure_FlickerFree60Hz);
		camera->WriteRotation(AxisCameraParams::kRotation_180);

		driverStationDisplay->PrintfLine(DriverStationLCD::kUser_Line1,"Camera init");
		driverStationDisplay->UpdateLCD();
		/* The camera is now initialized, so set the check to true */
		cameraInitialized = true;
	}
}
