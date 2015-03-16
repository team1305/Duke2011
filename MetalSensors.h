/*----------------------------------------------------------------------------*/
/* Class containing all external / internal interfaces on the robot			  */
/*----------------------------------------------------------------------------*/

#ifndef METALSENSORS_H_
#define METALSENSORS_H_
#include "WPILib.h"
#include "MetalPD.h"
//#include "ArcadePad.h"

/*------------------------------------------------------------------------------*/
/* Define all ports for the sensors	and input - these values correspond to		*/
/* the Channels on the robot and driver station								  	*/
/*------------------------------------------------------------------------------*/

/* Drive Motor Ports (Connected on Digital Sidecar DigitalOut) */
#define LEFT_FRONT_MOTOR_PORT			1
#define LEFT_REAR_MOTOR_PORT			2
#define RIGHT_FRONT_MOTOR_PORT			3
#define RIGHT_REAR_MOTOR_PORT			4

/* Arm Motor Ports (Connected on Digital Sidecar DigitalOut) */
#define CLAW_MOTOR_PORT					5
#define ARM_MOTOR_PORT					6

/* Encoder Ports (Connected on Digital Sidecar DigitalIn) */
#define	LEFT_ENCODER_A_PORT				4
#define	LEFT_ENCODER_B_PORT				5
#define	RIGHT_ENCODER_A_PORT			2
#define	RIGHT_ENCODER_B_PORT			3

/* IR Line Sensor Ports (Connected on Digital Sidecar DigitalIn) */
//#define	LEFT_LINE_SENSOR_PORT			1
//#define MIDDLE_LINE_SENSOR_PORT			2
//#define RIGHT_LINE_SENSOR_PORT			3
#define TRIP_SENSOR_PORT				1

/* Limit Switches Ports (Connected on Digital Sidecar DigitalIn)  */
#define UPPER_CLAW_LIMIT_PORT			8
#define LOWER_CLAW_LIMIT_PORT			9

/* Piston Magnetic Ports (Connected on Digital Sidecar DigitalIn)  */
#define LEFT_LOW_MAGNET_PORT			11//10
#define LEFT_HIGH_MAGNET_PORT			10//11
#define RIGHT_LOW_MAGNET_PORT			13//12
#define RIGHT_HIGH_MAGNET_PORT			12//13

/* Relays, Solenoids Ports (Connected on Solenoid Breakout Module)  */
#define EXTENSION_SOLENOID_PORT			1
#define MINIBOT_SOLENOID_PORT			3
#define GUIDE_SOLENOID_PORT				4
#define SHIFTING_SOLENOID_PORT			2
#define	WHITE_LIGHT_PORT				3
#define BLUE_LIGHT_PORT					2
#define RED_LIGHT_PORT					4


/* Compressor and Pressure Sensor Ports (Connected on Relays on Digital Sidecar & DigitalIn on Digital Sidecar)  */
#define COMPRESSOR_PORT					1
#define	PRESSURE_SWITCH_PORT			14

/* RPM Sensor Ports (Connected on Analog Breakout) */
#define LEFT_FRONT_POSITIVE_RPM_PORT		1
#define LEFT_FRONT_NEGATIVE_RPM_PORT		2
#define	LEFT_REAR_POSITIVE_RPM_PORT			3
#define	LEFT_REAR_NEGATIVE_RPM_PORT			4
#define	RIGHT_FRONT_POSITIVE_RPM_PORT		5
#define RIGHT_FRONT_NEGATIVE_RPM_PORT		6
#define	RIGHT_REAR_POSITIVE_RPM_PORT		7
#define RIGHT_REAR_NEGATIVE_RPM_PORT		8

/* Potentiometers Ports (Connected on Analog Breakout 2) */
#define ARM_POTENTIOMETER_PORT			2
#define GYRO_PORT						1

/*------------------------------------------------------------------------------*/
/* Additional Configuration for Sensors or Motors								*/
/*------------------------------------------------------------------------------*/

/* Drive Motor Configuration (1 is not inverted, -1 is inverted) */
#define LEFT_SIDE_INVERTED			1
#define RIGHT_SIDE_INVERTED			-1
#define INVERTED					-1
#define PULSES						250
#define WHEEL						5.01
#define PI							3.1415926538

/* Camera Configuration */
#define CAMERA_TIME_DELAY			7.5

/* PD controller Configuration (P and D Gains) */
#define DRIVE_PD_CONTROLLER_PGAIN	0.0
#define DRIVE_PD_CONTROLLER_DGAIN	0.0
//
#define ARM_PD_CONTROLLER_PGAIN		0.140
#define ARM_PD_CONTROLLER_DGAIN		0.175

class MetalSensors
{
public:
	/*----------------------------------------------------------------------------*/
	/* Simple functions for initialization etc.									  */
	/*----------------------------------------------------------------------------*/

	MetalSensors(void);
	void InitializeSensors(void);
	void InitializeDriverStation(void);
	void InitializeCamera(void);

	/*----------------------------------------------------------------------------*/
	/* All inputs and outputs on the robot are listed below						  */
	/*----------------------------------------------------------------------------*/

	/* Drive Motor Constructs */
	Victor			*leftFrontMotor;
	Victor			*rightFrontMotor;
	Victor			*leftRearMotor;
	Victor			*rightRearMotor;

	/* Arm Motor Constructs */
	Victor			*clawMotor;
	Victor			*armMotor;

	/* Encoder Constructs */
	Encoder			*leftEncoder;
	Encoder			*rightEncoder;

	/* Gyro & Accelerometer Constructs */
	//Gyro			*gyro;
	//Accelerometer	*accelerometer;

	/* IR Line Sensor Constructs */
	//DigitalInput 		*leftLineSensor;
	//DigitalInput		*middleLineSensor;
	//DigitalInput 		*rightLineSensor;
	//DigitalInput		*tripSensor;

	/* Limit Switches */
	//DigitalInput		*upperClawLimit;
	//DigitalInput		*lowerClawLimit;

	/* Piston Magnetic Sensors */
	DigitalInput		*leftLowMagnet;
	DigitalInput		*leftHighMagnet;
	DigitalInput		*rightLowMagnet;
	DigitalInput		*rightHighMagnet;

	/* Driver Station Constructs */
	DriverStation		*driverStation;
	DriverStationLCD	*driverStationDisplay;
	//ArcadePad			*iPad;


	/* Relays, Solenoids, Compressor Constructs */
	Solenoid			*extensionSolenoid;
	//Solenoid			*clawSolenoid;
	Solenoid			*shiftingSolenoid;
	Solenoid			*minibotSolenoid;
	Solenoid			*guideSolenoid;
	Compressor			*compressor;
	Relay				*whiteLight;
	Relay				*redLight;
	Relay				*blueLight;

	/* RPM Sensors */
	AnalogChannel		*leftFrontPositiveRPM;
	AnalogChannel		*leftFrontNegativeRPM;
	AnalogChannel		*leftRearPositiveRPM;
	AnalogChannel		*leftRearNegativeRPM;
	AnalogChannel		*rightFrontPositiveRPM;
	AnalogChannel		*rightFrontNegativeRPM;
	AnalogChannel		*rightRearPositiveRPM;
	AnalogChannel		*rightRearNegativeRPM;

	/* Potentiometers */
	AnalogChannel		*armPotentiometer;

	/* Camera */
	AxisCamera 		*camera;

	/* Timers */
	Timer			*shiftingTimeout;
	Timer			*shiftingWait;
	Timer			*cameraTimer;

	/* PD controller Constructs */
	MetalPD		*drivePDController;
	MetalPD		*armPDController;

	float reference;
	float previousReference;
private:
	bool sensorsInitialized;	bool cameraInitialized;
	bool driverStationInitialized;
};
#endif
