#ifndef _H_JOY_DRIVERS_
#define _H_JOY_DRIVERS_

#include "sensor_msgs/Joy.h"

//Incoming data from joysticks
struct joyStruct{
	ros::Time stamp; //Time at which message was received
	int buttonA;
	int buttonB;
	int buttonX;
	int buttonY;
	int buttonR1;
	int buttonL1;
	int buttonSelect;
	int buttonStart;
	int buttonLeft;
	int buttonRight;
	int buttonUp;
	int buttonDown;

	double LstickHor;
	double LstickVer;
	double RstickHor;
	double RstickVer;
	double L2;
	double R2;
};


//Driver for receiving data from a Xbox One controller
joyStruct driverXboxOne(sensor_msgs::Joy msg);

//Driver for receiving data from a Xbox 360 controller
joyStruct driverXbox360(sensor_msgs::Joy msg);

//Driver for receiving data from a wired Xbox 360 controller
joyStruct driverXbox360Wired(sensor_msgs::Joy msg);


//Function to print values received from joystick (debug purposes)
void printJoyValues(joyStruct joy);

#endif