#include "joyDrivers.h"


joyStruct driverXboxOne(sensor_msgs::Joy msg){
	joyStruct joy;

	joy.stamp = ros::Time::now();
	joy.buttonA = msg.buttons[0];
	joy.buttonB = msg.buttons[1];
	joy.buttonX = msg.buttons[2];
	joy.buttonY = msg.buttons[3];
	joy.buttonL1 = msg.buttons[4];
	joy.buttonR1 = msg.buttons[5];
	joy.buttonSelect = msg.buttons[6];
	joy.buttonStart = msg.buttons[7];

	joy.LstickHor = msg.axes[0];
	joy.LstickVer = msg.axes[1];
	joy.L2 = msg.axes[2];
	joy.RstickHor = msg.axes[3];
	joy.RstickVer = msg.axes[4];
	joy.R2 = msg.axes[5];

	if(msg.axes[6] == 1){
		joy.buttonLeft = 1;
	    joy.buttonRight = 0;
	}
	else if(msg.axes[6] == -1){
		joy.buttonLeft = 0;
	    joy.buttonRight = 1;
	}
	else{
		joy.buttonLeft = 0;
	    joy.buttonRight = 0;
	}

	if(msg.axes[7] == 1){
		joy.buttonUp = 1;
	    joy.buttonDown = 0;
	}
	else if(msg.axes[7] == -1){
		joy.buttonUp = 0;
	    joy.buttonDown = 1;
	}
	else{
		joy.buttonUp = 0;
	    joy.buttonDown = 0;
	}

	return joy;
}

joyStruct driverXbox360(sensor_msgs::Joy msg){
	joyStruct joy;

	joy.stamp = ros::Time::now();
	joy.buttonA = msg.buttons[0];
	joy.buttonB = msg.buttons[1];
	joy.buttonX = msg.buttons[2];
	joy.buttonY = msg.buttons[3];
	joy.buttonL1 = msg.buttons[4];
	joy.buttonR1 = msg.buttons[5];
	joy.buttonSelect = msg.buttons[6];
	joy.buttonStart = msg.buttons[7];
	joy.buttonLeft = msg.buttons[11];
    joy.buttonRight = msg.buttons[12];
	joy.buttonUp = 0;
	joy.buttonDown = 0;

	joy.LstickHor = msg.axes[0];
	joy.LstickVer = msg.axes[1];
	joy.L2 = msg.axes[2];
	joy.RstickHor = msg.axes[3];
	joy.RstickVer = msg.axes[4];
	joy.R2 = msg.axes[5];

	return joy;
}

joyStruct driverXbox360Wired(sensor_msgs::Joy msg){
	joyStruct joy;

	joy.stamp = ros::Time::now();
	joy.buttonA = msg.buttons[0];
	joy.buttonB = msg.buttons[1];
	joy.buttonX = msg.buttons[2];
	joy.buttonY = msg.buttons[3];
	joy.buttonL1 = msg.buttons[4];
	joy.buttonR1 = msg.buttons[5];
	joy.buttonSelect = msg.buttons[6];
	joy.buttonStart = msg.buttons[7];

	joy.LstickHor = msg.axes[0];
	joy.LstickVer = msg.axes[1];
	joy.L2 = msg.axes[2];
	joy.RstickHor = msg.axes[3];
	joy.RstickVer = msg.axes[4];
	joy.R2 = msg.axes[5];

	if(msg.axes[6] == 1){
		joy.buttonLeft = 1;
	    joy.buttonRight = 0;
	}
	else if(msg.axes[6] == -1){
		joy.buttonLeft = 0;
	    joy.buttonRight = 1;
	}
	else{
		joy.buttonLeft = 0;
	    joy.buttonRight = 0;
	}

	if(msg.axes[7] == 1){
		joy.buttonUp = 1;
	    joy.buttonDown = 0;
	}
	else if(msg.axes[7] == -1){
		joy.buttonUp = 0;
	    joy.buttonDown = 1;
	}
	else{
	joy.buttonUp = 0;
	    joy.buttonDown = 0;
	}

		return joy;
}



void printJoyValues(joyStruct joy){

	std::cout << "Time: " << "\t" << joy.stamp.toSec() << "\n";
	std::cout << "Buttons" << "\t";
	std::cout << joy.buttonA << " " << joy.buttonB << " "
	          << joy.buttonX << " "<< joy.buttonY << " "
			  << joy.buttonR1 << " " << joy.buttonL1 << " "
	          << joy.buttonSelect << " "<< joy.buttonStart << " "
			  << joy.buttonLeft << " " << joy.buttonRight << " "
	          << joy.buttonUp << " "<< joy.buttonDown;
	std::cout << std::setprecision(3) << "\nAxes" << "\t";
	std::cout << joy.LstickHor << "\t" << joy.LstickVer << "\t"
	          << joy.RstickHor << "\t"<< joy.RstickVer << "\t"
			  << joy.L2 << "\t" << joy.R2 << "\n";
}

