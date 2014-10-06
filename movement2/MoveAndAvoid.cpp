
// Move and control algorithm. 
// The on/off switch will start the robot moving in a straight line
// If it detects an obstacle via the IR sensors it will stop and turn left until it is clear of the obstacle and then continue in s straight line
// If it detects an obstacle via the whisker sensors it will stop, reverse first, then turn left until it is clear of the obstacle,
// and then continue in s straight line. The whiskers are used as a backup in case the IR sensors fail

#include <stdio.h>
#include <phidget21.h>

class MoveAndAvoid {
	
	/////////////////////////////////////////////////////////////////////////
	///// Main control loop
	/////////////////////////////////////////////////////////////////////////

	int main(int argc, char* argv[]) //start moving
	{	
		Robot robot;		
		
		// create robot object
		robot = new Robot();
		robot.initialiseInterfaceControl();
		robot.initialiseDriveControl();

		// wait for input
		while(true);
		return 0;
	}

};

