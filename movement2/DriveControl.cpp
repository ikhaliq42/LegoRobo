
#include <stdio.h>
#include <phidget21.h>
#include <time.h>

class DriveControl : public Robot {
    
	CPhidgetMotorControlHandle motoControl;
        CPhidgetAdvancedServoHandle servo;
	int servoPos; // 0 = 180 degrees, 1 = 130 degrees.

private:
        
	/////////////////////////////////////////////////////////////////////////
	///// RCX Motor setup
	///////////////////////////////////////////////////////////////////////

	int InputChangeHandler(CPhidgetMotorControlHandle MC, void *usrptr, int Index, int State)
	{
		printf("Input %d > State: %d\n", Index, State);
		return 0;
	}

	int VelocityChangeHandler(CPhidgetMotorControlHandle MC, void *usrptr, int Index, double Value)
	{
		//printf("Motor %d > Current Speed: %f\n", Index, Value);
		return 0;
	}

	int CurrentChangeHandler(CPhidgetMotorControlHandle MC, void *usrptr, int Index, double Value)
	{
		printf("Motor: %d > Current Draw: %f\n", Index, Value);
		return 0;
	}

	int InitialiseRCXMotors()
	{
		int result;
		const char *err;
                
		//Declare a motor control handle
		//CPhidgetMotorControlHandle motoControl = 0;

		//create the motor control object                
                //motoControl = new CPhidgetMotorControlHandle();
		CPhidgetMotorControl_create(&motoControl);

		//Set the handlers to be run when the device is plugged in or opened from software, unplugged or closed from software, or generates an error.
		CPhidget_set_OnAttach_Handler((CPhidgetHandle)motoControl, AttachHandler, this);
		CPhidget_set_OnDetach_Handler((CPhidgetHandle)motoControl, DetachHandler, this);
		CPhidget_set_OnError_Handler((CPhidgetHandle)motoControl, ErrorHandler, this);

		//Registers a callback that will run if an input changes.
		//Requires the handle for the Phidget, the function that will be called, and a arbitrary pointer that will be supplied to the callback function (may be NULL).
		CPhidgetMotorControl_set_OnInputChange_Handler (motoControl, InputChangeHandler, this);

		//Registers a callback that will run if a motor changes.
		//Requires the handle for the Phidget, the function that will be called, and a arbitrary pointer that will be supplied to the callback function (may be NULL).
		CPhidgetMotorControl_set_OnVelocityChange_Handler (motoControl, VelocityChangeHandler, this);

		//Registers a callback that will run if the current draw changes.
		//Requires the handle for the Phidget, the function that will be called, and a arbitrary pointer that will be supplied to the callback function (may be NULL).
		CPhidgetMotorControl_set_OnCurrentChange_Handler (motoControl, CurrentChangeHandler, this);

		//open the motor control for device connections
		CPhidget_open((CPhidgetHandle)motoControl, -1);

		//get the program to wait for a motor control device to be attached
		printf("Waiting for MotorControl to be attached....");
		if((result = CPhidget_waitForAttachment((CPhidgetHandle)motoControl, 10000)))
		{
			CPhidget_getErrorDescription(result, &err);
			printf("Problem waiting for attachment: %s\n", err);
			return 0;
		}
		return 0;
	}

	/////////////////////////////////////////////////////////////////////////
	///// Servo motor setup
	///////////////////////////////////////////////////////////////////////

	int PositionChangeHandler(CPhidgetAdvancedServoHandle ADVSERVO, void *usrptr, int Index, double Value)
	{
	//	printf("Motor: %d > Current Position: %f\n", Index, Value);
		return 0;
	}

	int InitialiseServoMotors() 
        {

		int result;
		double curr_pos;
		const char *err;
		double minAccel, maxVel;
                
                servoPos = 0;

		//Declare an advanced servo handle
		//CPhidgetAdvancedServoHandle servo = 0;

		//create the advanced servo object
		CPhidgetAdvancedServo_create(&servo);

		//Set the handlers to be run when the device is plugged in or opened from software, unplugged or closed from software, or generates an error.
		CPhidget_set_OnAttach_Handler((CPhidgetHandle)servo, AttachHandler, this);
		CPhidget_set_OnDetach_Handler((CPhidgetHandle)servo, DetachHandler, this);
		CPhidget_set_OnError_Handler((CPhidgetHandle)servo, ErrorHandler, this);

		//Registers a callback that will run when the motor position is changed.
		//Requires the handle for the Phidget, the function that will be called, 
		//and an arbitrary pointer that will be supplied to the callback function (may be NULL).
		CPhidgetAdvancedServo_set_OnPositionChange_Handler(servo, PositionChangeHandler, this);

		//open the device for connections
		CPhidget_open((CPhidgetHandle)servo, -1);

		//get the program to wait for an advanced servo device to be attached
		printf("Waiting for Phidget to be attached....");
		if((result = CPhidget_waitForAttachment((CPhidgetHandle)servo, 10000)))
		{
			CPhidget_getErrorDescription(result, &err);
			printf("Problem waiting for attachment: %s\n", err);
			return 0;
		}
		CPhidgetAdvancedServo_getAccelerationMin(servo, 0, &minAccel);
		CPhidgetAdvancedServo_setAcceleration(servo, 0, 182857.14);
		CPhidgetAdvancedServo_getVelocityMax(servo, 0, &maxVel);
		CPhidgetAdvancedServo_setVelocityLimit(servo, 0, 316);

		CPhidgetAdvancedServo_setPosition (servo, 0, 180);
		CPhidgetAdvancedServo_setEngaged(servo, 0, 1);
		servoPos = 0;

		return 0;
	}

	int SetServo(int position){ // 0 = Straight, 1 = rotate
		double minAccel, maxVel;
		//CPhidgetAdvancedServo_setAcceleration(servo, 0, 182857.14);
		//CPhidgetAdvancedServo_setVelocityLimit(servo, 0, 316);
		if (position == 0){
				CPhidgetAdvancedServo_setPosition (servo, 0, 180);
				CPhidgetAdvancedServo_setEngaged(servo, 0, 1);
				servoPos = 0;
	}
		else if (position = 1){
				CPhidgetAdvancedServo_setPosition (servo, 0, 130);
				CPhidgetAdvancedServo_setEngaged(servo, 0, 1);
				servoPos = 1;
	}
		else{
			printf("position for SetServo needs to be 0 or 1\n");
		}
		return 0;
	}

public:

	/////////////////////////////////////////////////////////////////////////
	///// Drive functions
	///////////////////////////////////////////////////////////////////////

	int DriveMotorsStraight(int dir, int power) //dir:0 = Foreward, dir:1 = Backwards
	{
		if (servoPos == 1){
			SetServo(0);
		}
		//Control the motor a bit.
		//Step 1: increase acceleration to 50, set target sped at 100
		CPhidgetMotorControl_setAcceleration (motoControl, 0, 20.00);
		CPhidgetMotorControl_setAcceleration (motoControl, 1, 20.00);
		if (dir == 0){
		CPhidgetMotorControl_setVelocity (motoControl, 0, power);
		CPhidgetMotorControl_setVelocity (motoControl, 1, -power);
		}
		else if (dir == 1){
			CPhidgetMotorControl_setVelocity (motoControl, 0, -power);
			CPhidgetMotorControl_setVelocity (motoControl, 1, power);
		}
		else{
			printf("dir for DriveMotorsStraight needs to be 0 or 1\n");
		}
		return 0;
	}

	int DriveMotorsStop() 
	{
		//Control the motor a bit.
		//Step 1: increase acceleration to 50, set target sped at 100
		CPhidgetMotorControl_setAcceleration (motoControl, 0, 20.00);
		CPhidgetMotorControl_setAcceleration (motoControl, 1, 20.00);
			CPhidgetMotorControl_setVelocity (motoControl, 0, 0);
			CPhidgetMotorControl_setVelocity (motoControl, 1, 0);
			CPhidgetAdvancedServo_setEngaged(servo, 0, 0);
		return 0;
	}

	int DriveMotorsRotate(int dir, int power) //dir: 0 = antiClockwise, dir: 1 = Clockwise
	{
		if (servoPos == 0){
			SetServo(1);
		}
		//Control the motor a bit.
		//Step 1: increase acceleration to 50, set target sped at 100
		CPhidgetMotorControl_setAcceleration (motoControl, 0, 20.00);
		CPhidgetMotorControl_setAcceleration (motoControl, 1, 20.00);
		if (dir == 0){
		CPhidgetMotorControl_setVelocity (motoControl, 0, power);
		CPhidgetMotorControl_setVelocity (motoControl, 1, -power);
		}
		else if (dir == 1){
			CPhidgetMotorControl_setVelocity (motoControl, 0, -power);
			CPhidgetMotorControl_setVelocity (motoControl, 1, power);
		}
		else{
			printf("dir for DriveMotorsRotate needs to be 0 or 1\n");
		}
		return 0;
	}

	int InitialiseMotors() 
        {
		InitialiseRCXMotors();
		InitialiseServoMotors();
		return 0;
	}

	int ShutDownMotors()
	{
		//subroutine to close the motor control phidget and delete the object we created
		printf("Closing...\n");
		CPhidget_close((CPhidgetHandle)motoControl);
		CPhidget_delete((CPhidgetHandle)motoControl);
		CPhidget_close((CPhidgetHandle)servo);
		CPhidget_delete((CPhidgetHandle)servo);

		//all done, exit
		return 0;
	}


	/////////////////////////////////////////////////////////////////////////
	///// Advanced Drive functions
	///////////////////////////////////////////////////////////////////////

	//rotate for duration sec (in seconds) at power pwr in direction dir (0 = anticlockwise, 1 = clockwise)
	void timedTurn(int dir, int sec, int pwr, int dist)
	{
		clock_t start_time = clock();
		clock_t end_time = sec * 1000 + start_time;
		DriveMotorsRotate(dir, pwr);
		while(clock() != end_time);
		DriveMotorsStop();
	}

	//Reverse for duration sec (in seconds) at power pwr
	void timedReverse(int sec, int pwr)
	{
		clock_t start_time = clock();
		clock_t end_time = sec * 1000 + start_time;
		DriveMotorsStraight(1, pwr);
		while(clock() != end_time);
		DriveMotorsStop();	
	}
	
	/////////////////////////////////////////////////////////////////////////
	///// General
	///////////////////////////////////////////////////////////////////////

	int display_properties(CPhidgetMotorControlHandle phid)
	{
		int serialNo, version, numInputs, numMotors;
		const char* ptr;

		CPhidget_getDeviceType((CPhidgetHandle)phid, &ptr);
		CPhidget_getSerialNumber((CPhidgetHandle)phid, &serialNo);
		CPhidget_getDeviceVersion((CPhidgetHandle)phid, &version);
	
		CPhidgetMotorControl_getInputCount(phid, &numInputs);
		CPhidgetMotorControl_getMotorCount(phid, &numMotors);

		printf("%s\n", ptr);
		printf("Serial Number: %10d\nVersion: %8d\n", serialNo, version);
		printf("# Inputs: %d\n# Motors: %d\n", numInputs, numMotors);

		return 0;
	}


	//// End of class definition

};
