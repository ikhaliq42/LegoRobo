// - MotorControl functions

//use InitialiseMotors(); at start of program
//use ShutDownMotors(); at end of program
//use DriveMotorsStraight(dir, power) to go straight, dir:0 = Foreward, dir:1 = Backwards, power 30>100
//use DriveMotorsRotate(int dir, int power) to go rotate on the spot, dir: 0 = antiClockwise, dir: 1 = Clockwise, power 30>100
//use DriveMotorsStop() to stop the motors
//


#include <stdio.h>
#include <phidget21.h>

CPhidgetMotorControlHandle motoControl = 0;
CPhidgetAdvancedServoHandle servo = 0;
int ServoPos = 0; // 0 = 180 degrees, 1 = 130 degrees.


/////////////////////////////////////////////////////////////////////////
///// RCX Motor setup
///////////////////////////////////////////////////////////////////////

int AttachHandler(CPhidgetHandle MC, void *userptr)
{
	int serialNo;
	const char *name;

	CPhidget_getDeviceName (MC, &name);
	CPhidget_getSerialNumber(MC, &serialNo);
	printf("%s %10d attached!\n", name, serialNo);

	return 0;
}

int DetachHandler(CPhidgetHandle MC, void *userptr)
{
	int serialNo;
	const char *name;

	CPhidget_getDeviceName (MC, &name);
	CPhidget_getSerialNumber(MC, &serialNo);
	printf("%s %10d detached!\n", name, serialNo);

	return 0;
}

int ErrorHandler(CPhidgetHandle MC, void *userptr, int ErrorCode, const char *Description)
{
	printf("Error handled. %d - %s\n", ErrorCode, Description);
	return 0;
}


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

int InitialiseRCXMotors()
{
	int result;
	const char *err;

	//Declare a motor control handle
	//CPhidgetMotorControlHandle motoControl = 0;

	//create the motor control object
	CPhidgetMotorControl_create(&motoControl);

	//Set the handlers to be run when the device is plugged in or opened from software, unplugged or closed from software, or generates an error.
	CPhidget_set_OnAttach_Handler((CPhidgetHandle)motoControl, AttachHandler, NULL);
	CPhidget_set_OnDetach_Handler((CPhidgetHandle)motoControl, DetachHandler, NULL);
	CPhidget_set_OnError_Handler((CPhidgetHandle)motoControl, ErrorHandler, NULL);

	//Registers a callback that will run if an input changes.
	//Requires the handle for the Phidget, the function that will be called, and a arbitrary pointer that will be supplied to the callback function (may be NULL).
	CPhidgetMotorControl_set_OnInputChange_Handler (motoControl, InputChangeHandler, NULL);

	//Registers a callback that will run if a motor changes.
	//Requires the handle for the Phidget, the function that will be called, and a arbitrary pointer that will be supplied to the callback function (may be NULL).
	CPhidgetMotorControl_set_OnVelocityChange_Handler (motoControl, VelocityChangeHandler, NULL);

	//Registers a callback that will run if the current draw changes.
	//Requires the handle for the Phidget, the function that will be called, and a arbitrary pointer that will be supplied to the callback function (may be NULL).
	CPhidgetMotorControl_set_OnCurrentChange_Handler (motoControl, CurrentChangeHandler, NULL);

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

int InitialiseServoMotors() {

	int result;
	double curr_pos;
	const char *err;
	double minAccel, maxVel;

	//Declare an advanced servo handle
	//CPhidgetAdvancedServoHandle servo = 0;

	//create the advanced servo object
	CPhidgetAdvancedServo_create(&servo);

	//Set the handlers to be run when the device is plugged in or opened from software, unplugged or closed from software, or generates an error.
	CPhidget_set_OnAttach_Handler((CPhidgetHandle)servo, AttachHandler, NULL);
	CPhidget_set_OnDetach_Handler((CPhidgetHandle)servo, DetachHandler, NULL);
	CPhidget_set_OnError_Handler((CPhidgetHandle)servo, ErrorHandler, NULL);

	//Registers a callback that will run when the motor position is changed.
	//Requires the handle for the Phidget, the function that will be called, and an arbitrary pointer that will be supplied to the callback function (may be NULL).
	CPhidgetAdvancedServo_set_OnPositionChange_Handler(servo, PositionChangeHandler, NULL);

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
	ServoPos = 0;

	return 0;
}

/////////////////////////////////////////////////////////////////////////
///// Drive functions
///////////////////////////////////////////////////////////////////////

int SetServo(int position){ // 0 = Straight, 1 = rotate
	double minAccel, maxVel;
	//CPhidgetAdvancedServo_setAcceleration(servo, 0, 182857.14);
	//CPhidgetAdvancedServo_setVelocityLimit(servo, 0, 316);
	if (position == 0){
			CPhidgetAdvancedServo_setPosition (servo, 0, 180);
			CPhidgetAdvancedServo_setEngaged(servo, 0, 1);
			ServoPos = 0;
}
	else if (position = 1){
			CPhidgetAdvancedServo_setPosition (servo, 0, 130);
			CPhidgetAdvancedServo_setEngaged(servo, 0, 1);
			ServoPos = 1;
}
	else{
		printf("position for SetServo needs to be 0 or 1\n");
	}
	return 0;
}

int DriveMotorsStraight(int dir, int power) //dir:0 = Foreward, dir:1 = Backwards
{
	if (ServoPos == 1){
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

int DriveMotorsStop() //dir:0 = Foreward, dir:1 = Backwards
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
	if (ServoPos == 0){
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



int InitialiseMotors() {
	InitialiseRCXMotors();
	InitialiseServoMotors();
	return 0;
}

int testMotor0(){
	return 0;
	//write test for Motor0
}

int testMotor1(){
	//write test for Motor1
return 0;
}

int testServo(){
	//write test for Servo
return 0;
}

int ShutDownMotors()
{
	//since user input has been read, this is a signal to terminate the program so we will close the phidget and delete the object we created
	printf("Closing...\n");
	CPhidget_close((CPhidgetHandle)motoControl);
	CPhidget_delete((CPhidgetHandle)motoControl);
	CPhidget_close((CPhidgetHandle)servo);
	CPhidget_delete((CPhidgetHandle)servo);

	//all done, exit
	return 0;
}

int main(int argc, char* argv[]) //just to test functions
{
	InitialiseMotors();

	printf("Press any key to move foreward\n");
	getchar();
	
	DriveMotorsStraight(0, 100);
	printf("Press any key to rotate clockwise\n");
	getchar();

	DriveMotorsRotate(1, 30);
	printf("Press any key to move Forewards\n");
	getchar();

	DriveMotorsStraight(0, 100);
	printf("Press any key to rotate anticlockwise\n");
	getchar();

	DriveMotorsRotate(0, 30);
	printf("Press any key to move backwards\n");
	getchar();

	DriveMotorsStraight(1, 100);
	printf("Press any key to stop\n");
	getchar();

	DriveMotorsStop();
	ShutDownMotors();
	return 0;
}

