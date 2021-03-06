// Move and control algorithm. 
// The on/off switch will start the robot moving in a straight line
// If it detects an obstacle via the IR sensors it will stop and turn left until it is clear of the obstacle and then continue in s straight line
// If it detects an obstacle via the whisker sensors it will stop, reverse first, then turn left until it is clear of the obstacle,
// and then continue in s straight line. The whiskers are used as a backup in case the IR sensors fail

#include <stdio.h>
#include <phidget21.h>
#include <time.h>
#include <stdbool.h>
#include <stdlib.h>
#include "GeneralHandlers.h"
#include "RCXMotorSetup.h"

////////////////////////////////////////
//// PARAMETERS
//////////////////////////////////////

// Phidget Handles
CPhidgetInterfaceKitHandle IFK;
CPhidgetMotorControlHandle motoControl;

// set other parameters
int on_off_Index1 = 0;
int on_off_Index2 = 1;
int whisker1Index = 2;
int whisker2Index = 3;
int ir1Index = 3;
int ir2Index = 4;
int safeDistance = 25;
int safeDistance2 = 40; 
int defaultDirection = 0;
int defaultPower = 100;
int defaultTurnPower = 30;
int defaultReversePower = 30;
int irThreshold = 100;
int IanVal = 0;
int HallCount = 0;
bool exitFlag = false;
int tooClose = 0;
int tooClose2 = 0;
int activatedWhisker = 0;
bool turning = false;

/*
/////////////////////////////////////////////////////////////////////////
///// General Handlers
/////////////////////////////////////////////////////////////////////////

int AttachHandler(CPhidgetHandle phid, void *userptr) {
    int serialNo;
    const char *name;

    CPhidget_getDeviceName(phid, &name);
    CPhidget_getSerialNumber(phid, &serialNo);
    printf("%s %10d attached!\n", name, serialNo);

    return 0;
}

int DetachHandler(CPhidgetHandle phid, void *userptr) {
    int serialNo;
    const char *name;

    CPhidget_getDeviceName(phid, &name);
    CPhidget_getSerialNumber(phid, &serialNo);
    printf("%s %10d detached!\n", name, serialNo);

    return 0;
}

int ErrorHandler(CPhidgetHandle phid, void *userptr, int ErrorCode, const char *Description) {
    printf("Error handled. %d - %s\n", ErrorCode, Description);
    return 0;
}
*/
/*
/////////////////////////////////////////////////////////////////////////
///// RCX Motor setup
///////////////////////////////////////////////////////////////////////

int InputChangeHandlerMoto(CPhidgetMotorControlHandle MC, void *usrptr, int Index, int State) {
    printf("Input %d > State: %d\n", Index, State);
    return 0;
}

int VelocityChangeHandler(CPhidgetMotorControlHandle MC, void *usrptr, int Index, double Value) {
    //printf("Motor %d > Current Speed: %f\n", Index, Value);
    return 0;
}

int CurrentChangeHandler(CPhidgetMotorControlHandle MC, void *usrptr, int Index, double Value) {
    printf("Motor: %d > Current Draw: %f\n", Index, Value);
    return 0;
}

int display_properties_moto(CPhidgetMotorControlHandle phid) {
    int serialNo, version, numInputs, numMotors;
    const char* ptr;

    CPhidget_getDeviceType((CPhidgetHandle) phid, &ptr);
    CPhidget_getSerialNumber((CPhidgetHandle) phid, &serialNo);
    CPhidget_getDeviceVersion((CPhidgetHandle) phid, &version);

    CPhidgetMotorControl_getInputCount(phid, &numInputs);
    CPhidgetMotorControl_getMotorCount(phid, &numMotors);

    printf("%s\n", ptr);
    printf("Serial Number: %10d\nVersion: %8d\n", serialNo, version);
    printf("# Inputs: %d\n# Motors: %d\n", numInputs, numMotors);

    return 0;
}

int InitialiseRCXMotors() {
    int result;
    const char *err;

    //Declare a motor control handle
    //CPhidgetMotorControlHandle motoControl = 0;

    //create the motor control object
    CPhidgetMotorControl_create(&motoControl);

    //Set the handlers to be run when the device is plugged in or opened from software, 
    //unplugged or closed from software, or generates an error.
    CPhidget_set_OnAttach_Handler((CPhidgetHandle) motoControl, AttachHandler, NULL);
    CPhidget_set_OnDetach_Handler((CPhidgetHandle) motoControl, DetachHandler, NULL);
    CPhidget_set_OnError_Handler((CPhidgetHandle) motoControl, ErrorHandler, NULL);

    //Registers a callback that will run if an input changes.
    //Requires the handle for the Phidget, the function that will be called, 
    //and a arbitrary pointer that will be supplied to the callback function (may be NULL).
    CPhidgetMotorControl_set_OnInputChange_Handler(motoControl, InputChangeHandlerMoto, NULL);

    //Registers a callback that will run if a motor changes.
    //Requires the handle for the Phidget, the function that will be called, 
    //and a arbitrary pointer that will be supplied to the callback function (may be NULL).
    CPhidgetMotorControl_set_OnVelocityChange_Handler(motoControl, VelocityChangeHandler, NULL);

    //Registers a callback that will run if the current draw changes.
    //Requires the handle for the Phidget, the function that will be called, 
    //and a arbitrary pointer that will be supplied to the callback function (may be NULL).
    CPhidgetMotorControl_set_OnCurrentChange_Handler(motoControl, CurrentChangeHandler, NULL);

    //open the motor control for device connections
    CPhidget_open((CPhidgetHandle) motoControl, -1);

    //get the program to wait for a motor control device to be attached
    printf("Waiting for MotorControl to be attached....\n");
    if ((result = CPhidget_waitForAttachment((CPhidgetHandle) motoControl, 10000))) {
        CPhidget_getErrorDescription(result, &err);
        printf("Problem waiting for attachment: %s\n", err);
        return 0;
    }
    return 0;
}
*/
/////////////////////////////////////////////////////////////////////////
///// Servo motor setup
///////////////////////////////////////////////////////////////////////

CPhidgetAdvancedServoHandle servo = 0;
int ServoPos = 0; // 0 = 180 degrees, 1 = 130 degrees.

int PositionChangeHandler(CPhidgetAdvancedServoHandle ADVSERVO, void *usrptr, int Index, double Value) {
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

    //Set the handlers to be run when the device is plugged in or opened from software, 
    //unplugged or closed from software, or generates an error.
    CPhidget_set_OnAttach_Handler((CPhidgetHandle) servo, AttachHandler, NULL);
    CPhidget_set_OnDetach_Handler((CPhidgetHandle) servo, DetachHandler, NULL);
    CPhidget_set_OnError_Handler((CPhidgetHandle) servo, ErrorHandler, NULL);

    //Registers a callback that will run when the motor position is changed.
    //Requires the handle for the Phidget, the function that will be called, 
    //and an arbitrary pointer that will be supplied to the callback function (may be NULL).
    CPhidgetAdvancedServo_set_OnPositionChange_Handler(servo, PositionChangeHandler, NULL);

    //open the device for connections
    CPhidget_open((CPhidgetHandle) servo, -1);

    //get the program to wait for an advanced servo device to be attached
    printf("Waiting for Phidget to be attached....\n");
    if ((result = CPhidget_waitForAttachment((CPhidgetHandle) servo, 10000))) {
        CPhidget_getErrorDescription(result, &err);
        printf("Problem waiting for attachment: %s\n", err);
        return 0;
    }
    CPhidgetAdvancedServo_getAccelerationMin(servo, 0, &minAccel);
    CPhidgetAdvancedServo_setAcceleration(servo, 0, 182857.14);
    CPhidgetAdvancedServo_getVelocityMax(servo, 0, &maxVel);
    CPhidgetAdvancedServo_setVelocityLimit(servo, 0, 316);

    CPhidgetAdvancedServo_setPosition(servo, 0, 180);
    CPhidgetAdvancedServo_setEngaged(servo, 0, 1);
    ServoPos = 0;

    return 0;
}

/////////////////////////////////////////////////////////////////////////
///// Drive functions
///////////////////////////////////////////////////////////////////////

int SetServo(int position) { // 0 = Straight, 1 = rotate
    double minAccel, maxVel;
    CPhidgetAdvancedServo_setAcceleration(servo, 0, 182857.14);
    CPhidgetAdvancedServo_setVelocityLimit(servo, 0, 316);
    if (position == 0) {
        CPhidgetAdvancedServo_setPosition(servo, 0, 180);
        CPhidgetAdvancedServo_setEngaged(servo, 0, 1);
        ServoPos = 0;
    } else if (position = 1) {
        CPhidgetAdvancedServo_setPosition(servo, 0, 130);
        CPhidgetAdvancedServo_setEngaged(servo, 0, 1);
        ServoPos = 1;
    } else {
        printf("position for SetServo needs to be 0 or 1\n");
    }
    return 0;
}

int DriveMotorsStraight(int dir, int power) //dir:0 = Foreward, dir:1 = Backwards
{
    if (ServoPos == 1) {
        SetServo(0);
    }
    //Control the motor a bit.
    //Step 1: increase acceleration to 50, set target speed at 100
    CPhidgetMotorControl_setAcceleration(motoControl, 0, 20.00);
    CPhidgetMotorControl_setAcceleration(motoControl, 1, 20.00);
    if (dir == 0) {
        CPhidgetMotorControl_setVelocity(motoControl, 0, power);
        CPhidgetMotorControl_setVelocity(motoControl, 1, -power);
    } else if (dir == 1) {
        CPhidgetMotorControl_setVelocity(motoControl, 0, -power);
        CPhidgetMotorControl_setVelocity(motoControl, 1, power);
    } else {
        printf("dir for DriveMotorsStraight needs to be 0 or 1\n");
    }
    return 0;
}

int DriveMotorsStop() {
    //Control the motor a bit.
    //Step 1: increase acceleration to 50, set target sped at 100
    CPhidgetMotorControl_setAcceleration(motoControl, 0, 20.00);
    CPhidgetMotorControl_setAcceleration(motoControl, 1, 20.00);
    CPhidgetMotorControl_setVelocity(motoControl, 0, 0);
    CPhidgetMotorControl_setVelocity(motoControl, 1, 0);
    CPhidgetAdvancedServo_setEngaged(servo, 0, 0);
    return 0;
}

int DriveMotorsRotate(int dir, int power) //dir: 0 = antiClockwise, dir: 1 = Clockwise
{
    if (ServoPos == 0) {
        SetServo(1);
    }
    //Control the motor a bit.
    //Step 1: increase acceleration to 50, set target sped at 100
    CPhidgetMotorControl_setAcceleration(motoControl, 0, 20.00);
    CPhidgetMotorControl_setAcceleration(motoControl, 1, 20.00);
    if (dir == 0) {
        CPhidgetMotorControl_setVelocity(motoControl, 0, power);
        CPhidgetMotorControl_setVelocity(motoControl, 1, -power);
    } else if (dir == 1) {
        CPhidgetMotorControl_setVelocity(motoControl, 0, -power);
        CPhidgetMotorControl_setVelocity(motoControl, 1, power);
    } else {
        printf("dir for DriveMotorsRotate needs to be 0 or 1\n");
    }
    return 0;
}

int InitialiseMotors() {
    InitialiseRCXMotors(&motoControl);
    InitialiseServoMotors();
    return 0;
}

int testMotor0() {
    return 0;
    //write test for Motor0
}

int testMotor1() {
    //write test for Motor1
    return 0;
}

int testServo() {
    //write test for Servo
    return 0;
}

int ShutDownMotors() {
    //subroutine to close the motor control phidget and delete the object we created
    printf("Closing...\n");
    CPhidget_close((CPhidgetHandle) motoControl);
    CPhidget_delete((CPhidgetHandle) motoControl);
    CPhidget_close((CPhidgetHandle) servo);
    CPhidget_delete((CPhidgetHandle) servo);

    //all done, exit
    return 0;
}


/////////////////////////////////////////////////////////////////////////
///// Advanced Drive functions
///////////////////////////////////////////////////////////////////////

//rotate for set number of hall counts (clicks) at power pwr,
// in direction dir (0 = anticlockwise, 1 = clockwise)
void Turn(int dir, int clicks, int pwr) {
    int start_count = HallCount;
    int end_count = start_count + clicks;
    DriveMotorsRotate(dir, pwr);
    while (HallCount < end_count);
    //DriveMotorsStop();
}

//Reverse for set number of hall counts (clicks) at power pwr
void Reverse(int sec, int pwr) {
    int start_time = HallCount;
    int end_time = sec + start_time;
    DriveMotorsStraight(1, pwr);
    while (HallCount < end_time);
    //DriveMotorsStop();
}

/////////////////////////////////////////////////////////////////////////
///// Interface Kit Setup - For IR Sensor, Whiskers and On / Off Switch
/////////////////////////////////////////////////////////////////////////

// distance conversion function for IR sensor
double distanceConversion(int sensorValue) {
    return 4800/(sensorValue - 20);
}

// function to check if at the safe distance safeDist, or greater based on IR sensor reading
bool atSafeDistance(int index, int sensorValue) {
	
    //int sensor1Value;
    //int sensor2Value;
    //double dist1;
    //double dist2;
    double dist;

    // get sensor readings
    //CPhidgetInterfaceKit_getSensorValue(IFK, ir1Index, &sensor1Value);
    //CPhidgetInterfaceKit_getSensorValue(IFK, ir2Index, &sensor2Value);
    // calculate distance
    //dist1 = distanceConversion(sensor1Value);    
    //dist2 = distanceConversion(sensor2Value);   
    dist = distanceConversion(sensorValue);

    //printf("Sensors values: >>>>>>>> %i - %i\n", sensor1Value, sensor2Value);
    //printf("Distances from sensors: >>>>>>>> %f - %f\n", dist1, dist2);

    //printf("Sensor %i value: >>>>>>>> %i \n", index, sensorValue);
    printf("Distance from sensor %i: >>>>>>>> %f \n", index, dist);

    return dist >= safeDistance ;
}

// function to check if at the safe distance safeDist, or greater based on IR sensor reading
bool atSafeDistance2(int index, int sensorValue) {
	
    //int sensor1Value;
    //int sensor2Value;
    //double dist1;
    //double dist2;
    double dist;

    // get sensor readings
    //CPhidgetInterfaceKit_getSensorValue(IFK, ir1Index, &sensor1Value);
    //CPhidgetInterfaceKit_getSensorValue(IFK, ir2Index, &sensor2Value);
    // calculate distance
    //dist1 = distanceConversion(sensor1Value);    
    //dist2 = distanceConversion(sensor2Value);   
    dist = distanceConversion(sensorValue);

    //printf("Sensors values: >>>>>>>> %i - %i\n", sensor1Value, sensor2Value);
    //printf("Distances from sensors: >>>>>>>> %f - %f\n", dist1, dist2);

    //printf("Sensor %i value: >>>>>>>> %i \n", index, sensorValue);
    printf("Distance from sensor %i: >>>>>>>> %f \n", index, dist);

    return dist >= safeDistance2 ;
}

//callback that will run if an input changes.
//Index - Index of the input that generated the event, State - boolean (0 or 1) representing the input state (on or off)
int InputChangeHandler(CPhidgetInterfaceKitHandle IFK, void *usrptr, int Index, int State) {

    // show input readings
    //printf("Digital Input: %d > State: %d\n", Index, State);

	if(Index == 7){
		HallCount ++;
	}

	if(Index == 0){
		IanVal = State;
	}

    // increase IR threshold temporarily in order to prevent another event from triggering
    //CPhidgetInterfaceKit_setSensorChangeTrigger(IFK, ir1Index, 10000);

    // choose correct response based on input source
    if (Index == on_off_Index1 || Index == on_off_Index2) {
    	printf("On/Off switch pressed...\n");		
    	exitFlag = true;            
    }

    if (Index == whisker1Index  && State == 1) {
	
    	printf("whisker1 switch pressed...\n");
	activatedWhisker = 1;
	//WhiskerChangeResponse(IFK, Index, State);            
    }

    if (Index == whisker2Index && State == 1) {
	printf("whisker2 switch pressed...");
	activatedWhisker = 2;
	//WhiskerChangeResponse(IFK, Index, State);
    }

    // reset ir threshold to default value
    //CPhidgetInterfaceKit_setSensorChangeTrigger(IFK, ir1Index, irThreshold);

    return 0;
}


//callback that will run if the sensor value changes by more than the OnSensorChange trigger.
//Index - Index of the sensor that generated the event, Value - the sensor read value
int SensorChangeHandler(CPhidgetInterfaceKitHandle IFK, void *usrptr, int Index, int Value) {
    // show sensor readings
    //printf("Sensor: %d > Value: %d\n", Index, Value);
    
    // increase IR threshold temporarily in order to prevent another event from triggering
    //int currentTrigger;
    //CPhidgetInterfaceKit_getSensorChangeTrigger(IFK, ir1Index, &currentTrigger);
    //CPhidgetInterfaceKit_setSensorChangeTrigger(IFK, ir1Index, 10000);

    // choose correct response based on sensor reading
    if ((Index == ir1Index /*|| Index == ir2Index*/) && (Value > 80 && Value < 530)) {
    //if (Index == ir1Index) {
	if (atSafeDistance(Index, Value)) {
           tooClose = false;	   
           //printf("Comfortable distance...\n");
        }
	else {
           tooClose = true;
           //printf("Getting a tad too close...\n");
	}
	if (atSafeDistance2(Index, Value)) {
           tooClose2 = false;	   
           //printf("Comfortable distance...\n");
        }
	else {
           tooClose2 = true;
           //printf("Getting a tad too close...\n");
	}
    }

    // reset ir threshold to default value
    //CPhidgetInterfaceKit_setSensorChangeTrigger(IFK, ir1Index, currentTrigger);

    return 0;
}

//Display the properties of the attached phidget to the screen.  We will be displaying the name, serial number and version of the attached device.
//Will also display the number of inputs, outputs, and analog inputs on the interface kit as well as the state of the ratiometric flag
//and the current analog sensor sensitivity.
int display_properties_ifkit(CPhidgetInterfaceKitHandle phid)
{
	int serialNo, version, numInputs, numOutputs, numSensors, triggerVal, ratiometric, i;
	const char* ptr;

	CPhidget_getDeviceType((CPhidgetHandle)phid, &ptr);
	CPhidget_getSerialNumber((CPhidgetHandle)phid, &serialNo);
	CPhidget_getDeviceVersion((CPhidgetHandle)phid, &version);

	CPhidgetInterfaceKit_getInputCount(phid, &numInputs);
	CPhidgetInterfaceKit_getOutputCount(phid, &numOutputs);
	CPhidgetInterfaceKit_getSensorCount(phid, &numSensors);
	CPhidgetInterfaceKit_getRatiometric(phid, &ratiometric);

	printf("%s\n", ptr);
	printf("Serial Number: %10d\nVersion: %8d\n", serialNo, version);
	printf("# Digital Inputs: %d\n# Digital Outputs: %d\n", numInputs, numOutputs);
	printf("# Sensors: %d\n", numSensors);
	printf("Ratiometric: %d\n", ratiometric);

	for(i = 0; i < numSensors; i++)
	{
		CPhidgetInterfaceKit_getSensorChangeTrigger (phid, i, &triggerVal);

		printf("Sensor#: %d > Sensitivity Trigger: %d\n", i, triggerVal);
	}

	return 0;
}

int initialiseInterfacekit() {
    int result;
    const char *err;

    printf("Initialising Interface kit...");

    //Declare an InterfaceKit handle
    CPhidgetInterfaceKitHandle ifKit = 0;

    //create the InterfaceKit object
    CPhidgetInterfaceKit_create(&ifKit);

    //Set the handlers to be run when the device is plugged in or opened from software, 
    //unplugged or closed from software, or generates an error.
    CPhidget_set_OnAttach_Handler((CPhidgetHandle) ifKit, AttachHandler, NULL);
    CPhidget_set_OnDetach_Handler((CPhidgetHandle) ifKit, DetachHandler, NULL);
    CPhidget_set_OnError_Handler((CPhidgetHandle) ifKit, ErrorHandler, NULL);

    //Registers a callback that will run if an input changes.
    //Requires the handle for the Phidget, the function that will be called, 
    //and an arbitrary pointer that will be supplied to the callback function (may be NULL).
    CPhidgetInterfaceKit_set_OnInputChange_Handler(ifKit, InputChangeHandler, NULL);

    //Registers a callback that will run if the sensor value changes by more than the OnSensorChange trig-ger.
    //Requires the handle for the IntefaceKit, the function that will be called, 
    //and an arbitrary pointer that will be supplied to the callback function (may be NULL).
    CPhidgetInterfaceKit_set_OnSensorChange_Handler(ifKit, SensorChangeHandler, NULL);

    //open the interfacekit for device connections
    CPhidget_open((CPhidgetHandle) ifKit, -1);

    //get the program to wait for an interface kit device to be attached
    printf("Waiting for interface kit to be attached....");
    if ((result = CPhidget_waitForAttachment((CPhidgetHandle) ifKit, 10000))) {
        CPhidget_getErrorDescription(result, &err);
        printf("Problem waiting for attachment: %s\n", err);
        return 0;
    }

    //Display the properties of the attached interface kit device
    display_properties_ifkit(ifKit);

    //Change the sensitivity trigger of the sensors
    printf("Modifying sensor sensitivity trigger for phidget index %i", ir1Index);
    CPhidgetInterfaceKit_setSensorChangeTrigger(ifKit, ir1Index, 20);
    printf("Modifying sensor sensitivity trigger for phidget index %i", ir2Index);
    CPhidgetInterfaceKit_setSensorChangeTrigger(ifKit, ir2Index, 50);	

    return 0;
}

int ShutDownInterfacekit() {
    //subroutine to close the interface kit phidget and delete the objects we created
    printf("Closing...\n");
    CPhidget_close((CPhidgetHandle) IFK);
    CPhidget_delete((CPhidgetHandle) IFK);

    return 0;
}

/////////////////////////////////////////////////////////////////////////
///// Responses to obstacles and user inputs
/////////////////////////////////////////////////////////////////////////

// Code to respond to a detected change in the IR sensor 
void tooCloseResponse() {

    int sensorValue;
    int counts = 1;
    int dir;
    int random = rand() ;
    

    // set random direction
    dir = 0;

    // get IR sensor reading
    //CPhidgetInterfaceKit_getSensorValue(IFK, ir1Index, &sensorValue);

    // keep turning until obstacle is out of trajectory   
    turning = true; 

    printf(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Turning direction: %i\n", random);
    while (tooClose2 == 1) {                 	   	
       DriveMotorsRotate(dir, defaultTurnPower);
     //   CPhidgetInterfaceKit_getSensorValue(IFK, ir1Index, &sensorValue);
    //    counts = 1; // reduce duration for subsequent turns
    }

    tooClose = false;
    turning = false;    

    // Continue in a straight trajectory
    //DriveMotorsStraight(defaultDirection, defaultPower);
}

// Code to respond to a detected change in the on/off switch - this will stop and start the robot
//void On_Off_SwitchChangeResponse() {
   //if (value % 2 == 0) DriveMotorsStop();
   //else DriveMotorsStraight(defaultDirection, defaultPower);
//   if (pause == true) DriveMotorsStop(); else DriveMotorsStraight(defaultDirection, defaultPower);
//}

// Code to respond to a detected change in the whisker switches
void WhiskerChangeResponse() {

   int counts = 9;

   printf("Reverse now please or we'll crash\n");
   Reverse(8, defaultTurnPower );

   printf(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Turning direction: %i\n", activatedWhisker);
   Turn(activatedWhisker % 2 , counts, defaultTurnPower);
   activatedWhisker = 0;   
   
}

/////////////////////////////////////////////////////////////////////////
///// Main control loop
/////////////////////////////////////////////////////////////////////////

int main() //start moving
{
    time_t t;
    InitialiseMotors();
    initialiseInterfacekit();
    DriveMotorsStraight(defaultDirection, defaultPower);
    srand((unsigned) time(&t));

    exitFlag = false; // just in case
    turning = false;
    while(1){

   	if (activatedWhisker > 0) {
		WhiskerChangeResponse();
		DriveMotorsStraight(defaultDirection, defaultPower);
		printf("activatedWhisker = %i",  activatedWhisker);
	}
	
	if (tooClose == 1 && turning == false) {
		tooCloseResponse();
		DriveMotorsStraight(defaultDirection, defaultPower);
	}
        
        if (exitFlag) { 
        	printf("Exit flag detected. Breaking... \n ");
        	break;
	}

    }
    printf("\nDONE\n");
    DriveMotorsStop();
    ShutDownInterfacekit();
    ShutDownMotors();
    return 0;
}
