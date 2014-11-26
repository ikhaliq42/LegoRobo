/* 
 * File:   Robot.cpp
 * Author: s1461613
 * 
 * Created on 09 October 2014, 15:51
 */

#include <cstdlib>
#include <stdio.h>
#include <math.h>
#include "Handlers.h"
#include "Robot.h"
#include "Globals.h"
#include "Pose.h"
#include <vector>



using namespace std;

Robot_ptr Robot::_instance;

Robot_ptr Robot::getInstance()
{
    if(Robot::_instance == NULL) Robot::_instance.reset(new Robot());
    return Robot::_instance;
}


Robot::Robot() {

    //create the motor control object
    CPhidgetMotorControl_create(&motoControl);
    //create the advanced servo object
    CPhidgetAdvancedServo_create(&servo);
    //create the InterfaceKit object
    CPhidgetInterfaceKit_create(&IFK);

    // set parameters
    on_off_Index1 = 0;
    on_off_Index2 = 1;
    whisker1Index = 2;
    whisker2Index = 3;
    exitFlag = false;
    startFlag = false;
    activatedWhisker = 0;
    IanVal = 0;
    HallCount = 0;
    tooClose = 0;
    tooClose2 = 0;
    turningLeft= false;
    turningRight= false;
    movingForwards= false;
    movingBackwards = false;
    stopped = true;
    ServoPos = 2;
    x = 0;
    y = 0;
    theta = 0;

    numOfTurnsToPerform = 5;

    // for vision
    objectFound = false;

    //Initialise odometery data    
    //moveCounter.push_back(0);
    //rotateCounter.push_back(0);
          
}






Robot::~Robot() {
}

/////////////////////////////////////////////////////////////////////////
///// RCX Motor setup
///////////////////////////////////////////////////////////////////////

int Robot::display_properties_moto(CPhidgetMotorControlHandle phid) {
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

int Robot::InitialiseRCXMotors() {
    int result;
    const char *err;

    //Declare a motor control handle
    //CPhidgetMotorControlHandle motoControl = 0;


    //Set the handlers to be run when the device is plugged in or opened from software, 
    //unplugged or closed from software, or generates an error.
    CPhidget_set_OnAttach_Handler((CPhidgetHandle) motoControl, Handlers::AttachHandler, this);
    CPhidget_set_OnDetach_Handler((CPhidgetHandle) motoControl, Handlers::DetachHandler, this);
    CPhidget_set_OnError_Handler((CPhidgetHandle) motoControl, Handlers::ErrorHandler, this);

    //Registers a callback that will run if an input changes.
    //Requires the handle for the Phidget, the function that will be called, 
    //and a arbitrary pointer that will be supplied to the callback function (may be NULL).
    CPhidgetMotorControl_set_OnInputChange_Handler(motoControl, Handlers::InputChangeHandlerMoto, this);

    //Registers a callback that will run if a motor changes.
    //Requires the handle for the Phidget, the function that will be called, 
    //and a arbitrary pointer that will be supplied to the callback function (may be NULL).
    CPhidgetMotorControl_set_OnVelocityChange_Handler(motoControl, Handlers::VelocityChangeHandler, this);

    //Registers a callback that will run if the current draw changes.
    //Requires the handle for the Phidget, the function that will be called, 
    //and a arbitrary pointer that will be supplied to the callback function (may be NULL).
    CPhidgetMotorControl_set_OnCurrentChange_Handler(motoControl, Handlers::CurrentChangeHandler, this);

    //open the motor control for device connections
    CPhidget_open((CPhidgetHandle) motoControl, -1);

    //get the program to wait for a motor control device to be attached
    printf("Waiting for MotorControl to be attached....\n");
    if ((result = CPhidget_waitForAttachment((CPhidgetHandle) motoControl, 10000))) {
        CPhidget_getErrorDescription(result, &err);
        printf("Problem waiting for attachment: %s\n", err);
        return 0;
    }
    
    //Display the properties of the attached motor control device
    display_properties_moto(motoControl);
    
    return 0;
}


/////////////////////////////////////////////////////////////////////////
///// Servo motor setup
///////////////////////////////////////////////////////////////////////

int Robot::InitialiseServoMotors() {

    int result;
    double curr_pos;
    const char *err;
    double minAccel, maxVel;

    //Declare an advanced servo handle
    //CPhidgetAdvancedServoHandle servo = 0;


    //Set the handlers to be run when the device is plugged in or opened from software, 
    //unplugged or closed from software, or generates an error.
    CPhidget_set_OnAttach_Handler((CPhidgetHandle) servo, Handlers::AttachHandler, this);
    CPhidget_set_OnDetach_Handler((CPhidgetHandle) servo, Handlers::DetachHandler, this);
    CPhidget_set_OnError_Handler((CPhidgetHandle) servo, Handlers::ErrorHandler, this);

    //Registers a callback that will run when the motor position is changed.
    //Requires the handle for the Phidget, the function that will be called, 
    //and an arbitrary pointer that will be supplied to the callback function (may be NULL).
    CPhidgetAdvancedServo_set_OnPositionChange_Handler(servo, Handlers::PositionChangeHandler, this);

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

int Robot::InitialiseMotors() {
    InitialiseRCXMotors();
    InitialiseServoMotors();
    return 0;
}

int Robot::ShutDownMotors() {
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
///// Drive functions
///////////////////////////////////////////////////////////////////////

int Robot::SetServo(int position) { // 0 = Straight, 1 = rotate
    DriveMotorsStop();
    Globals::wait(100);
    double minAccel, maxVel;
    CPhidgetAdvancedServo_setAcceleration(servo, 0, 182857.14);
    CPhidgetAdvancedServo_setVelocityLimit(servo, 0, 316);
    printf("ServoPos: %d\n", ServoPos);
    printf("Hallcount: %i\n", HallCount);
    if (ServoPos == 0){
        x = HallCount * sin(theta*M_PI/180) + x; 
        y = HallCount * cos(theta*M_PI/180) + y;         
        HallCount = 0;   
        //int i = (int) poses.size() - 1;
        //printf("X: %d, Y: %d, Theta: %f\n", poses.at(i).x, poses.at(i).y, poses.at(i).theta);
        
    }
    if (ServoPos == 1){
        int oldTheta = theta;
        theta = theta - HallCount * 2.62;
        while (theta >= 360) theta = theta - 360;
        while (theta < 0) theta = theta + 360;
        
        if (sqrt(pow(oldTheta - theta,2)) >= 60) {
            x = HallCount * sin(theta*M_PI/180) + x; 
            y = HallCount * cos(theta*M_PI/180) + y;  
            poses.push_back(Pose(x,y,theta));
            int i = (int) poses.size() - 1;
            xa[i] = x;
            ya[i] = y;
            printf("Pushed: X: %d, Y: %d, Theta: %f\n", poses.at(i).x, poses.at(i).y, poses.at(i).theta);
                printf("Actual: X: %d, Y: %d, Theta: %f\n", x, y, theta);
       
        }
        
        HallCount = 0;
    }
    if (position == 0) {        
        CPhidgetAdvancedServo_setPosition(servo, 0, 180);
        CPhidgetAdvancedServo_setEngaged(servo, 0, 1);
        ServoPos = 0;        
    } else if (position == 1) {
        CPhidgetAdvancedServo_setPosition(servo, 0, 130);
        CPhidgetAdvancedServo_setEngaged(servo, 0, 1);
        ServoPos = 1;
    } else {
        printf("position for SetServo needs to be 0 or 1\n");
    }
    Globals::wait(100);
    return 0;
}

int Robot::DriveMotorsStraight(int dir, int power) //dir:0 = Foreward, dir:1 = Backwards
{
    if (ServoPos != 0) {
        SetServo(0);
    }
    turningLeft= false;
    turningRight= false;
    if (dir == 0) { movingForwards = true; movingBackwards = false; }
    if (dir == 1) { movingForwards = false; movingBackwards = true; }
    stopped = false;
    //Control the motor a bit.
    //Step 1: increase acceleration to 50, set target speed at 100
    CPhidgetMotorControl_setAcceleration(motoControl, 0, 5.0);
    CPhidgetMotorControl_setAcceleration(motoControl, 1, 5.0);
    if (dir == 0) {
        CPhidgetMotorControl_setVelocity(motoControl, 0, power);
        CPhidgetMotorControl_setVelocity(motoControl, 1, power);
    } else if (dir == 1) {
        CPhidgetMotorControl_setVelocity(motoControl, 0, power);
        CPhidgetMotorControl_setVelocity(motoControl, 1, power);
    } else {
        printf("dir for DriveMotorsStraight needs to be 0 or 1\n");
    }
    return 0;
}

int Robot::DriveMotorsStop() {
    //Control the motor a bit.
    //Step 1: increase acceleration to 50, set target sped at 100
    turningLeft= false;
    turningRight= false;
    movingForwards= false;
    movingBackwards = false;
    stopped = true;
    CPhidgetMotorControl_setAcceleration(motoControl, 0, 50);
    CPhidgetMotorControl_setAcceleration(motoControl, 1, 50);
    CPhidgetMotorControl_setVelocity(motoControl, 0, 0);
    CPhidgetMotorControl_setVelocity(motoControl, 1, 0);
    CPhidgetAdvancedServo_setEngaged(servo, 0, 0);
    //ServoPos = 2;
    return 0;
    
}

int Robot::DriveMotorsRotate(int dir, int power) //dir: 0 = antiClockwise, dir: 1 = Clockwise
{
    if (ServoPos != 1) {
        SetServo(1);
    }
    movingForwards = false;
    movingBackwards = false;
    if (dir == 0) { turningLeft = true; turningRight = false; }
    if (dir == 1) { turningLeft = false; turningRight = true; }
    stopped = false;
    //Control the motor a bit.
    //Step 1: increase acceleration to 50, set target sped at 100
    CPhidgetMotorControl_setAcceleration(motoControl, 0, 5.0);
    CPhidgetMotorControl_setAcceleration(motoControl, 1, 5.0);
    if (dir == 0) {
        CPhidgetMotorControl_setVelocity(motoControl, 0, power);
        CPhidgetMotorControl_setVelocity(motoControl, 1, power);
    } else if (dir == 1) {
        CPhidgetMotorControl_setVelocity(motoControl, 0, -power);
        CPhidgetMotorControl_setVelocity(motoControl, 1, -power);
    } else {
        printf("dir for DriveMotorsRotate needs to be 0 or 1\n");
    }
    return 0;
}

/////////////////////////////////////////////////////////////////////////
///// Advanced Drive functions
///////////////////////////////////////////////////////////////////////

//rotate for set number of hall counts (clicks) at power pwr,
// in direction dir (0 = anticlockwise, 1 = clockwise)
void Robot::Turn(int dir, int clicks, int pwr) {
    int start_count = HallCount;
    int end_count = start_count + clicks;
    int end_count2 = start_count - clicks;
    DriveMotorsRotate(dir, pwr);
    while (HallCount < end_count && HallCount > end_count2);
    DriveMotorsStop();
}

// move for set number of hall counts (clicks) at power pwr,
// in direction dir (0 = forward, 1 = backwards)   
void Robot::Move(int dir, int clicks, int pwr) {
    int start_count = HallCount;
    int end_count = clicks + start_count;
    int end_count2 = start_count - clicks;
    DriveMotorsStraight(dir, pwr);
    while (HallCount < end_count && HallCount > end_count2);
    DriveMotorsStop();
}

//Reverse for set number of hall counts (clicks) at power pwr
void Robot::Reverse(int clicks, int pwr) {
    int start_count = HallCount;
    int end_count = clicks + start_count;
    int end_count2 = start_count - clicks;
    DriveMotorsRotate(0, pwr);
    while (HallCount < end_count && HallCount > end_count2);
    DriveMotorsStop();
}

/////////////////////////////////////////////////////////////////////////
///// Interface Kit Setup - For IR Sensor, 
//s and On / Off Switch
/////////////////////////////////////////////////////////////////////////

int Robot::display_properties_ifkit(CPhidgetInterfaceKitHandle phid) {
    int serialNo, version, numInputs, numOutputs, numSensors, triggerVal, ratiometric, i; 
    int minDataRate, maxDataRate, currentDataRate;
    const char* ptr;

    CPhidget_getDeviceType((CPhidgetHandle) phid, &ptr);
    CPhidget_getSerialNumber((CPhidgetHandle) phid, &serialNo);
    CPhidget_getDeviceVersion((CPhidgetHandle) phid, &version);

    CPhidgetInterfaceKit_getInputCount(phid, &numInputs);
    CPhidgetInterfaceKit_getOutputCount(phid, &numOutputs);
    CPhidgetInterfaceKit_getSensorCount(phid, &numSensors);
    CPhidgetInterfaceKit_getRatiometric(phid, &ratiometric);

    printf("%s\n", ptr);
    printf("Serial Number: %10d\nVersion: %8d\n", serialNo, version);
    printf("# Digital Inputs: %d\n# Digital Outputs: %d\n", numInputs, numOutputs);
    printf("# Sensors: %d\n", numSensors);
    printf("Ratiometric: %d\n", ratiometric);

    for (i = 0; i < numSensors; i++) {
        CPhidgetInterfaceKit_getSensorChangeTrigger(phid, i, &triggerVal);
        CPhidgetInterfaceKit_getDataRateMax(phid, i, &maxDataRate);
        CPhidgetInterfaceKit_getDataRateMin(phid, i, &minDataRate);
        CPhidgetInterfaceKit_getDataRate(phid, i, &currentDataRate);
        printf("Sensor#: %d > Sensitivity Trigger: %d\n", i, triggerVal);
        printf("Sensor#: %d > Min data rate: %d\n", i, minDataRate);
        printf("Sensor#: %d > Max data rate: %d\n", i, maxDataRate);
        printf("Sensor#: %d > Current data rate: %d\n", i, currentDataRate);
    }
    
    

    return 0;
}

int Robot::initialiseInterfacekit() {

    int result;
    const char *err;

    //Declare an InterfaceKit handle
    //CPhidgetInterfaceKitHandle IFK = 0;

    //Set the handlers to be run when the device is plugged in or opened from software, 
    //unplugged or closed from software, or generates an error.
    CPhidget_set_OnAttach_Handler((CPhidgetHandle) IFK, Handlers::AttachHandler, this);
    CPhidget_set_OnDetach_Handler((CPhidgetHandle) IFK, Handlers::DetachHandler, this);
    CPhidget_set_OnError_Handler((CPhidgetHandle) IFK, Handlers::ErrorHandler, this);

    //Registers a callback that will run if an input changes.
    //Requires the handle for the Phidget, the function that will be called, 
    //and an arbitrary pointer that will be supplied to the callback function (may be NULL).
    CPhidgetInterfaceKit_set_OnInputChange_Handler(IFK, Handlers::InputChangeHandler, this);

    //Registers a callback that will run if the sensor value changes by more than the OnSensorChange trigger.
    //Requires the handle for the IntefaceKit, the function that will be called, 
    //and an arbitrary pointer that will be supplied to the callback function (may be NULL).
    CPhidgetInterfaceKit_set_OnSensorChange_Handler(IFK, Handlers::SensorChangeHandler, this);

    //open the interfacekit for device connections
    CPhidget_open((CPhidgetHandle) IFK, -1);

    //get the program to wait for an interface kit device to be attached
    printf("\nWaiting for interface kit to be attached....\n");
    if ((result = CPhidget_waitForAttachment((CPhidgetHandle) IFK, 10000))) {
        CPhidget_getErrorDescription(result, &err);
        printf("\nProblem waiting for attachment: %s\n", err);
        return 0;
    }

    //Display the properties of the attached interface kit device
    display_properties_ifkit(IFK);

    //Change the sensitivity trigger of the sensors
    printf("Modifying sensor sensitivity trigger for phidget index ", ir1Index);
    CPhidgetInterfaceKit_setSensorChangeTrigger(IFK, ir1Index, irThreshold);
    printf("Modifying sensor sensitivity trigger for phidget index ", ir2Index);
    CPhidgetInterfaceKit_setSensorChangeTrigger(IFK, ir2Index, irThreshold);

}

int Robot::ShutDownInterfacekit() {
    //subroutine to close the interface kit phidget and delete the objects we created
    printf("Closing...\n");
    CPhidget_close((CPhidgetHandle) IFK);
    CPhidget_delete((CPhidgetHandle) IFK);

    return 0;
}

/////////////////////////////////////////////////////////////////////////
///// Responses to obstacles and user inputs
/////////////////////////////////////////////////////////////////////////

// function to check if at the safe distance safeDist, or greater based on IR sensor reading

bool Robot::atSafeDistance(int index, int sensorValue, int safeDist) {

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
    if (sensorValue < 80) {
        dist = 150;
    }
    else {
    dist = Globals::distanceConversion(sensorValue);
    }
    //printf("Sensors values: >>>>>>>> %i - %i\n", sensor1Value, sensor2Value);
    //printf("Distances from sensors: >>>>>>>> %f - %f\n", dist1, dist2);

    //printf("Sensor %i value: >>>>>>>> %i \n", index, sensorValue);
    //printf("Distance from sensor %i: >>>>>>>> %f \n", index, dist);

    return dist >= safeDist;
}

int Robot::get_sensor_reading(int irIndex) {
    
    int val;    
    CPhidgetInterfaceKit_getSensorValue(IFK, irIndex, &val);
    return val;
}

int Robot::get_averaged_sensor_reading(int irIndex, int num_readings) {
    
    int val;
    int sum = 0;
    
    for (int n = 1; n <= num_readings; n++) {
        CPhidgetInterfaceKit_getSensorValue(IFK, irIndex, &val);       
        sum=sum+val;
    }
    
    return (int)(sum/num_readings);
}

// Code to respond to a detected change in the IR sensor 

void Robot::tooCloseResponse(int dir) {

    int sensorValue;
    int counts = 1;
    int random = rand();
    int sensor;

    // set random direction
    //dir = 0;

    // get IR sensor reading
    //CPhidgetInterfaceKit_getSensorValue(IFK, ir1Index, &sensorValue);
    
    //choose sensor to use
    if (dir == 0) sensor = ir2Index; else sensor = ir1Index;
    
    // keep turning until obstacle is out of trajectory   
    //turning = true;

   // printf(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Turning direction: %i\n", random);
    while (get_sensor_reading(sensor) > 110) {
        DriveMotorsRotate(dir, defaultTurnPower);
        //   CPhidgetInterfaceKit_getSensorValue(IFK, ir1Index, &sensorValue);
        //    counts = 1; // reduce duration for subsequent turns
    }
    //Turn(dir, 2,  defaultTurnPower);
    DriveMotorsStop();
    tooClose = false;
    //turning = false;

    // Continue in a straight trajectory
    //DriveMotorsStraight(defaultDirection, defaultPower);
}



bool Robot::turning() {
    return(ServoPos == 1);
}

bool Robot::movingStraight() {
    return(ServoPos == 0);
}


// Code to respond to a detected change in the whisker switches

void Robot::WhiskerChangeResponse() {

    int counts = 9;

    printf("Reverse now please or we'll crash\n");
    printf("Final Hall count = %i\n", HallCount);
    //Reverse(5, defaultTurnPower);    

    printf(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Turning direction: %i\n", activatedWhisker);
    DriveMotorsStraight(0,100);
    sleep(2);
    //Turn(activatedWhisker % 2, counts, defaultTurnPower);
    DriveMotorsStop();
    //Turn(1, 9, defaultTurnPower); // anticlockwise
    DriveMotorsRotate(1,100);
    sleep(2);
    DriveMotorsStraight(0,-100);
    activatedWhisker = 0;

    numOfTurnsToPerform--;
    if (numOfTurnsToPerform == 0) {
	printf("FINALLY STOPPING!!!!");
        DriveMotorsStop();
        exit(0);
    }

}

/*
// update path in terms of lines and angles moved
void Robot::buildPath() {    
    
    lines.push_back(0);
    rotations.push_back(0);
        
    for(int i = 0; i < moveCounter.size(); i++) {
 
        if(moveCounter.at(i) != 0) {
            if (rotations.back() != 0) {lines.push_back(0) ; rotations.push_back(0);}
            lines.back() = lines.back() + moveCounter.at(i);        
        }
        if(rotateCounter.at(i) != 0) {    
            if (lines.back() != 0) {lines.push_back(0) ; rotations.push_back(0);}
            rotations.back() = rotations.back() + rotateCounter.at(i);        
        }        
    }
}*/

/*
// build pose based on path travelled - start pose needs to be given as a parameter
// the point in the path to start building from also needs to be given
Pose Robot::buildPose(Pose startPose, int pathStart) {
    
    int line_dist;
    Pose currentPose(startPose.x, startPose.y, startPose.theta);
            
    // loop through each path item
    for(int n = pathStart; n < lines.size(); n++) {
        
        // if this item is a rotation then update current theta
        if(rotations.at(n) != 0) {
            currentPose.theta = currentPose.theta - rotations.at(n) * 2.62;
        }
        
        // if this item is a movement then update x and y offsets
        if(lines.at(n) != 0) {            
            line_dist = lines.at(n) * 0.43;
            currentPose.x = currentPose.x + line_dist * cos(currentPose.theta);
            currentPose.y = currentPose.y + line_dist * sin(currentPose.theta);           
        }        
    }
            
    return currentPose;
} */

/*
void Robot::outputClicks() {
    
    FILE * pFile;
    
    //Output clicks
    printf("moveCounter.size() = %i\n", moveCounter.size());
    printf("rotateCounter.size() = %i\n", rotateCounter.size());
    pFile = fopen("clicks.txt", "w");
    for (int n = 0; n < moveCounter.size(); n++) {
        fprintf(pFile, "%i  Moving: %i   Rotating:   %i \n", n, moveCounter.at(n), rotateCounter.at(n));
    }
    fclose(pFile);
     
}*/

/*
void Robot::outputPath() {
    
    FILE * pFile;
    
    //Output clicks
    pFile = fopen("path.txt", "w");
    for (int n = 0; n < lines.size(); n++) {
        fprintf(pFile, "%i  Line: %i   Rotation:   %i \n", n, lines.at(n), rotations.at(n));
    }
    fclose(pFile);
  
}*/



