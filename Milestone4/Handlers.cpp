/* 
 * File:   Handlers.cpp
 * Author: s1461613
 * 
 * Created on 09 October 2014, 11:39
 */

#include "Handlers.h"
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include "Globals.h"
#include "Robot.h"

int hallEffectDir = 0;
int hall0state = 0;
int hall1state = 0;

bool IR1tooClose = false;
bool IR1tooClose2 = false;
bool IR2tooClose = false;
bool IR2tooClose2 = false;

/////////////////////////////////////////////////////////////////////////
///// General Handlers
/////////////////////////////////////////////////////////////////////////

int Handlers::AttachHandler(CPhidgetHandle phid, void *userptr) {
    int serialNo;
    const char *name;

    CPhidget_getDeviceName(phid, &name);
    CPhidget_getSerialNumber(phid, &serialNo);
    printf("%s %10d attached!\n", name, serialNo);

    return 0;
}

int Handlers::DetachHandler(CPhidgetHandle phid, void *userptr) {
    int serialNo;
    const char *name;

    CPhidget_getDeviceName(phid, &name);
    CPhidget_getSerialNumber(phid, &serialNo);
    printf("%s %10d detached!\n", name, serialNo);

    return 0;
}

int Handlers::ErrorHandler(CPhidgetHandle phid, void *userptr, int ErrorCode, const char *Description) {
    printf("Error handled. %d - %s\n", ErrorCode, Description);
    return 0;
}


/////////////////////////////////////////////////////////////////////////
///// Interface Kit Setup - For IR Sensor, Whiskers and On / Off Switch
/////////////////////////////////////////////////////////////////////////

int Handlers::InputChangeHandler(CPhidgetInterfaceKitHandle IFK, void *usrptr, int Index, int State) {

    // show input readings
    //printf("Digital Input: %d > State: %d\n", Index, State);

    Robot *robo = (Robot*) usrptr;

    if (Index == 6 && (robo->movingStraight() == true)) {

        hall0state = State;
        if (hallEffectDir == 0) {
            if (hall0state == hall1state) {
                robo->HallCount--;
                //robo->moveCounter.push_back(-1);
                //robo->rotateCounter.push_back(0);
            } else {
                robo->HallCount = robo->HallCount + 3;
                hallEffectDir = 1;
                //robo->moveCounter.push_back(3);                
                //robo->rotateCounter.push_back(0);
            }
        } else if (hallEffectDir == 1) {
            if (hall0state == hall1state) {                
                robo->HallCount = robo->HallCount - 3;
                hallEffectDir = 0;
                //robo->moveCounter.push_back(-3);
                //robo->rotateCounter.push_back(0);
            } else {
                robo->HallCount++;                
                //robo->moveCounter.push_back(1);
                //robo->rotateCounter.push_back(0);                
            }
        }
    }
        if (Index == 6 && (robo->movingStraight() == false)) {

        hall0state = State;
        if (hallEffectDir == 0) {
            if (hall0state == hall1state) {
                robo->HallCount--;
                //robo->moveCounter.push_back(0);
                //robo->rotateCounter.push_back(-1);
            } else {
                robo->HallCount = robo->HallCount + 3;
                hallEffectDir = 1;
                //robo->moveCounter.push_back(0);                
                //robo->rotateCounter.push_back(3);
            }
        } else if (hallEffectDir == 1) {
            if (hall0state == hall1state) {                
                robo->HallCount = robo->HallCount - 3;
                hallEffectDir = 0;
                //robo->moveCounter.push_back(0);
                //robo->rotateCounter.push_back(-3);
            } else {
                robo->HallCount++;                
                robo->moveCounter.push_back(0);
                robo->rotateCounter.push_back(1);                
            }
        }

    }

    if (Index == 7 && (robo->movingStraight() == true)) {
        hall1state = State;
        if (hallEffectDir == 1) {
            if (hall0state == hall1state) {
                robo->HallCount++;                
                //robo->moveCounter.push_back(1);
                //robo->rotateCounter.push_back(0);
            } else {
                robo->HallCount = robo-> HallCount - 3;
                hallEffectDir = 0;
                //robo->moveCounter.push_back(-3);
                //robo->rotateCounter.push_back(0);
            }
        } else if (hallEffectDir == 0) {
            if (hall0state == hall1state) {
                robo->HallCount = robo->HallCount + 3;
                hallEffectDir = 1;
                //robo->moveCounter.push_back(3);
                //robo->rotateCounter.push_back(0);
            } else {
                robo->HallCount--;
                //robo->moveCounter.push_back(-1);
                //robo->rotateCounter.push_back(0);
            }
        }
    }

    if (Index == 7 && (robo->movingStraight() == false)) {
        hall1state = State;
        if (hallEffectDir == 1) {
            if (hall0state == hall1state) {
                robo->HallCount++;
                //robo->moveCounter.push_back(0);
                //robo->rotateCounter.push_back(1);
            } else {
                robo->HallCount = robo-> HallCount - 3;
                hallEffectDir = 0;
                //robo->moveCounter.push_back(0);
                //robo->rotateCounter.push_back(-3);
            }
        } else if (hallEffectDir == 0) {
            if (hall0state == hall1state) {
                robo->HallCount = robo->HallCount + 3;
                hallEffectDir = 1;
                //robo->moveCounter.push_back(0);
                //robo->rotateCounter.push_back(3);
            } else {
                robo->HallCount--;
                //robo->moveCounter.push_back(0);
                //robo->rotateCounter.push_back(-1);
            }
        }
    }


    // increase IR threshold temporarily in order to prevent another event from triggering
    //CPhidgetInterfaceKit_setSensorChangeTrigger(IFK, ir1Index, 10000);

    // choose correct response based on input source
    if (Index == robo->on_off_Index1) {
        printf("On/Off switch 1 pressed...\n");
        if (robo->startFlag == false && robo->exitFlag == false) robo->startFlag = true;
    }

    if (Index == robo->on_off_Index2) {
        printf("On/Off switch 2 pressed...\n");
        if (robo->startFlag == true && robo->exitFlag == false) robo->exitFlag = true;
    }

    if (Index == robo->whisker1Index && State == 1) {

        printf("whisker1 switch pressed...\n");
        robo->activatedWhisker = 1;
	//robo->DriveMotorsStraight(0,0);
        robo->WhiskerChangeResponse();            
    }

    if (Index == robo->whisker2Index && State == 1) {
        printf("whisker2 switch pressed...");
        robo->activatedWhisker = 2;
        robo->WhiskerChangeResponse();
    }

    // reset ir threshold to default value
    //CPhidgetInterfaceKit_setSensorChangeTrigger(IFK, ir1Index, irThreshold);

    return 0;
}

//callback that will run if the sensor value changes by more than the OnSensorChange trigger.
//Index - Index of the sensor that generated the event, Value - the sensor read value

int Handlers::SensorChangeHandler(CPhidgetInterfaceKitHandle IFK, void *usrptr, int Index, int Value) {
    // show sensor readings
    //printf("Sensor: %d > Value: %d\n", Index, Value);

    // increase IR threshold temporarily in order to prevent another event from triggering
    //int currentTrigger;
    //CPhidgetInterfaceKit_getSensorChangeTrigger(IFK, ir1Index, &currentTrigger);
    //CPhidgetInterfaceKit_setSensorChangeTrigger(IFK, ir1Index, 10000);

    Robot *robo = (Robot*) usrptr;
    
        if ((Index == ir1Index) && (Value > 1 && Value < 600)) {
        //if (Index == ir1Index) {
        //if (robo->atSafeDistance(Index, Value, safeDistance)) {
            if (Value < 230){
            IR1tooClose = false;
            //printf("Comfortable distance...\n");
        } else {
            IR1tooClose = true;
            //printf("Getting a tad too close...\n");
        }
        if (robo->atSafeDistance(Index, Value, safeDistance2)) {
            IR1tooClose2 = false;
            //printf("Comfortable distance...\n");
        } else {
            IR1tooClose2 = true;
            //printf("Getting a tad too close...\n");
        }
        robo->tooClose =  IR2tooClose || IR1tooClose;
        robo->tooClose2 = IR2tooClose2 || IR1tooClose2;
        robo->wallskimLeft = IR1tooClose && !IR2tooClose;
        robo->wallskimRight = !IR1tooClose && IR2tooClose;
    }

    // choose correct response based on sensor reading
    if ((Index == ir2Index) && (Value > 1 && Value < 600)) {
        //if (Index == ir1Index) {
        //if (robo->atSafeDistance(Index, Value, safeDistance)) {
                        if (Value < 230){
            IR2tooClose = false;
            //printf("Comfortable distance...\n");
        } else {
            IR2tooClose = true;
            //printf("Getting a tad too close...\n");
        }
        if (robo->atSafeDistance(Index, Value, safeDistance2)) {
            IR2tooClose2 = false;
            //printf("Comfortable distance...\n");
        } else {
            IR2tooClose2 = true;
            //printf("Getting a tad too close...\n");
        }
        robo->tooClose =  IR2tooClose || IR1tooClose;
        robo->tooClose2 = IR2tooClose2 || IR1tooClose2;
        robo->wallskimLeft = IR1tooClose && !IR2tooClose;
        robo->wallskimRight = !IR1tooClose && IR2tooClose;
        
    }
    
    

    // reset ir threshold to default value
    //CPhidgetInterfaceKit_setSensorChangeTrigger(IFK, ir1Index, currentTrigger);

    return 0;
}

/////////////////////////////////////////////////////////////////////////
///// RCX Motor setup
///////////////////////////////////////////////////////////////////////

int Handlers::InputChangeHandlerMoto(CPhidgetMotorControlHandle MC, void *usrptr, int Index, int State) {
    printf("Input %d > State: %d\n", Index, State);
    return 0;
}

int Handlers::VelocityChangeHandler(CPhidgetMotorControlHandle MC, void *usrptr, int Index, double Value) {
    //printf("Motor %d > Current Speed: %f\n", Index, Value);
    return 0;
}

int Handlers::CurrentChangeHandler(CPhidgetMotorControlHandle MC, void *usrptr, int Index, double Value) {
    printf("Motor: %d > Current Draw: %f\n", Index, Value);
    return 0;
}

/////////////////////////////////////////////////////////////////////////
///// Servo motor setup
///////////////////////////////////////////////////////////////////////

int Handlers::PositionChangeHandler(CPhidgetAdvancedServoHandle ADVSERVO, void *usrptr, int Index, double Value) {
    //	printf("Motor: %d > Current Position: %f\n", Index, Value);
    return 0;
}

Handlers::Handlers() {
}

Handlers::Handlers(const Handlers& orig) {
}

Handlers::~Handlers() {
}

