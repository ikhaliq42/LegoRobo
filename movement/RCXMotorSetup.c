#include <stdio.h>
#include <phidget21.h>
#include <time.h>
#include <stdbool.h>
#include <stdlib.h>
#include "GeneralHandlers.h"
#include "RCXMotorSetup.h"
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

int InitialiseRCXMotors(CPhidgetMotorControlHandle *motoControl) {
    int result;
    const char *err;

    //Declare a motor control handle
    //CPhidgetMotorControlHandle motoControl = 0;

    //create the motor control objct
    CPhidgetMotorControl_create(motoControl);

    //Set the handlers to be run when the device is plugged in or opened from software, 
    //unplugged or closed from software, or generates an error.
    CPhidget_set_OnAttach_Handler((CPhidgetHandle) *motoControl, AttachHandler, NULL);
    CPhidget_set_OnDetach_Handler((CPhidgetHandle) *motoControl, DetachHandler, NULL);
    CPhidget_set_OnError_Handler((CPhidgetHandle) *motoControl, ErrorHandler, NULL);

    //Registers a callback that will run if an input changes.
    //Requires the handle for the Phidget, the function that will be called, 
    //and a arbitrary pointer that will be supplied to the callback function (may be NULL).
    CPhidgetMotorControl_set_OnInputChange_Handler(*motoControl, InputChangeHandlerMoto, NULL);

    //Registers a callback that will run if a motor changes.
    //Requires the handle for the Phidget, the function that will be called, 
    //and a arbitrary pointer that will be supplied to the callback function (may be NULL).
    CPhidgetMotorControl_set_OnVelocityChange_Handler(*motoControl, VelocityChangeHandler, NULL);

    //Registers a callback that will run if the current draw changes.
    //Requires the handle for the Phidget, the function that will be called, 
    //and a arbitrary pointer that will be supplied to the callback function (may be NULL).
    CPhidgetMotorControl_set_OnCurrentChange_Handler(*motoControl, CurrentChangeHandler, NULL);

    //open the motor control for device connections
    CPhidget_open((CPhidgetHandle) *motoControl, -1);

    //get the program to wait for a motor control device to be attached
    printf("Waiting for MotorControl to be attached....\n");
    if ((result = CPhidget_waitForAttachment((CPhidgetHandle) *motoControl, 10000))) {
        CPhidget_getErrorDescription(result, &err);
        printf("Problem waiting for attachment: %s\n", err);
        return 0;
    }
    return 0;
}

