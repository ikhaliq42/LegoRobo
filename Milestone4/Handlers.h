/* 
 * File:   Handlers.h
 * Author: s1461613
 *
 * Created on 09 October 2014, 11:39
 */

#ifndef GENERALHANDLERS_H
#define	GENERALHANDLERS_H

#include <phidget21.h>

class Handlers {
    
public:

static int AttachHandler(CPhidgetHandle phid, void *userptr);
static int DetachHandler(CPhidgetHandle phid, void *userptr);
static int ErrorHandler(CPhidgetHandle phid, void *userptr, int ErrorCode, const char *Description);  
static int InputChangeHandler(CPhidgetInterfaceKitHandle IFK, void *usrptr, int Index, int State); 
static int SensorChangeHandler(CPhidgetInterfaceKitHandle IFK, void *usrptr, int Index, int Value);
static int InputChangeHandlerMoto(CPhidgetMotorControlHandle MC, void *usrptr, int Index, int State);
static int VelocityChangeHandler(CPhidgetMotorControlHandle MC, void *usrptr, int Index, double Value);
static int CurrentChangeHandler(CPhidgetMotorControlHandle MC, void *usrptr, int Index, double Value);
static int PositionChangeHandler(CPhidgetAdvancedServoHandle ADVSERVO, void *usrptr, int Index, double Value); 

Handlers();
Handlers(const Handlers& orig);
virtual ~Handlers();

    


};

#endif	/* GENERALHANDLERS_H */

