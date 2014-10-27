#include <stdio.h>
#include <phidget21.h>
#include <time.h>
#include <stdbool.h>
#include <stdlib.h>
#include "GeneralHandlers.h"

/////////////////////////////////////////////////////////////////////////
///// RCX Motor setup
///////////////////////////////////////////////////////////////////////

int InputChangeHandlerMoto(CPhidgetMotorControlHandle MC, void *usrptr, int Index, int State);

int VelocityChangeHandler(CPhidgetMotorControlHandle MC, void *usrptr, int Index, double Value);

int CurrentChangeHandler(CPhidgetMotorControlHandle MC, void *usrptr, int Index, double Value);

int display_properties_moto(CPhidgetMotorControlHandle phid);

int InitialiseRCXMotors(CPhidgetMotorControlHandle *motoControl);


