#include <stdio.h>
#include <phidget21.h>
#include <time.h>
#include <stdbool.h>
#include<stdlib.h>


/////////////////////////////////////////////////////////////////////////
///// General Handlers
/////////////////////////////////////////////////////////////////////////

int AttachHandler(CPhidgetHandle phid, void *userptr);

int DetachHandler(CPhidgetHandle phid, void *userptr);

int ErrorHandler(CPhidgetHandle phid, void *userptr, int ErrorCode, const char *Description);

