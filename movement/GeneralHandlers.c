#include <stdio.h>
#include <phidget21.h>
#include <time.h>
#include <stdbool.h>
#include <stdlib.h>
#include "GeneralHandlers.h"

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
