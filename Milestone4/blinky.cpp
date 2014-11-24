#include <phidget21.h>
#include <stdio.h>

int AttachHandler(CPhidgetHandle IFK, void *userptr)
{
	int serialNo;
	const char *name;

	CPhidget_getDeviceName(IFK, &name);
	CPhidget_getSerialNumber(IFK, &serialNo);

	printf("%s %10d attached!\n", name, serialNo);

	return 0;
}

int DetachHandler(CPhidgetHandle IFK, void *userptr)
{
	int serialNo;
	const char *name;

	CPhidget_getDeviceName (IFK, &name);
	CPhidget_getSerialNumber(IFK, &serialNo);

	printf("%s %10d detached!\n", name, serialNo);

	return 0;
}

int ErrorHandler(CPhidgetHandle IFK, void *userptr, int ErrorCode, const char *unknown) 
{
	printf("Error handled. %d - %s", ErrorCode, unknown);
	return 0;
}

int main(int argc, char** argv) {

    //Declare an InterfaceKit handle
    CPhidgetInterfaceKitHandle ifKit;

    //create the InterfaceKit object
    CPhidgetInterfaceKit_create(&ifKit);

    //Set the handlers to be run when the device is plugged in or opened from software, unplugged or closed from software, or generates an error.
    CPhidget_set_OnAttach_Handler((CPhidgetHandle)ifKit, AttachHandler, NULL);
    CPhidget_set_OnDetach_Handler((CPhidgetHandle)ifKit, DetachHandler, NULL);
    CPhidget_set_OnError_Handler((CPhidgetHandle)ifKit, ErrorHandler, NULL);

    CPhidget_open((CPhidgetHandle)ifKit, -1);



    printf("Press any key...");
    getchar();

    //set LED to on
    CPhidgetInterfaceKit_setOutputState(ifKit, 0, 1);

    printf("Press any key2...\n");
    getchar();

    CPhidgetInterfaceKit_setOutputState(ifKit, 0, 0);

    printf("Press any key to terminate...");
    getchar();

    //Close Phidgets
    CPhidget_close((CPhidgetHandle)ifKit);
    CPhidget_delete((CPhidgetHandle)ifKit);

return 0;
}