#include <iostream>
#include <stdio.h>
#include <phidget21.h>

#include "SensorManager.h"

SensorManager::SensorManager() {
	// Declare an InterfaceKit handle
	interfaceKit = 0;
	// Create an InterfaceKit object
	CPhidgetInterfaceKit_create(&interfaceKit);

	CPhidget_set_OnAttach_Handler((CPhidgetHandle)interfaceKit, SensorManager::AttachHandler, this);
	CPhidget_set_OnError_Handler((CPhidgetHandle)interfaceKit, SensorManager::ErrorHandler, this);

	CPhidgetInterfaceKit_set_OnInputChange_Handler(interfaceKit, SensorManager::InputChangeHandler, this);
	CPhidgetInterfaceKit_set_OnSensorChange_Handler(interfaceKit, SensorManager::SensorChangeHandler, this);

	CPhidget_open((CPhidgetHandle)interfaceKit, -1);

	std::cout << "Sensors ready!" << std::endl;
}

SensorManager::~SensorManager() {

	CPhidget_set_OnDetach_Handler((CPhidgetHandle)interfaceKit, SensorManager::DetachHandler, this);
	// Close the phidget
	CPhidget_close((CPhidgetHandle)interfaceKit);
	// Delete the created object
	CPhidget_delete((CPhidgetHandle)interfaceKit);

	std::cout << "Sensors stopped!" << std::endl;
}

int SensorManager::AttachHandler(CPhidgetHandle ifk, void* usrptr) {
	SensorManager* sm = (SensorManager*)usrptr;
	std::cout << "Sensor handlers attached" << std::endl;
	return 0;
}

int SensorManager::DetachHandler(CPhidgetHandle ifk, void* usrptr) {
	SensorManager* sm = (SensorManager*)usrptr;
	std::cout << "Sensor handlers detached" << std::endl;
	return 0;
}
int SensorManager::ErrorHandler(CPhidgetHandle ifk, void* usrptr, int errorCode, const char* unknown) {
	SensorManager* sm = (SensorManager*)usrptr;
	printf("Error handled. %d - %s\n", errorCode, unknown);
	return 0;
}

int SensorManager::InputChangeHandler(CPhidgetInterfaceKitHandle ifk, void* usrptr, int index, int state) {
	SensorManager* sm = (SensorManager*)usrptr;
	if(index == SensorManager::LeftOffOn) {
		printf("Left touch sensor: %i\n", state);
	}
	if(index == SensorManager::RightOffOn) {
		printf("Right touch sensor: %i\n", state);
	}
	if(index == SensorManager::LeftWhisker) {
		printf("Left whisker sensor: %i\n", state);
	}
	if(index == SensorManager::RightWhisker) {
		printf("Right whisker sensor: %i\n", state);
	}
	return 0;
}
int SensorManager::SensorChangeHandler(CPhidgetInterfaceKitHandle ifk, void* usrptr, int index, int value) {
	SensorManager* sm = (SensorManager*)usrptr;
	if(index == SensorManager::LeftIR) {
			printf("Left IR sensor: %i\n", value);
	}
	if(index == SensorManager::RightIR) {
			printf("Right IR sensor: %i\n", value);
	}
	return 0;
}
