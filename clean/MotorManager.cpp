#include <iostream>
#include <stdio.h>
#include <phidget21.h>

#include "MotorManager.h"

MotorManager::MotorManager() {
	// Declare a motor handle
	motorControl = 0;
	// Create a motor control object
	CPhidgetMotorControl_create (&motorControl);

	CPhidget_set_OnAttach_Handler((CPhidgetHandle)motorControl, MotorManager::AttachHandler, this);
	CPhidget_set_OnError_Handler((CPhidgetHandle)motorControl, MotorManager::ErrorHandler, this);

	CPhidget_open((CPhidgetHandle)motorControl, -1);

	std::cout << "Motors ready!" << std::endl;
}

MotorManager::~MotorManager() {

	CPhidget_set_OnDetach_Handler((CPhidgetHandle)motorControl, MotorManager::DetachHandler, this);
	// Close the phidget
	CPhidget_close((CPhidgetHandle)motorControl);
	// Delete the created object
	CPhidget_delete((CPhidgetHandle)motorControl);

	std::cout << "Motors stopped!" << std::endl;
}

int MotorManager::AttachHandler(CPhidgetHandle mtrCtrl, void* usrptr) {
	MotorManager* sm = (MotorManager*)usrptr;
	std::cout << "Motor handlers attached" << std::endl;
	return 0;
}

int MotorManager::DetachHandler(CPhidgetHandle mtrCtrl, void* usrptr) {
	MotorManager* sm = (MotorManager*)usrptr;
	std::cout << "Motor handlers detached" << std::endl;
	return 0;
}
int MotorManager::ErrorHandler(CPhidgetHandle mtrCtrl, void* usrptr, int errorCode, const char* unknown) {
	MotorManager* sm = (MotorManager*)usrptr;
	printf("Error handled. %d - %s\n", errorCode, unknown);
	return 0;
}

void MotorManager::SetAcceleration(const double acceleration) {
	CPhidgetMotorControl_setAcceleration(motorControl, MotorManager::LeftMotor, acceleration);
	CPhidgetMotorControl_setAcceleration(motorControl, MotorManager::RightMotor, acceleration);
}

void MotorManager::SetSpeed(const double speed) {
	CPhidgetMotorControl_setVelocity(motorControl, MotorManager::LeftMotor, speed);
	CPhidgetMotorControl_setVelocity(motorControl, MotorManager::RightMotor, -speed);
}
