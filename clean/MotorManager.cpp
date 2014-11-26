#include <iostream>
#include <stdio.h>
#include <phidget21.h>

#include "MotorManager.h"

MotorManager::MotorManager() {
	// Declare a motor handle
	motorControl = 0;
	// Declare a Servo handle
	servo = 0;
	// Create a motor control object
	CPhidgetMotorControl_create (&motorControl);
	// Create the servo object
	CPhidgetAdvancedServo_create(&servo);

	CPhidget_set_OnAttach_Handler((CPhidgetHandle)motorControl, MotorManager::AttachHandler, this);
	CPhidget_set_OnAttach_Handler((CPhidgetHandle)servo, MotorManager::AttachHandler, this);
	CPhidget_set_OnError_Handler((CPhidgetHandle)motorControl, MotorManager::ErrorHandler, this);
	CPhidget_set_OnError_Handler((CPhidgetHandle)servo, MotorManager::ErrorHandler, this);

	// Open Motor control for device connections
	CPhidget_open((CPhidgetHandle)motorControl, -1);
	// Open the servo for device connections
	CPhidget_open((CPhidgetHandle)servo, -1);

	std::cout << "Motors ready!" << std::endl;
}

MotorManager::~MotorManager() {

	Stop();

	CPhidget_set_OnDetach_Handler((CPhidgetHandle)motorControl, MotorManager::DetachHandler, this);
	// Close the phidget
	CPhidget_close((CPhidgetHandle)motorControl);
	// Delete the created object
	CPhidget_delete((CPhidgetHandle)motorControl);
	// Close the servo
	CPhidget_set_OnDetach_Handler((CPhidgetHandle)servo, MotorManager::DetachHandler, this);
	CPhidget_close((CPhidgetHandle)servo);
	// Delete the created servo object
	CPhidget_delete((CPhidgetHandle)servo);

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
	CPhidgetMotorControl_setVelocity(motorControl, MotorManager::RightMotor, speed);
}

void MotorManager::SetDirection(const Direction direction) {
	if(direction == MotorManager::Forward || direction == MotorManager::Backward) {
		CPhidgetAdvancedServo_setPosition (servo, 0, 180.00);
		CPhidgetAdvancedServo_setEngaged(servo, 0, 1);
	} else if(direction == MotorManager::Left || direction == MotorManager::Right){
		CPhidgetAdvancedServo_setPosition (servo, 0, 130.00);
		CPhidgetAdvancedServo_setEngaged(servo, 0, 1);
	} else {
		printf("Not a legal Direction value!");
	}
}

void MotorManager::Go(const Direction direction, const double speed) {
	SetDirection(direction);
	if(direction == MotorManager::Forward) {
		SetSpeed(-speed);
	} else if (direction == MotorManager::Backward) {
		SetSpeed(speed);
	} else if (direction == MotorManager::Left) {
		SetSpeed(speed);
	} else if (direction == MotorManager::Right) {
		SetSpeed(-speed);
	}
}

void MotorManager::Stop() {
	SetSpeed(0.0);
}
