#include <iostream>
#include <stdio.h>
#include <unistd.h>

#include "SensorManager.h"
#include "MotorManager.h"

int main(int rgc, char* argv[]){
	std::cout << "Program commences!" << std::endl;

	SensorManager sensors;
	MotorManager motors;
	std::cout << "Go (test sensors and motors)" << std::endl;

	sleep(2);
	motors.SetAcceleration(100.0);
	motors.SetSpeedStraight(100.0);
	std::cout << "Let's go forward at speed 100 :)" << std::endl;

	sleep(10);
	motors.SetSpeedStraight(-50.0);
	std::cout << "Let's go backward at speed 50 :)" << std::endl;

	sleep(10);
	motors.SetSpeedStraight(0.0);
	std::cout << "Let's pause the motors :)" << std::endl;

	sleep(10);
	motors.SetSpeedRotate(40.0);
	std::cout << "Let's go rotate at speed 40 :)" << std::endl;

	sleep(10);
	motors.SetSpeedRotate(0.0);
	std::cout << "Let's pause the motors :)" << std::endl;

	sensors.~SensorManager();
	motors.~MotorManager();
	std::cout << "Program terminates!" << std:: endl;
	return 0;
}
