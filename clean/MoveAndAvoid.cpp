#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <boost/smart_ptr/shared_ptr.hpp>

#include "SensorManager.h"
#include "MotorManager.h"
#include "Robot.h"

int main(int rgc, char* argv[]){
	std::cout << "Program commences!" << std::endl;

	//SensorManager sensors;
	//MotorManager motors;
	Robot_ptr robot = Robot::getInstance();
	std::cout << "Go (test sensors and motors)" << std::endl;
	sleep(2);

	robot->motors.Go(MotorManager::Forward, 100.0);
	std::cout << "===================================" << std::endl;
	std::cout << "Let's go forward at speed 100 :)" << std::endl;
	std::cout << "===================================" << std::endl;
	sleep(5);

	robot->motors.Go(MotorManager::Backward, 100.0);
	std::cout << "===================================" << std::endl;
	std::cout << "Let's go backward at speed 100 :)" << std::endl;
	std::cout << "===================================" << std::endl;
	sleep(5);

	robot->motors.Go(MotorManager::Left, 100.0);
	std::cout << "===================================" << std::endl;
	std::cout << "Let's go left at speed 100 :)" << std::endl;
	std::cout << "===================================" << std::endl;
	sleep(5);

	robot->motors.Go(MotorManager::Right, 100.0);
	std::cout << "===================================" << std::endl;
	std::cout << "Let's go left at speed 100 :)" << std::endl;
	std::cout << "===================================" << std::endl;
	sleep(5);

	robot->motors.Stop();
	std::cout << "===================================" << std::endl;
	std::cout << "Let's pause the motors :)" << std::endl;
	std::cout << "===================================" << std::endl;

	std::cout << "Program terminates!" << std:: endl;
	return 0;
}
