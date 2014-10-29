#include <iostream>
#include <stdio.h>
#include <unistd.h>

#include "SensorManager.h"

int main(int rgc, char* argv[]){
	std::cout << "Program commences!" << std::endl;

	SensorManager sensors;
	std::cout << "Go (test sensors)" << std::endl;
	sleep(10);
	sensors.~SensorManager();
	std::cout << "Program terminates!" << std:: endl;
	return 0;
}
