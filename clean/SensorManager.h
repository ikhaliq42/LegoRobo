#ifndef SENSORMANAGER_H_
#define SENSORMANAGER_H_

#include <phidget21.h>

class SensorManager;

class SensorManager {
public:
	SensorManager();
	virtual ~SensorManager();

	enum DigitalSensor{
		LeftOffOn = 0,
		RightOffOn = 1,
		LeftIR = 2,
		RightIR = 3,

	};
	enum AnalogSensor{
		LeftWhisker = 3,
		RightWhisker = 4
	};

	static int AttachHandler(CPhidgetHandle ifk, void* usrptr);
	static int DetachHandler(CPhidgetHandle ifk, void* usrptr);
	static int ErrorHandler(CPhidgetHandle ifk, void* usrptr, int errorCode, const char* unknown);
	static int InputChangeHandler(CPhidgetInterfaceKitHandle ifk, void* usrptr, int index, int state);
	static int SensorChangeHandler(CPhidgetInterfaceKitHandle ifk, void* usrptr, int index, int value);

private:
	CPhidgetInterfaceKitHandle interfaceKit;
};

#endif /* SENSORMANAGER_H_ */
