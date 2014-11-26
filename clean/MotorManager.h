#ifndef MOTORMANAGER_H_
#define MOTORMANAGER_H_

#include <phidget21.h>

class MotorManager {
public:
	MotorManager();
	virtual ~MotorManager();

	enum Motor{
			LeftMotor = 0,
			RightMotor = 1
		};

	enum Direction{
			Forward = 0,
			Backward = 1,
			Left = 2,
			Right = 3
	};

	static int AttachHandler(CPhidgetHandle handle, void* usrptr);
	static int DetachHandler(CPhidgetHandle hadnle, void* usrptr);
	static int ErrorHandler(CPhidgetHandle handle, void* usrptr, int errorCode, const char* unknown);


	void SetAcceleration(const double accelertion);
	void SetSpeed(const double speed);
	void SetDirection(const Direction direction);
	void Go(const Direction direction, const double speed);
	void Stop();

private:
	CPhidgetMotorControlHandle motorControl;
	CPhidgetAdvancedServoHandle servo;
};

#endif /* MOTORMANAGER_H_ */
