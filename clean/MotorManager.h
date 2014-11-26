#ifndef MOTORMANAGER_H_
#define MOTORMANAGER_H_

#include <phidget21.h>

class MotorManager {
public:
	MotorManager();
	virtual ~MotorManager();

	enum Motor{
			LeftMotor = 0,
			RightMotor = 2
		};

	enum Direction{
			Forward = 0,
			Backward = 1
	};

	static int AttachHandler(CPhidgetHandle mtrCtrl, void* usrptr);
	static int DetachHandler(CPhidgetHandle mtrCtrl, void* usrptr);
	static int ErrorHandler(CPhidgetHandle mtrCtrl, void* usrptr, int errorCode, const char* unknown);


	void SetAcceleration(const double accelertion);
	void SetSpeed(const double speed);
	void SetDirection(const Direction direction);
	void SetSpeedRotate(const double speed);

private:
	CPhidgetMotorControlHandle motorControl;
	CPhidgetServoHandle servo;
};

#endif /* MOTORMANAGER_H_ */
