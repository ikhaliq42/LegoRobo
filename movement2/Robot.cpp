// Main Robot class which encapsulates the different functions

class Robot {

	// main control objects
	DriveControl driveControl;
	InterfaceControl interfaceControl;

	// set other parameters
	int safeDistance = 10;
	int defaultDirection = 0;
	int defaultPower = 100;
	int irThreshold = 100;

	//indices for attachments - don't know these yet so these are just guesses!
	int on_off_Index = 0;
	int whisker1Index = 1;
	int whisker2Index = 2;
	int irIndex = 3;	
        
public:
    
	/////////////////////////////////////////////////////////////////////////
	///// General Handlers
	/////////////////////////////////////////////////////////////////////////
            
	int AttachHandler(CPhidgetHandle IFK, void *userptr)
	{
		int serialNo;
		const char *name;

		CPhidget_getDeviceName (IFK, &name);
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

	int ErrorHandler(CPhidgetHandle IFK, void *userptr, int ErrorCode, const char *Description)
	{
		printf("Error handled. %d - %s\n", ErrorCode, Description);
		return 0;
	}    
    
	int InputChangeHandler(int Index, int State)
	{		
		// show input readings
		printf("Digital Input: %d > State: %d\n", Index, State);

		// increase IR threshold temporarily in order to prevent another event from triggering
		InterfaceControl.setIRTrigger(10000); 

		// choose correct response based on input source
		switch (Index)
		{
			case on_off_Index:
			On_Off_SwitchChangeResponse(State);
			break;

                        case whisker1Index:
			WhiskerChangeResponse(State);
			break;
		
                        case whisker2Index:
			WhiskerChangeResponse(State);		
			break;
		}

		// reset ir threshold to default value
		InterfaceControl.setIRTrigger(irThreshold);  
		 
		return 0;
	}

	//Index - Index of the sensor that generated the event, Value - the sensor read value
	int SensorChangeHandler(int Index, int Value)
	{
		// show sensor readings
		printf("Sensor: %d > Value: %d\n", Index, Value);

		// increase IR threshold temporarily in order to prevent another event from triggering
		int currentTrigger;  
                interfaceControl.setIRTrigger(10000);

		// choose correct response based on sensor reading
		switch (Index)
		{		
			case irIndex:
			IR_ChangeResponse(Value);
			break;
		}

		// reset ir threshold to default value
		interfaceControl.setIRTrigger(irThreshold);  
		 
		return 0;
	}

	// Code to respond to a detected change in the IR sensor 
	void IR_ChangeResponse(int value)
	{
		int dur = 3;
		int sensorValue;
	
		// get IR sensor reading
		sensorValue = interfaceControl.getIRSensorReading();

		// keep turning until obstacle is out of trajectory
		// initial turn is for three seconds, subsequent turns are for 1 second
		while(!atSafeDistance(sensorValue, safeDistance))
		{
			driveControl.timedTurn(dur, defaultPower);
			sensorValue = interfaceControl.getIRSensorReading();
			dur = 1; // reduce duration for subsequent turns
		}	
	
		// Continue in a straight trajectory
		driveControl.DriveMotorsStraight(defaultDirection, defaultPower);	
	}

	// Code to respond to a detected change in the on/off switch - this will stop and start the robot
	void On_Off_SwitchChangeResponse(int value)
	{
		if (value % 2 == 0)
                        {driveControl.DriveMotorsStop();} 
                else 
                        {driveControl.DriveMotorsStraight(defaultDirection, defaultPower);}
	}

	// Code to respond to a detected change in the whisker switches
	void WhiskerChangeResponse(CPhidgetInterfaceKitHandle IFK, int index, int value)
	{	
		// keep reversing until the whisker reading is zero
		while (value == 1) 
		{
			//reverse robot
			driveControl.timedReverse(1, 30);
			//read new sensor value 
			CPhidgetInterfaceKit_getSensorValue(CPhidgetInterfaceKitHandle IFK, index, *value);	  	 			
		}

		// next step is the same as the IR change response
		IR_ChangeResponse(IFK, index, value);
	}

	void initialiseInterfaceControl()
	{	
		// create interface control object
		interfaceControl = new InterfaceControl;		
		
		// initialie interface kit
		interfaceControl.initialiseInterfacekit();

		// set indices for attachments
		interfaceControl.setOnOFFIndex = on_off_Index;
		interfaceControl.setWhisker1Index = whisker1Index;
		interfaceControl.setWhisker2Index = whisker2Index;
		interfaceControl.setIRIndex = irIndex;

		//Change the sensitivity trigger of the sensors
		printf("Modifying sensor sensitivity trigger for IR sensor to: ", irThreshold);
		interfaceControl.setIRTrigger(irThreshold);

	}

	void initialiseDriveControl()
	{
		driveControl = new DriveControl();
		driveControl.InitialiseMotors();
	}
	
	// function to check if at the safe distance safeDist, or greater based on IR sensor reading
	bool atSafeDistance(int sensorValue, int safeDist)
	{
		int dist;

		dist = interfaceControl.distanceConversion(sensorValue);
		if (dist >= safeDist) return true; else return false;
	}

};
