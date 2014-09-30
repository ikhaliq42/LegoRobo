
#include <stdio.h>
#include <phidget21.h>

class InterfaceControl : public Robot {
	
	CPhidgetInterfaceKitHandle IFK;
	int irIndex = -1, whisker1Index = -1, whisker2Index = -1, on_off_Index = -1;	 	
        
private:

	/////////////////////////////////////////////////////////////////////////
	///// Interface Kit Setup - For IR Sensor, Whiskers and On / Off Switch
	/////////////////////////////////////////////////////////////////////////
	
	//callback that will run if an input changes.
	//Index - Index of the input that generated the event, State - boolean (0 or 1) representing the input state (on or off)
	int InputChangeHandler(CPhidgetInterfaceKitHandle IFK, void *usrptr, int Index, int State)
	{
		Robot thisRobot = (Robot)this;
		
		// show input readings
		printf("Digital Input: %d > State: %d\n", Index, State);

		// run handler in main robot object
		thisRobot.InputChangeHandler(Index, State);
		 
		return 0;
	}

	//callback that will run if the sensor value changes by more than the OnSensorChange trigger.
	//Index - Index of the sensor that generated the event, Value - the sensor read value
	int SensorChangeHandler(CPhidgetInterfaceKitHandle IFK, void *usrptr, int Index, int Value)
	{
		Robot thisRobot = (Robot)this;

		// show sensor readings
		printf("Sensor: %d > Value: %d\n", Index, Value);

		// run handler in main robot object
		thisRobot.InputChangeHandler(Index, Value);  
		 
		return 0;
	}

public:

	/////////////////////////////////////////////////////////////////////////
	///// Interface Kit User Functions
	/////////////////////////////////////////////////////////////////////////
	
	// set Whisker 1 index
	void setWhisker1Index(int index) {whisker1Index = index;}

	// get Whisker 1 index
	int getWhisker1Index() {return whisker1Index;}

	// set Whisker 2 index
	void setWhisker2Index(int index) {whisker2Index = index;}

	// get Whisker 2 index
	int getWhisker2Index() {return whisker2Index;}

	// set On/Off index
	void setOnOFFIndex(int index) {on_off_Index = index;}

	// get On/Off index
	int getOnOFFIndex() {return on_off_Index;}

	// set IR sensor index
	void setIRIndex(int index) {irIndex = index;}

	// get IR sensor index
	int getIRIndex(int index) {return irIndex;}	

	// set the IR event trigger threshold
	void setIRTrigger(int threshold)
	{
		CPhidgetInterfaceKit_setSensorChangeTrigger(IFK, irIndex, threshold);  
	}

	// get the IR event trigger threshold
	int getIRTrigger()
	{
		int threshold;
		CPhidgetInterfaceKit_getSensorChangeTrigger(IFK, irIndex, &threshold);  
		return threshold;
	}

	// distance conversion function for IR sensor
	double distanceConversion(int sensorValue)
	{
		return 4800/(sensorValue - 20);
	}

	// get distance based on sensor reading
	double getDistance()
	{
		return distanceConversion(getIRSensorReading());				
	}

	// get sensor reading
	int getIRSensorReading()
	{
		int sensorValue;
	
		// get IR sensor reading
		CPhidgetInterfaceKit_getSensorValue (IFK, irIndex, &sensorValue);

		return sensorValue;	
	}

	int initialiseInterfacekit()
	{
		int result;
		const char *err;

		//create the InterfaceKit object
		CPhidgetInterfaceKit_create(&IFK);

		//Set the handlers to be run when the device is plugged in or opened from software, unplugged or closed from software, or generates an error.
		CPhidget_set_OnAttach_Handler((CPhidgetHandle)IFK, AttachHandler, this);
		CPhidget_set_OnDetach_Handler((CPhidgetHandle)IFK, DetachHandler, this);
		CPhidget_set_OnError_Handler((CPhidgetHandle)IFK, ErrorHandler, this);

		//Registers a callback that will run if an input changes.
		//Requires the handle for the Phidget, the function that will be called, 
		//and an arbitrary pointer that will be supplied to the callback function (may be NULL).
		CPhidgetInterfaceKit_set_OnInputChange_Handler (IFK, InputChangeHandler, this);

		//Registers a callback that will run if the sensor value changes by more than the OnSensorChange trig-ger.
		//Requires the handle for the IntefaceKit, the function that will be called, 
		//and an arbitrary pointer that will be supplied to the callback function (may be NULL).
		CPhidgetInterfaceKit_set_OnSensorChange_Handler (IFK, SensorChangeHandler, this);

		//open the interfacekit for device connections
		CPhidget_open((CPhidgetHandle)IFK, -1);

		//get the program to wait for an interface kit device to be attached
		printf("Waiting for interface kit to be attached....");
		if((result = CPhidget_waitForAttachment((CPhidgetHandle)IFK, 10000)))
		{
			CPhidget_getErrorDescription(result, &err);
			printf("Problem waiting for attachment: %s\n", err);
			return 0;
		}

		//Display the properties of the attached interface kit device
		display_properties(IFK);	

		return 0;
	}

	int ShutDownInterfacekit()
	{
		//subroutine to close the interface kit phidget and delete the objects we created
		printf("Closing...\n");
		CPhidget_close((CPhidgetHandle)IFK);
		CPhidget_delete((CPhidgetHandle)IFK);

		return 0;
	}
        
        //Display the properties of the attached phidget to the screen.  We will be displaying the name, serial number and version of the attached device.
        //Will also display the number of inputs, outputs, and analog inputs on the interface kit as well as the state of the ratiometric flag
        //and the current analog sensor sensitivity.
        int display_properties(CPhidgetInterfaceKitHandle phid)
        {
                int serialNo, version, numInputs, numOutputs, numSensors, triggerVal, ratiometric, i;
                const char* ptr;

                CPhidget_getDeviceType((CPhidgetHandle)phid, &ptr);
                CPhidget_getSerialNumber((CPhidgetHandle)phid, &serialNo);
                CPhidget_getDeviceVersion((CPhidgetHandle)phid, &version);

                CPhidgetInterfaceKit_getInputCount(phid, &numInputs);
                CPhidgetInterfaceKit_getOutputCount(phid, &numOutputs);
                CPhidgetInterfaceKit_getSensorCount(phid, &numSensors);
                CPhidgetInterfaceKit_getRatiometric(phid, &ratiometric);

                printf("%s\n", ptr);
                printf("Serial Number: %10d\nVersion: %8d\n", serialNo, version);
                printf("# Digital Inputs: %d\n# Digital Outputs: %d\n", numInputs, numOutputs);
                printf("# Sensors: %d\n", numSensors);
                printf("Ratiometric: %d\n", ratiometric);

                for(i = 0; i < numSensors; i++)
                {
                        CPhidgetInterfaceKit_getSensorChangeTrigger (phid, i, &triggerVal);

                        printf("Sensor#: %d > Sensitivity Trigger: %d\n", i, triggerVal);
                }

                return 0;
        }
        
};
