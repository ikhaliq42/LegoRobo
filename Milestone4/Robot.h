/* 
 * File:   Robot.h
 * Author: s1461613
 *
 * Created on 09 October 2014, 15:51
 */

#ifndef ROBOT_H
#define	ROBOT_H
#include <vector>
#include "Pose.h"
#include <boost/smart_ptr/shared_ptr.hpp>

class Robot;

typedef boost::shared_ptr<Robot> Robot_ptr;

class Robot {
public:

    
    // Get instance of this singleton class
    static Robot_ptr getInstance();
    
    // Phidget Handles
    CPhidgetInterfaceKitHandle IFK;
    CPhidgetMotorControlHandle motoControl;
    CPhidgetAdvancedServoHandle servo;    

    // parameters
    int on_off_Index1;
    int on_off_Index2;
    int whisker1Index;
    int whisker2Index;
    bool exitFlag;
    bool startFlag;
    int activatedWhisker;
    int IanVal;
    int HallCount;
    int tooClose;
    int tooClose2;
    bool wallskimLeft;
    bool wallskimRight;
    bool turningLeft;
    bool turningRight;
    bool movingForwards;
    bool movingBackwards;
    bool stopped;
    int ServoPos; // 0 = 180 degrees, 1 = 130 degrees.
    int x;
    int y;
    double theta;
    std::vector<Pose> poses;
    int xa[1000];
    int ya[1000];
    
    // for vision
    bool objectFound;
    int midP;

    int numOfTurnsToPerform;
    
    // Other values - these may have to be relocated to a more appropriate class later
    std::vector<int> moveCounter;
    std::vector<int> rotateCounter;
    std::vector<int> lines;
    std::vector<int> rotations;
    
    Robot();
    virtual ~Robot();

    /////////////////////////////////////////////////////////////////////////
    ///// RCX Motor setup
    ///////////////////////////////////////////////////////////////////////
    int InitialiseRCXMotors();
    int InitialiseServoMotors();
    int InitialiseMotors();
    int ShutDownMotors();
    int SetServo(int position);

    /////////////////////////////////////////////////////////////////////////
    ///// Drive functions
    ///////////////////////////////////////////////////////////////////////
    int DriveMotorsStraight(int dir, int power);
    int DriveMotorsStop();
    int DriveMotorsRotate(int dir, int power);    
    int display_properties_moto(CPhidgetMotorControlHandle phid) ;    

    /////////////////////////////////////////////////////////////////////////
    ///// Advanced Drive functions
    ///////////////////////////////////////////////////////////////////////

    //rotate for set number of hall counts (clicks) at power pwr,
    // in direction dir (0 = anticlockwise, 1 = clockwise)
    void Turn(int dir, int clicks, int pwr);

    // move for set number of hall counts (clicks) at power pwr,
    // in direction dir (0 = forward, 1 = backwards)   
    void Move(int dir, int clicks, int pwr);
    
    //Reverse for set number of hall counts (clicks) at power pw
    void Reverse(int sec, int pwr);

    /////////////////////////////////////////////////////////////////////////
    ///// Interface Kit Setup - For IR Sensor, Whiskers and On / Off Switch
    /////////////////////////////////////////////////////////////////////////

    int display_properties_ifkit(CPhidgetInterfaceKitHandle phid);
    int initialiseInterfacekit();
    int ShutDownInterfacekit();
    bool atSafeDistance(int index, int sensorValue, int safeDist);
    bool turning();
    bool movingStraight();

    /////////////////////////////////////////////////////////////////////////
    ///// Interface kit read functions
    /////////////////////////////////////////////////////////////////////////
    
    int get_sensor_reading(int irIndex);
    int get_averaged_sensor_reading(int irIndex, int num_readings); 
    
    /////////////////////////////////////////////////////////////////////////
    ///// Responses to obstacles and user inputs
    /////////////////////////////////////////////////////////////////////////

    // Code to respond to a detected change in the IR sensor 
    void tooCloseResponse(int dir);

    // Code to respond to a detected change in the whisker switches
    void WhiskerChangeResponse();

    // update path in terms of lines and angles moved
    //void buildPath();
    
    // build pose based on path travelled - start pose needs to be given as a parameter
    // the point in the path to start building from also needs to be given
    Pose buildPose(Pose startPose, int pathStart); 
    //void outputClicks();
    //void outputPath();

private:

    //Private static instance (singleton)
    static Robot_ptr _instance;

};

#endif	/* ROBOT_H */

