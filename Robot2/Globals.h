/* 
 * File:   Globals.h
 * Author: s1461613
 *
 * Created on 09 October 2014, 18:28
 */

#ifndef GLOBALS_H
#define	GLOBALS_H
#include <vector>

using namespace std;

extern int safeDistance;
extern int safeDistance2;
extern int defaultDirection;
extern int defaultPower;
extern int defaultTurnPower;
extern int defaultReversePower;
//extern int ServoPos; // 0 = 180 degrees, 1 = 130 degrees.
extern int ir1Index;
extern int ir2Index;
extern int sonarIndex;
extern int irThreshold;

class Globals {
public:
    Globals();
    Globals(const Globals& orig);
    virtual ~Globals();
    
    // distance conversion function for IR sensor
    static double distanceConversion(int sensorValue);
    
    // distance conversion function for sonar sensor
    static double sonarDistanceConversion(int sensorValue); 
    
    // output sensor readings
    static void outputSensorReadings(vector< vector<int> > readings, vector<int> counts);
    
    // output distance estimates from sensor readings
    static void outputDistances(vector< vector<int> > readings, vector<int> counts);
    
    // Consistent wait function (use instead of other implementations)
    static int wait(unsigned long milisec);
    
private:

};

#endif	/* GLOBALS_H */

