/* 
 * File:   Globals.cpp
 * Author: s1461613
 * 
 * Created on 09 October 2014, 18:28
 */

#include "Globals.h"
#include <stdio.h>
#include <sys/time.h>


//#include <cstdlib>

//#include <math.h>

#include <iostream>


using namespace std;

// set parameters
int safeDistance = 30;
int safeDistance2 = 30;
int defaultDirection = 0;
int defaultPower = 100;
int defaultTurnPower = 40;
int defaultReversePower = 30;
//int ServoPos = 2; // 0 = 180 degrees, 1 = 130 degrees.
int ir1Index = 3;
int ir2Index = 4;
int sonarIndex = 3;
int irThreshold = 5;

Globals::Globals() {
}

Globals::Globals(const Globals& orig) {
}

Globals::~Globals() {
}

// distance conversion function for IR sensor
double Globals::distanceConversion(int sensorValue) {    
    return 4800 / (sensorValue - 20);
}

// distance conversion function for sonar sensor
double Globals::sonarDistanceConversion(int sensorValue) {
    return (int)(sensorValue * 1.296);
}

// output sensor readings
void Globals::outputSensorReadings(vector< vector<int> > readings, vector<int> counts) {

    FILE * pFile;
   
    pFile = fopen("sensor_data.txt", "w");   
    
    //counts
    fprintf(pFile, "\t\t");
    for (int n = 0; n < counts.size(); n++) {
        fprintf(pFile, "%i\t", counts.at(n));
    }
    
    //readings
    for (int i = 1; i < readings.size(); i++) {
        fprintf(pFile, "\nSensor %i:\t", i);
        for (int n = 0; n < readings.at(i).size(); n++) {
            fprintf(pFile, "%i\t", readings.at(i).at(n));
        }   
    }
    
    fclose(pFile);
    
    
    
}

// output sensor readings
void Globals::outputDistances(vector< vector<int> > distances, vector<int> counts) {

    FILE * pFile;
   
    pFile = fopen("distances.txt", "w");   
    
    //counts
    fprintf(pFile, "\t\t");
    for (int n = 0; n < counts.size(); n++) {
        fprintf(pFile, "%i\t", counts.at(n));
    }
    
    //readings
    for (int i = 1; i < distances.size(); i++) {
        fprintf(pFile, "\nSensor %i:\t", i);
        for (int n = 0; n < distances.at(i).size(); n++) {
            fprintf(pFile, "%i\t", distances.at(i).at(n));
        }   
    }
    
    fclose(pFile);
    
}
 
// Consistent wait function (use instead of other implementations)
int Globals::wait(unsigned long milisec) {
    struct timespec req = {0};
    time_t sec = (int) (milisec / 1000);
    milisec = milisec - (sec * 1000);
    req.tv_sec = sec;
    req.tv_nsec = milisec * 1000000L;
    while (nanosleep(&req, &req) == -1)
        continue;
    return 1;
}

