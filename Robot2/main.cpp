/* 
 * File:   main.cpp
 * Author: s1461613
 *
 * Created on 09 October 2014, 11:39
 */

#include <cstdlib>
#include <stdio.h>
#include <math.h>
#include "Handlers.h"
#include "Robot.h"
#include "Globals.h"
#include <iostream>
#include <vector>

using namespace std;

int square = 0;

void PositionCheck(Robot *robo) {

    int THESEHOLD = 30;
    int match = 0;



    int n = (int) robo->poses.size() - 1;
    printf("N is: %d\n", n);

    //int a = robo->poses.at(n).x;


    if (n >= 4) {
        double fred = sqrt(pow(robo->xa[n] - robo->xa[n - 4], 2) + pow(robo->ya[n] - robo->ya[n - 4], 2));
        printf("Fred: %f", fred);

        /*    if (sqrt(pow((robo->poses.at(n).x - robo->poses.at(n - 3).x), 2) + pow((robo->poses.at(n).y - robo->poses.at(n - 3).y), 2)) < THESEHOLD)
                match = 3;
            if (sqrt(pow((robo->poses.at(n).x - robo->poses.at(n - 4).x), 2) + pow((robo->poses.at(n).y - robo->poses.at(n - 4).y), 2)) < THESEHOLD)
                match = 4;*/

        if (sqrt(pow((robo->xa[n] - robo->xa[n - 3]), 2) + pow((robo->ya[n] - robo->ya[n - 3]), 2)) < THESEHOLD)
            match = 3;
        if (sqrt(pow(robo->xa[n] - robo->xa[n - 4], 2) + pow(robo->ya[n] - robo->ya[n - 4], 2)) < THESEHOLD)
            match = 4;
        //current x,y,theta
        // match = 4;
        //list of previous x,y,theta



        //get distance between current and past 3-4(match), n = current 

        // if distance is less then preset amount (match)

        //get x min, y min, x max, y max

        //get area

        //compare to known box areas

        //decide which we're in
        if (match == 3 || match == 4) {
            printf("MATCH!");
            int minX = 1000;
            int minY = 1000;
            int maxX = -1000;
            int maxY = -1000;
            for (int i = n; i > (n - match); --i) { // run for the last three
                printf("i = %d\n", i);
                if (minX > robo->xa[i]) {
                    minX = robo->xa[i];
                }
                if (minY > robo->ya[i]) {
                    minY = robo->ya[i];
                }
                if (maxX < robo->xa[i]) {
                    maxX = robo->xa[i];
                }
                if (maxY < robo->ya[i]) {
                    maxY = robo->ya[i];
                }
            }
            //printf("MaxX: %d, MaxY %d, MinX %d, MinY %d\n", maxX, maxY  ,minX ,minY);
            double difX = maxX - minX;
            double difY = maxY - minY;
            double area = difX * difY;
            printf("Area of box: %f\n", area);
            printf("DIffX: %f, DiffY %f\n", difX, difY);

            if (area > 18000 && area < 38000){
                square = 1;
            }
            else if (area > 42000 && area < 54000){
                square = 2;
            }
            else if (area > 55000 && area < 80000){
                square = 3;
            }
            else square = 0;
            printf("We are in box: %d \n", square);
            //int square = square;
            //pick square using diffX, diffY, area


        }
    }
    //    return 0;
}

void gotobase(Robot *robo) {
    //exit square
    printf("try to exit square\n");
    int q = (int) robo->poses.size() - 1;
    robo->Turn(0, 34, 40);
    robo->tooCloseResponse(0);
    int p = (int) robo->poses.size() - 1 - q;
    //while(abs(robo->xa[q + p + 2] - robo->xa[q - p]) < 40 || abs(robo->ya[q+p +2] - robo->ya[q-p]) < 40){
    double jim = pow(robo->xa[robo->poses.size() - 1] - robo->xa[robo->poses.size() - 1 - p],2);
//            std::abs(robo->ya[robo->poses.size() - 1] - robo->ya[robo->poses.size() - 1 - p]);
    while(std::abs(robo->xa[robo->poses.size() - 1] - robo->xa[robo->poses.size() - 1 - p]) < 60
            && std::abs(robo->ya[robo->poses.size() - 1] - robo->ya[robo->poses.size() - 1 - p]) < 60){
          
        robo->DriveMotorsStraight(0, 100);
        while (robo->tooClose == false ) {

        }
           robo->SetServo(1);
           Globals::wait(100);

        //robo->SetServo(1);
        //Globals::wait(100);
        p = (int) robo->poses.size() - 1 - q;
        
        if (std::abs(robo->xa[robo->poses.size() - 1] - robo->xa[robo->poses.size() - 1 - p]) < 60
                && std::abs(robo->ya[robo->poses.size() - 1] - robo->ya[robo->poses.size() - 1 - p]) < 60) {
            robo->tooCloseResponse(0);
            printf("still in square\n");
        }
        //while
        //if ( abs(robo->xa[q + p] - robo->xa[q - p]) > 40 || abs(robo->ya[q+p] - robo->ya[q-p]) > 40);
        //else (robo->tooCloseResponse(1);
    }
    printf("left square\n");
    //robo->tooCloseResponse(1);
    //turn 90 degrees left
    //turn left with wall
    //go forward until wall
    if (square == 1) {
        robo->Turn(1, 34, 40);
        robo->Move(0, 100, 100);
        robo->Turn(0, 34, 40);
        robo->DriveMotorsStraight(defaultDirection, defaultPower);
        while (robo->tooClose == false);
        robo->tooCloseResponse(1);
        robo->DriveMotorsStraight(defaultDirection, defaultPower);
        while (robo->tooClose == false);
        robo->DriveMotorsStop();
        //go forward to wall
        //turn right with wall
        ////we're there

    } else if (square == 2) {
        robo->tooCloseResponse(0);
        robo->DriveMotorsStraight(defaultDirection, defaultPower);
        while (robo->tooClose == false);
        robo->tooCloseResponse(0);
        robo->DriveMotorsStraight(defaultDirection, defaultPower);
        while (robo->tooClose == false);           
                robo->Turn(1, 34, 40);
        robo->Move(0, 100, 100);
        robo->Turn(0, 34, 40);
        robo->DriveMotorsStraight(defaultDirection, defaultPower);
        while (robo->tooClose == false);
        robo->tooCloseResponse(1);
        robo->DriveMotorsStraight(defaultDirection, defaultPower);
        while (robo->tooClose == false);
        robo->DriveMotorsStop();
                //turn left
        //go forward to wall
        //turn left
        //go forward to wall
                //turn 90 degrees right
        //go forward x clicks
        //turn 90 degrees left
        //go forward to wall
        //turn right with wall
        ////we're there

    } else if (square == 3) {
                robo->tooCloseResponse(0);
                robo->Move(0, 290, 100);
                robo->Turn(1, 34, 40);
                        robo->DriveMotorsStraight(defaultDirection, defaultPower);
        while (robo->tooClose == false);
                robo->tooCloseResponse(1);
        robo->DriveMotorsStraight(defaultDirection, defaultPower);
        while (robo->tooClose == false);
        robo->DriveMotorsStop();
        //turn left with wall
        //go forward x clicks
        //turn right 90 degree
        //go forward to wall
        //turn right with wall
        //go forward to wall
        //we're there

    } else if (square == 4) {
        //turn 150 degrees left
        //go forward to wall
        //turn right
        //go to wall
        //stop, we're there

    } else {
        printf("Fail!\n");
    }
}

void IanFaceWall(Robot *robo) {
    //Robot *robo = new Robot;

    robo->DriveMotorsStop();
    Globals::wait(100);
    int IR1Ian;
    int IR2Ian;
    int errorMargin = 3;
    int turnClicks = 1;
    int turndir = 0;
    bool linedUp = false;
    int fudgeValue = 5;

    IR1Ian = robo->get_averaged_sensor_reading(3, 10);
    IR2Ian = robo->get_averaged_sensor_reading(4, 10) + fudgeValue;
    while (!linedUp) {
        if (IR1Ian > (IR2Ian + errorMargin)) {
            robo->Turn(0, turnClicks, defaultTurnPower);
            robo->DriveMotorsStop();
            Globals::wait(100);
            IR1Ian = robo->get_averaged_sensor_reading(3, 10);
            IR2Ian = robo->get_averaged_sensor_reading(4, 10) + fudgeValue;
            turndir = 1;
        } else if (IR1Ian < (IR2Ian - errorMargin)) {
            robo->Turn(1, turnClicks, defaultTurnPower);
            robo->DriveMotorsStop();
            Globals::wait(100);
            IR1Ian = robo->get_averaged_sensor_reading(3, 10);
            IR2Ian = robo->get_averaged_sensor_reading(4, 10) + fudgeValue;
            turndir = 2;
        } else {
            //printf("Looks ok - press any key...!\n");
            //getchar();
            if (turndir == 1) {
                //printf("final adjustment ...1 click anticlockwise!\n");
                //robo->Turn(0, 1, defaultTurnPower);
            }
            if (turndir == 2) {
                //printf("final adjustment...1 clicks clockwise!\n");
                //robo->Turn(1, 1, defaultTurnPower);;
            }
            linedUp = true;
            printf("happy days!\n");
        }
        printf("Current Values are IR1: %d, IR2: %d, diff: %d\n", IR1Ian, IR2Ian, IR1Ian - IR2Ian);

    }

    printf("happy days!\n");
    robo->Turn(1, 34, defaultPower);
}

void MoveAndAvoid() {

    Robot *robo = new Robot;

    robo->InitialiseMotors();
    robo->initialiseInterfacekit();
    //srand((unsigned) time(&t));

    robo->startFlag = false;
    robo->exitFlag = false; // just in case

    // wait for button press on robot
    while (robo->startFlag == false) {
    }

    //start moving
    //robo->turning = false;
    printf("\nMoving...\n");
    robo->DriveMotorsStraight(defaultDirection, defaultPower);
    //int numOfTurns = 0;
    while (1) {

        if (robo->exitFlag == true) {
            printf("Exit flag detected. Breaking... \n ");
            break;
        }
        if (robo->activatedWhisker > 0) {
            robo->WhiskerChangeResponse();
            //numOfTurns += 1;
            robo->DriveMotorsStraight(defaultDirection, defaultPower);
            printf("activatedWhisker = %i", robo->activatedWhisker);
        }

        if (robo->tooClose == true /*&& robo->turning() == false*/) {
            // IanFaceWall(robo); 
            //robo->DriveMotorsStop();
            //Globals::wait(100);
            //numOfTurns += 1;
            robo->tooCloseResponse(1);
            printf("PositionCheckRunning\n");
            PositionCheck(robo);
            if (square != 0) {
                gotobase(robo);
                printf("are we there?\n");
                break;
            } else {
                robo->DriveMotorsStraight(defaultDirection, defaultPower);
                printf("going straight\n");
            }
        }
        /*
                if (numOfTurns > 3) {
                    PositionCheck();
                }
         */
        /*       if ((robo->wallskimLeft || robo->wallskimRight)) {
                   robo->wallSkimResponse();            
                   robo->DriveMotorsStraight(defaultDirection, defaultPower);
               }*/

    }
    printf("\nDONE\n");
    robo->DriveMotorsStop();
    robo->ShutDownInterfacekit();
    robo->ShutDownMotors();
    robo->~Robot();
    //return 0;

}

void FaceWall() {

    //hall counts determines number of turns
    //period determines how many hall counts between each reading 

    vector< vector<int> > readings(9); // readings by ordered sensor index first, then sequence of values    
    vector<int> counts;
    vector< vector<int> > distances(9); // distances as estimated by sensor readings
    vector<int> minDistances(9); // minimum distances recorded by each sensor  
    vector<int> minDistanceCounts(9); // hall counts relating to minimum distances recorded by each sensor  
    int val;
    int bestGuess = 0;


    Robot *robo = new Robot();

    robo->InitialiseMotors();
    robo->initialiseInterfacekit();
    //srand((unsigned) time(&t));

    robo->startFlag = false;
    robo->exitFlag = false; // just in case

    // wait for button press on robot
    while (robo->startFlag == false) {
    }

    int hallCounts = 69, period = 3;

    // read from all sensors
    for (int n = 0; n <= hallCounts; n = n + period) {

        // turn robot for 1 hall count
        robo->Turn(1, period, defaultTurnPower);

        // take sensor readings and calculated distances
        for (int i = 1; i < readings.size(); i++) {

            // readings
            val = robo->get_averaged_sensor_reading(i, 10);
            readings.at(i).push_back(val);

            // distances            
            if (i == ir1Index || i == ir2Index) distances.at(i).push_back(val = Globals::distanceConversion(val));
            if (i == sonarIndex) distances.at(i).push_back(val = Globals::sonarDistanceConversion(val));
        }
        counts.push_back(n);
    }

    //Output readings
    Globals::outputSensorReadings(readings, counts);
    Globals::outputDistances(distances, counts);

    // find hall counts relating to smallest distance estimates per reading
    for (int i = 1; i < distances.size(); i++) {
        for (int n = 0; n < distances.at(i).size(); n++) {
            if (distances.at(i).at(n) < minDistances.at(i)) {
                minDistances.at(i) = distances.at(ir1Index).at(n);
                minDistanceCounts.at(i) = counts.at(n);
            }
        }
    }

    // interpolate for best guess - just use IR sensors for now
    bestGuess = hallCounts - (int) ((minDistanceCounts.at(ir1Index) + minDistanceCounts.at(ir2Index)) / 2);

    // turn to best guess point
    robo->Turn(0, bestGuess, defaultTurnPower);

    // shut down interfaces   
    robo->DriveMotorsStop();
    robo->ShutDownInterfacekit();
    robo->ShutDownMotors();
    robo->~Robot();

}

void MoveSquare() {

    Robot *robo = new Robot;

    robo->InitialiseMotors();
    robo->initialiseInterfacekit();
    //srand((unsigned) time(&t));

    robo->startFlag = false;
    robo->exitFlag = false; // just in case    
    int turns = 0;

    Pose startPose(0, 0, 0);
    Pose endPose(15, 75, 270);

    // wait for button press on robot
    while (robo->startFlag == false) {
    }

    //start moving
    //robo->turning = false;
    printf("\nMoving...\n");
    robo->DriveMotorsStraight(1, defaultPower);
    while (turns < 5) {

        if (robo->exitFlag == true) {
            printf("Exit flag detected. Breaking... \n ");
            break;
        }
        if (robo->activatedWhisker > 0) {
            robo->WhiskerChangeResponse();
            robo->DriveMotorsStraight(defaultDirection, defaultPower);
            printf("activatedWhisker = %i", robo->activatedWhisker);
        }
        if (robo->lines.back() > 50) {
            robo->Turn(0, 45, defaultTurnPower);
            robo->DriveMotorsStraight(defaultDirection, defaultPower);
            turns++;
        }
    }
    robo->DriveMotorsStop();

    //output clicks
    //robo->outputClicks();

    //build path and return pose
    //robo->buildPath();
    //robo->outputPath();
    //endPose = robo->buildPose(startPose, 0);

    printf("\nStart Pose: %i, %i, %i", (int) startPose.x, (int) startPose.y, (int) startPose.theta);
    printf("\nEnd Pose: %i, %i, %i", (int) endPose.x, (int) endPose.y, (int) endPose.theta);

    // Shut down interfaces
    robo->ShutDownInterfacekit();
    robo->ShutDownMotors();

    // build pose and output result
    //endPose = robo->buildPose(startPose, 0);    
    printf("\nStart Pose : (i%, i%, i%)", (int) startPose.x, (int) startPose.y, (int) startPose.theta);
    printf("\nEnd Pose : (i%, i%, i%)", (int) endPose.x, (int) endPose.y, (int) endPose.theta);
    printf("\nFinal Hall count = %i\n", robo->HallCount);

    robo->~Robot();


    //return 0;

}

void MoveForward() {

    Robot *robo = new Robot;
    Pose startPose(0, 0, 0), endPose(0, 0, 0);

    robo->InitialiseMotors();
    robo->initialiseInterfacekit();
    //srand((unsigned) time(&t));

    robo->startFlag = false;
    robo->exitFlag = false; // just in case


    // wait for button press on robot
    while (robo->startFlag == false) {
    }

    //start moving
    //robo->turning = false;
    printf("\nMoving...\n");
    robo->Move(1, 112, defaultPower);

    //output clicks
    //robo->outputClicks();

    //build path and return pose
    //robo->buildPath();
    //robo->outputPath();
    //endPose = robo->buildPose(startPose, 0);

    printf("\nStart Pose: %i, %i, %i", (int) startPose.x, (int) startPose.y, (int) startPose.theta);
    printf("\nEnd Pose: %i, %i, %i", (int) endPose.x, (int) endPose.y, (int) endPose.theta);
    printf("\nFinal Hall count = %i\n", robo->HallCount);

    robo->ShutDownInterfacekit();
    robo->ShutDownMotors();
    robo->~Robot();


    //return 0;

}

void Rotate() {

    Robot *robo = new Robot;

    robo->InitialiseMotors();
    robo->initialiseInterfacekit();
    //srand((unsigned) time(&t));

    robo->startFlag = false;
    robo->exitFlag = false; // just in case


    // wait for button press on robot
    while (robo->startFlag == false) {
    }

    //start moving
    //robo->turning = false;
    printf("\nMoving...\n");
    robo->Turn(1, 200, defaultPower);

    //output clicks
    //robo->outputClicks();

    //output path
    //robo->buildPath();
    //robo->outputPath();

    printf("\nFinal Hall count = %i\n", robo->HallCount);

    printf("\nDONE\n");
    robo->DriveMotorsStop();

    printf("\nFinal Hall count = %i\n", robo->HallCount);

    robo->ShutDownInterfacekit();
    robo->ShutDownMotors();
    robo->~Robot();


    //return 0;

}

int main(int argc, char** argv) {

    MoveAndAvoid();
    //MoveForward();
    // FaceWall();
    //MoveSquare();
    //Rotate();
    return 0;
}


