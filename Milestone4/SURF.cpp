/**
 * @file SURF_Homography
 * @brief SURF detector + descriptor + FLANN Matcher + FindHomography
 * @author A. Huaman
 */

#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include <phidget21.h>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <boost/thread.hpp>

using namespace std;
using namespace cv;

#include <stdlib.h>
#include <math.h>
#include <cv.h>
#include <highgui.h>
#include "Robot.h"
#include "Globals.h"

/*
int AttachHandler(CPhidgetHandle IFK, void *userptr)
{
	int serialNo;
	const char *name;

	CPhidget_getDeviceName(IFK, &name);
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

int ErrorHandler(CPhidgetHandle IFK, void *userptr, int ErrorCode, const char *unknown) 
{
	printf("Error handled. %d - %s", ErrorCode, unknown);
	return 0;
}
*/


/**
 * @function readme
 */
void readme() {
    std::cout << " Usage: ./SURF_Homography <img1> <img2>" << std::endl;
}

int midPoint(vector<Point2f> scene_corners) {	
	
	int max = scene_corners[0].x;
	int min = scene_corners[0].x;
	int mid;

	for (int i = 1; i < scene_corners.size(); i++) {
		if(scene_corners[i].x < min) min = scene_corners[i].x;
		if(scene_corners[i].x > max) max = scene_corners[i].x;
	}

	mid = (min+max)/2;

	//printf("min = %i\n", min);
	//printf("max = %i\n", max);
	//printf("mid = %i\n", mid);

	return mid;
}

bool aligned(int midP, int midLineTarget, int err) {

	printf("object mid point is at: %i\n", midP);
        printf("target mid point is at: %i\n", midLineTarget);
	if(fabs(midP - midLineTarget) < err) { return true; }
	else { return false; }
}

bool tryAlign(int midP, int target, int scene_size) {
    
	// Get robot instance (singleton)
	Robot_ptr robo = Robot::getInstance();

	int camera_field = 48;	
	int clicks_per_degree = 3;
	
	int diff = fabs(target - midP);		
	double degrees;
	degrees = (diff * camera_field) * 1.0 / scene_size;
	int clicks = degrees / clicks_per_degree + 0.5;
	printf("here: target = %i, midP = %i, range = %i, diff = %i, degrees = %f, clicks = %i\n", target, midP, scene_size , diff, degrees, clicks);

	if (clicks > 0 && fabs(target - midP) > 2) {
		if (midP < target) {
			printf("Align: clicks = %i, clockwise...\n", clicks);
			//getchar();
			robo->Turn(0, clicks, defaultTurnPower);
		}
		else {
			printf("Align: clicks = %i, anticlockwise...\n");
			//getchar();
			robo->Turn(1, clicks, defaultTurnPower);
		}
	}
	else {
		return true;
	}
	return false;
}


/**
 * @function main
 * @brief Main function
 */
int vision() {

    /*
    //Declare an InterfaceKit handle
    CPhidgetInterfaceKitHandle ifKit;

    //create the InterfaceKit object
    CPhidgetInterfaceKit_create(&ifKit);

    //Set the handlers to be run when the device is plugged in or opened from software, unplugged or closed from software, or generates an error.
    CPhidget_set_OnAttach_Handler((CPhidgetHandle)ifKit, AttachHandler, NULL);
    CPhidget_set_OnDetach_Handler((CPhidgetHandle)ifKit, DetachHandler, NULL);
    CPhidget_set_OnError_Handler((CPhidgetHandle)ifKit, ErrorHandler, NULL);

    CPhidget_open((CPhidgetHandle)ifKit, -1);
    */

    // Get robot instance (singleton)
    Robot_ptr robo = Robot::getInstance();
    
    /*
    if (argc != 3) {
        readme();
        return -1;
    }*/

    Mat img_object_full = imread("iron.png", IMREAD_GRAYSCALE); 
    Mat img_object, img_scene, img_scene_full; // = imread( argv[2], IMREAD_GRAYSCALE );

    //////////////////////////////////////////////////
    //cvNamedWindow( "Example", CV_WINDOW_AUTOSIZE );
    /////////////////////////////////////////////

    //initisle camera
    VideoCapture capture(0);
    cvWaitKey(1000);

    CvCapture* capture_ptr = cvCaptureFromCAM( 1 );
    cvSetCaptureProperty( capture_ptr, CV_CAP_PROP_FRAME_WIDTH, 640 );
    cvSetCaptureProperty( capture_ptr, CV_CAP_PROP_FRAME_HEIGHT, 480 );
    //IplImage* frame;
    Mat frame;
    if (!capture.isOpened()) {
        fprintf(stderr, "Cannot initialise camera!\n");
        return 1;
    }
    int key = 0;
    Mat gray_image;

    int non_matches=0;
    ///bool targetLocked = false; //declared below
    while (robo->exitFlag == false /*key != 'q'*/) {
        frame = cvQueryFrame( capture_ptr );
        Mat bottle;
        capture.read(bottle);
        capture.read(bottle);
        capture.read(bottle);
        capture.read(bottle);
        capture.read(bottle);
        capture.read(bottle);
        capture.read(bottle);
        capture.read(bottle);
        capture.read(bottle);
        capture.read(bottle);
        capture.read(frame);
	
        //if( !frame ) break;
        /////////////////////////////////////
        //cvShowImage( "Example", frame );
        //imshow("Example", frame);
        //cvSaveImage("matteo.jpg",frame);
        key = cvWaitKey(100);
        ///////////////////////////////////
        //img_scene = imread( argv[2], IMREAD_GRAYSCALE );

        cvtColor(frame, img_scene_full, CV_BGR2GRAY);
	
	// scale image scene and add blurring
	//img_scene = img_scene_full;
 	resize(img_scene_full, img_scene, Size(), 1, 1);
	//GaussianBlur(img_scene, img_scene, Size(7,7), 2.0, 2.0);

	// scale object image and add blurring
 	resize(img_object_full, img_object, Size(), 0.7, 0.7);
	GaussianBlur(img_object, img_object, Size(7,7), 2.0, 2.0);

        //cvCvtColor(&frame, &img_scene, CV_BGR2GRAY);

        if (!img_object.data || !img_scene.data) {
            std::cout << " --(!) Error reading images " << std::endl;
            return -1;
        }

        //-- Step 1: Detect the keypoints using SURF Detector
        int minHessian = 400;

        SurfFeatureDetector detector(minHessian);

        std::vector<KeyPoint> keypoints_object, keypoints_scene;

        detector.detect(img_object, keypoints_object);
        detector.detect(img_scene, keypoints_scene);

        //-- Step 2: Calculate descriptors (feature vectors)
        SurfDescriptorExtractor extractor;

        Mat descriptors_object, descriptors_scene;

        extractor.compute(img_object, keypoints_object, descriptors_object);
        extractor.compute(img_scene, keypoints_scene, descriptors_scene);

	//////////////////// To help prevent random crashes
	descriptors_object.convertTo(descriptors_object,CV_32F);
	descriptors_scene.convertTo(descriptors_scene,CV_32F);


        //-- Step 3: Matching descriptor vectors using FLANN matcher

        FlannBasedMatcher matcher;
        std::vector< DMatch > matches;
	matcher.match(descriptors_object, descriptors_scene, matches);

        double max_dist = 0;
        double min_dist = 100;
        fprintf(stderr, "HILLO!\n");
        //-- Quick calculation of max and min distances between keypoints
        for (int i = 0; i < descriptors_object.rows; i++) {
            double dist = matches[i].distance;
            if (dist < min_dist) min_dist = dist;
            if (dist > max_dist) max_dist = dist;
        }
        //printf("-- Match" );
        printf("-- Max dist : %f \n", max_dist);
        printf("-- Min dist : %f \n", min_dist);
        //printf("-- Match" );
        //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
        std::vector< DMatch > good_matches;

        for (int i = 0; i < descriptors_object.rows; i++) {
            if (matches[i].distance < 3 * min_dist) {
                good_matches.push_back(matches[i]);
            }
        }
        //printf("Match" );
        Mat img_matches;
        drawMatches(img_object, keypoints_object, img_scene, keypoints_scene,
                good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);


        //-- Localize the object from img_1 in img_2
        std::vector<Point2f> obj;
        std::vector<Point2f> scene;

        for (size_t i = 0; i < good_matches.size(); i++) {
            //-- Get the keypoints from the good matches
            obj.push_back(keypoints_object[ good_matches[i].queryIdx ].pt);
            scene.push_back(keypoints_scene[ good_matches[i].trainIdx ].pt);
        }
	
	Mat H;
        try { H = findHomography(obj, scene, RANSAC); } 
	catch (int e) { 
		H = (Mat_ <int>(3,3) << 0, 0, 0, 0, 0, 0, 0, 0, 0);
		printf("Caught error in find homography - returning zero matrix instead.\n");
	}
	 

        //-- Get the corners from the image_1 ( the object to be "detected" )
        std::vector<Point2f> obj_corners(4);
        obj_corners[0] = Point(0, 0);
        obj_corners[1] = Point(img_object.cols, 0);
        obj_corners[2] = Point(img_object.cols, img_object.rows);
        obj_corners[3] = Point(0, img_object.rows);
        std::vector<Point2f> scene_corners(4);

        perspectiveTransform(obj_corners, scene_corners, H);


        //-- Draw lines between the corners (the mapped object in the scene - image_2 )
        Point2f offset((float) img_object.cols, 0);
        line(img_matches, scene_corners[0] + offset, scene_corners[1] + offset, Scalar(0, 0, 255), 4);
        line(img_matches, scene_corners[1] + offset, scene_corners[2] + offset, Scalar(0, 0, 255), 4);
        line(img_matches, scene_corners[2] + offset, scene_corners[3] + offset, Scalar(0, 0, 255), 4);
        line(img_matches, scene_corners[3] + offset, scene_corners[0] + offset, Scalar(0, 0, 255), 4);

        //-- Show detected matches

        double diff_x1 = scene_corners[0].x - scene_corners[2].x;
        double diff_y1 = scene_corners[0].y - scene_corners[2].y;

        double area1 = diff_x1 * diff_x1 + diff_y1 * diff_y1;

        printf("Area1: %f\n", area1);

        double diff_x2 = scene_corners[1].x - scene_corners[3].x;
        double diff_y2 = scene_corners[1].y - scene_corners[3].y;

        double area2 = diff_x2 * diff_x2 + diff_y2 * diff_y2;

        printf("Area2: %f\n", area2);	
		
	//printf("Image Matrix Rows = %i\n", img_matches.rows);	
	printf("Image Matrix Columns = %i\n", img_matches.cols);
	
	int midLineTarget = (img_matches.cols + offset.x) / 2;
	int scene_size = img_matches.cols - offset.x;
	
	// draw target line
	Point2f targetLineStart(midLineTarget, 0.0), targetLineEnd(midLineTarget, img_matches.rows);
        line(img_matches, targetLineStart, targetLineEnd, Scalar(0, 255, 0), 4);

        if (area1 > 5000 && area2 > 5000 && area1 > area2 - 25000 && area1 < area2 + 25000) {
            printf("Match\n");
            non_matches=0;
	    robo->midP = midPoint(scene_corners)+offset.x;
	    robo->objectFound = true;
	    Point2f midLineStart(robo->midP, 0.0), midLineEnd(robo->midP, img_matches.rows);
	    bool targetLocked = false;
	    line(img_matches, midLineStart, midLineEnd, Scalar(0, 0, 255), 4);
	    if(!aligned(robo->midP, midLineTarget, 10)) {
		targetLocked = tryAlign(scene_size, robo->midP, midLineTarget);
	    } else {
		targetLocked = true;
	    }
	    //boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
	    //break;
	    
            if (targetLocked) {
		robo->DriveMotorsStraight(0, -50);
		break;
	    //} else {
		//robo->DriveMotorsStraight(0, 0);
		//break;
	    }
	    /*
	    if (aligned(scene_size, robo->midP, 10)) {
		robo->DriveMotorsStraight(0, -50);
	    } else {
		//robo->DriveMotorsStraight(0, 0);
	    }
	    */
	    printf("Image mid point = %i\n", robo->midP);
            //CPhidgetInterfaceKit_setOutputState(ifKit, 0, 1);
        }/* else if (targetLocked) {
	    robo->DriveMotorsStraight(0, 0);
	    printf("I have the cube. Mwhaha!!");
	}*/ else {
            printf("Nope :(\n");
	    printf("Non matches = %i\n", non_matches);
	    robo->objectFound = false;
            non_matches=non_matches+1;	    
	    if(non_matches>4) {
	        non_matches=0;		
		robo->Turn(1,16,defaultTurnPower);
	    }
            //CPhidgetInterfaceKit_setOutputState(ifKit, 0, 0);
	    
        }
        imshow("Good Matches & Object detection", img_matches);
        // printf("Match" );


	//  key = cvWaitKey(1000);
        //  if (key =='t') cvSaveImage("matteo.jpg",frame);
    }
	
    //cvDestroyWindow( "Example" );
    
    //Close Phidgets
    //CPhidget_close((CPhidgetHandle)ifKit);
    //CPhidget_delete((CPhidgetHandle)ifKit);

    return 0;
}



int main(int argc, char** argv) {

    // Get robot instance (singleton)
    Robot_ptr robo = Robot::getInstance();
    
    robo->InitialiseMotors();
    robo->initialiseInterfacekit();

    Globals::wait(5000);

    robo->startFlag = false;
    robo->exitFlag = false; // just in case
    
    robo->objectFound = false;    

    printf("\nStart flag = %d\n", robo->startFlag);	

    // wait for button press on robot
    while (robo->startFlag == false) {}    

    // start vision thread
    boost::thread visionThread(vision);
    //vision();	
    visionThread.interrupt();
    //robo->DriveMotorsStraight(0, 0);
 
 
    //robo->DriveMotorsRotate(0,defaulTurnPower);
    //Globals::wait(5000);


    int attempts = 0;    
    while (robo->exitFlag == false) {
        /*
	if(robo->objectFound == true && attempts <= 3) {
	    int m = robo->midP;	    
	    while(!aligned(m, midLineTarget, 10)) { 
		tryAlign(m, midLineTarget);
		attempts = attempts + 1;
		if (attempts > 3) break;
            }
	}
	if (attempts > 3) break;*/
    }
    printf("joining vision thread ------------>\n");
    //visionThread.join();
    

    robo->ShutDownInterfacekit();
    robo->ShutDownMotors();
    robo->~Robot();


    return 0;
}

