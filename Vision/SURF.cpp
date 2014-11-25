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

using namespace std;
using namespace cv;

#include <stdlib.h>
#include <math.h>
#include <cv.h>
#include <highgui.h>

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


int midPoint(vector<Point2f> scene_corners) {	
	
	int max = scene_corners[0].x;
	int min = scene_corners[0].x;
	int mid;

	for (int i = 1; i < scene_corners.size(); i++) {
		if(scene_corners[i].x < min) min = scene_corners[i].x;
		if(scene_corners[i].x > max) max = scene_corners[i].x;
	}

	mid = (min+max)/2;

	printf("min = %i\n", min);
	printf("max = %i\n", max);
	printf("mid = %i\n", mid);

	return mid;
}

void readme();

/**
 * @function main
 * @brief Main function
 */
int main(int argc, char** argv) {

    //Declare an InterfaceKit handle
    CPhidgetInterfaceKitHandle ifKit;

    //create the InterfaceKit object
    CPhidgetInterfaceKit_create(&ifKit);

    //Set the handlers to be run when the device is plugged in or opened from software, unplugged or closed from software, or generates an error.
    CPhidget_set_OnAttach_Handler((CPhidgetHandle)ifKit, AttachHandler, NULL);
    CPhidget_set_OnDetach_Handler((CPhidgetHandle)ifKit, DetachHandler, NULL);
    CPhidget_set_OnError_Handler((CPhidgetHandle)ifKit, ErrorHandler, NULL);

    CPhidget_open((CPhidgetHandle)ifKit, -1);



    if (argc < 2) {
        readme();
        return -1;
    }

    Mat img_object = imread(argv[1], IMREAD_GRAYSCALE); 
    Mat img_scene;

    //////////////////////////////////////////////////
    //cvNamedWindow( "Example", CV_WINDOW_AUTOSIZE );
    /////////////////////////////////////////////

    //initisle camera
    VideoCapture capture(0);
    cvWaitKey(1000);

    CvCapture* capture2 = cvCaptureFromCAM( 1 );
    cvSetCaptureProperty( capture2, CV_CAP_PROP_FRAME_WIDTH, 640 );
    cvSetCaptureProperty( capture2, CV_CAP_PROP_FRAME_HEIGHT, 480 );
    IplImage* frame2;
    Mat frame;
    if (!capture.isOpened()) {
        fprintf(stderr, "Cannot initialise camera!\n");
        return 1;
    }
    int key = 0;
    Mat gray_image;
    while (key != 'q') {
        frame = cvQueryFrame( capture2 );
       /* Mat bottle;
        capture.read(bottle);
        capture.read(bottle);
        capture.read(bottle);
        capture.read(bottle);
        capture.read(bottle);
        capture.read(bottle);
        capture.read(bottle);
        capture.read(bottle);
        capture.read(bottle);
        capture.read(bottle); */
        capture.read(frame);
        //if( !frame ) break;
        /////////////////////////////////////
        //cvShowImage( "Example", frame2 );
       // imshow("Example", frame);
        //cvSaveImage("matteo.jpg",frame);
        key = cvWaitKey(100);
        ///////////////////////////////////

        cvtColor(frame, img_scene, CV_BGR2GRAY);



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

	////////////////////
	descriptors_object.convertTo(descriptors_object,CV_32F);
	descriptors_scene.convertTo(descriptors_scene,CV_32F);
        //-- Step 3: Matching descriptor vectors using FLANN matcher

        FlannBasedMatcher matcher;
        std::vector< DMatch > matches;
        matcher.match(descriptors_object, descriptors_scene, matches);

        double max_dist = 0;
        double min_dist = 100;
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
        
	Mat H = findHomography(obj, scene, RANSAC);
        
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

        if (area1 > 5000 && area2 > 5000 && area1 > area2 - 25000 && area1 < area2 + 25000) {
            printf("Match\n");
	    int midPx = midPoint(scene_corners) + offset.x;
	    Point2f midLineStart(midPx, 0.0), midLineEnd(midPx, img_matches.rows);
	    line(img_matches, midLineStart, midLineEnd, Scalar(0, 0, 255), 4);
	    Point2f targetLineStart(680, 0.0), targetLineEnd(680, img_matches.rows);
	    line(img_matches, targetLineStart, targetLineEnd, Scalar(0, 255, 0), 4);
            CPhidgetInterfaceKit_setOutputState(ifKit, 0, 1);
        } else {
            printf("Nope :(\n");
            CPhidgetInterfaceKit_setOutputState(ifKit, 0, 0);
        }
        imshow("Good Matches & Object detection", img_matches);
        // printf("Match" );


      //  key = cvWaitKey(1000);
        //if (key =='t') cvSaveImage("matteo.jpg",frame);
    }
    //cvDestroyWindow( "Example" );
    
    //Close Phidgets
    CPhidget_close((CPhidgetHandle)ifKit);
    CPhidget_delete((CPhidgetHandle)ifKit);

    return 0;
}

/**
 * @function readme
 */
void readme() {
    std::cout << " Usage: ./SURF_Homography <img1>" << std::endl;
}

