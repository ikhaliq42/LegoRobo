/* 
 * File:   Pose.h
 * Author: s1461613
 *
 * Created on 14 October 2014, 18:03
 */

#ifndef POSE_H
#define	POSE_H

class Pose {
public:
    double x, y, theta;
    
    Pose(double x_, double y_, double theta_);
    Pose(const Pose& orig);
    virtual ~Pose();
private:

};

#endif	/* POSE_H */

