/* 
 * File:   Pose.cpp
 * Author: s1461613
 * 
 * Created on 14 October 2014, 18:03
 */

#include "Pose.h"

Pose::Pose(double x_, double y_, double theta_) {
    x=x_;
    y=y_;
    theta=theta_;
}

Pose::Pose(const Pose& orig) {
}

Pose::~Pose() {
}

