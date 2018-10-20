#ifndef PLUS_H_
#define PLUS_H_

#include <cmath>
#include <iostream>
#include <vector>
#include "opencv2/core.hpp"
using namespace std;
using namespace cv;
struct Quaterniond {
    double w;
	double x;
	double y;
	double z;
	
};

Quaterniond toQuaternion(double pitch, double roll, double yaw);


Point3d toEulerAngle(const Quaterniond& q);

double computeDiffAng(double gt_angle, double gt_est);

#endif