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

Quaterniond toQuaternion(double roll, double pitch,  double yaw);


Point3d toRPY(const Quaterniond& q);

Point3d toRPY360(Point3d angles);

double computeDiff(double gt_angle, double gt_est);

#endif