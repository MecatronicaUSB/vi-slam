#ifndef PLUS_H_
#define PLUS_H_

#include <cmath>
#include <iostream>
#include <vector>

using namespace std;

struct Quaterniond {
    double w;
	double x;
	double y;
	double z;
	
};

Quaterniond toQuaternion(double pitch, double roll, double yaw);


static void toEulerAngle(const Quaterniond& q, double& roll, double& pitch, double& yaw);

#endif