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

Mat point2MatPlusOne(Point3d point);
Mat point2Mat(Point3d point);
Point3d Mat2point(Mat position);
Point3d transformationMatrix2RPY(Mat transformationMatrix);
Point3d transformationMatrix2position(Mat transformationMatrix);
Point3d rotationMatrix2RPY(Mat rotationMatrix);
Mat RPY2rotationMatrix(Point3d rpy );
Mat RPYAndPosition2transformationMatrix(Point3d rpy, Point3d position);
Mat transformationMatrix2rotationMatrix(Mat transformationMatrix );

Mat RPYWorld2ResidualAngImu(Point3d rpy ); //Sobrante

#endif