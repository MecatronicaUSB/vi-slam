#include "../include/Plus.hpp"

Quaterniond toQuaternion( double roll, double pitch, double yaw)
{
	Quaterniond q;
        // Abbreviations for the various angular functions
	double cy = cos(yaw * 0.5);
	double sy = sin(yaw * 0.5);
	double cr = cos(roll * 0.5);
	double sr = sin(roll * 0.5);
	double cp = cos(pitch * 0.5);
	double sp = sin(pitch * 0.5);

	q.w = cy * cr * cp + sy * sr * sp;
	q.x = cy * sr * cp - sy * cr * sp;
	q.y = cy * cr * sp + sy * sr * cp;
	q.z = sy * cr * cp - cy * sr * sp;
	return q;
}



Point3d toRPY(const Quaterniond& q)
{
	// roll (x-axis rotation)
	double roll, pitch,  yaw;
	



	double sinr_cosp = +2.0 * (q.w * q.x + q.y * q.z);
	double cosr_cosp = +1.0 - 2.0 * (q.x * q.x + q.y * q.y);
	roll = atan2(sinr_cosp, cosr_cosp);

	// pitch (y-axis rotation)
	double sinp = +2.0 * (q.w * q.y - q.z * q.x);
	if (fabs(sinp) >= 1)
		pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
	else
		pitch = asin(sinp);

	// yaw (z-axis rotation)
	double siny_cosp = +2.0 * (q.w * q.z + q.x * q.y);
	double cosy_cosp = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);  
	yaw = atan2(siny_cosp, cosy_cosp);

	Point3d angles;
	angles.x = roll;
	angles.y = pitch;
	angles.z = yaw;


	return angles;
}

Point3d rotationMatrix2RPY(Matx33f rotationMatrix)
{
	// roll (x-axis rotation)
	double roll, pitch,  yaw;
	double r11 = rotationMatrix(0,0);
	double r12 = rotationMatrix(0,1);
	double r13 = rotationMatrix(0,2);

	double r21 = rotationMatrix(1,0);
	double r22 = rotationMatrix(1,1);
	double r23 = rotationMatrix(1,2);

	double r31 = rotationMatrix(2,0);
	double r32 = rotationMatrix(2,1);
	double r33 = rotationMatrix(2,2);

	yaw = atan2(r21, r11);
	pitch = atan2(-r31, sqrt(r32*r32+r33*r33));
	roll = atan2(r32, r33);

	Point3d angles;
	angles.x = roll;
	angles.y = pitch;
	angles.z = yaw;


	return angles;
}


Matx44d PAndR2T(Matx33f rotationMatrix, Point3f position)
{
	// roll (x-axis rotation)
	Matx44d T;
	T(0,0)= rotationMatrix(0,0);
	T(0,1) = rotationMatrix(0,1);
	T(0,2) = rotationMatrix(0,2);

	T(1,0) = rotationMatrix(1,0);
	T(1,1) = rotationMatrix(1,1);
	T(1,2) = rotationMatrix(1,2);

	T(2,0) = rotationMatrix(2,0);
	T(2,1) = rotationMatrix(2,1);
	T(2,2) = rotationMatrix(2,2);

	T(0,3) = position.x;
	T(1,3) = position.y;
	T(2,3) = position.z;

	T(3,0) = 0.0;
	T(3,1) = 0.0;
	T(3,2) = 0.0;
	T(3,3) = 1.0;



	return T;
}


Point3d transformationMatrix2RPY(Matx44d transformationMatrix)
{
	// roll (x-axis rotation)
	double roll, pitch,  yaw;
	
	double r11 = transformationMatrix(0,0);
	double r12 = transformationMatrix(0,1);
	double r13 = transformationMatrix(0,2);

	double r21 = transformationMatrix(1,0);
	double r22 = transformationMatrix(1,1);
	double r23 = transformationMatrix(1,2);

	double r31 = transformationMatrix(2,0);
	double r32 = transformationMatrix(2,1);
	double r33 = transformationMatrix(2,2);

	yaw = atan2(r21, r11);
	pitch = atan2(-r31, sqrt(r32*r32+r33*r33));
	roll = atan2(r32, r33);



	Point3d angles;
	angles.x = roll;
	angles.y = pitch;
	angles.z = yaw;


	return angles;
}

Point3d transformationMatrix2position(Matx44d transformationMatrix)
{
	Point3d translation;

	translation.x = transformationMatrix(0,3) ;
	translation.y = transformationMatrix(1,3) ;
	translation.z = transformationMatrix(2,3) ;

	return translation;


}

double computeDiff(double angle_ref, double angle2)
{
	double diff;
	double diff2;
	diff2 = abs(angle_ref-angle2);
	if (diff2>M_PI){
		diff2 = abs(diff2-2*M_PI);
	}
	if (angle_ref<0.0)
	{
		angle_ref= angle_ref+2*M_PI;
	}
	if (angle2<0.0)
	{
		angle2 = angle2+2*M_PI;
	}
	diff = angle_ref-angle2;

	if ((diff < M_PI) && (diff>0.0)){
		return -diff2;
	}
	else if (diff < - M_PI){
		return -diff2;
	}
	else{
		return diff2;
	}

	

}

Point3d toRPY360(Point3d angles)
{
	Point3d transform360;
	transform360 = angles;
	if (angles.x < 0.0)
	{
		transform360.x = angles.x +2*M_PI;
	}
	if (angles.y < 0.0)
	{
		transform360.y = angles.y +2*M_PI;
	}
	if (angles.z < 0.0)
	{
		transform360.z = angles.z +2*M_PI;
	}

	return transform360;
}


Matx33f RPY2rotationMatrix(Point3d rpy )
{
	// roll (x-axis rotation)
	double roll = rpy.x;
    double pitch = rpy.y;
    double yaw = rpy.z;

    //cout << "roll = " << roll*180/M_PI << "pitch" << pitch*180/M_PI << "yaw" << yaw*180/M_PI <<endl;
    double c1 = cos(roll);
    double s1 = sin(roll);
    double c2 = cos(pitch);
    double s2 = sin(pitch);
    double c3 = cos(yaw);
    double s3 = sin(yaw);
	
	Matx33f rotationMatrix ;// = Mat::zeros(3,3,CV_32FC1);


	rotationMatrix(0,0) = c3*c2;
	rotationMatrix(0,1) = c3*s2*s1-s3*c1;
	rotationMatrix(0,2) = c3*s2*c1+s3*s1;

	rotationMatrix(1,0) = s3*c2;
	rotationMatrix(1,1) = s3*s2*s1+c3*c1;
	rotationMatrix(1,2) = s3*s2*c1-c3*s1;

	rotationMatrix(2,0) = -s2;
	rotationMatrix(2,1) = c2*s1;
	rotationMatrix(2,2) = c2*c1;

	/*
	worldVector.x = c3*c2*acc.x + (c3*s2*s1-s3*c1)*acc.y+(c3*s2*c1+s3*s1)*acc.z;
    worldVector.y = s3*c2*acc.x + (s3*s2*s1+c3*c1)*acc.y +(s3*s2*c1-c3*s1)*acc.z;
    worldVector.z = -s2*acc.x +c2*s1*acc.y +c2*c1*acc.z;
	*/


	return rotationMatrix;
}

Matx33f transformationMatrix2rotationMatrix(Matx44d transformationMatrix )
{
	Matx33f rotationMatrix ;

	rotationMatrix(0,0) = transformationMatrix(0,0);
	rotationMatrix(0,1) = transformationMatrix(0,1);
	rotationMatrix(0,2) = transformationMatrix(0,2);

	rotationMatrix(1,0) = transformationMatrix(1,0);
	rotationMatrix(1,1) = transformationMatrix(1,1);
	rotationMatrix(1,2) = transformationMatrix(1,2);

	rotationMatrix(2,0) = transformationMatrix(2,0);
	rotationMatrix(2,1) = transformationMatrix(2,1);
	rotationMatrix(2,2) = transformationMatrix(2,2);

	return rotationMatrix;

}


Mat point2MatPlusOne(Point3d point)
{
	Mat pointMat = Mat::zeros(4,1,CV_32FC1);

	pointMat.at<float>(0,0) = point.x;
	pointMat.at<float>(1,0) = point.y;
	pointMat.at<float>(2,0) = point.z;
	pointMat.at<float>(3,0) = 1.0;


	return pointMat;


}

Mat point2Mat(Point3d point)
{
	Mat pointMat = Mat::zeros(3,1,CV_32FC1);

	pointMat.at<float>(0,0) = point.x;
	pointMat.at<float>(1,0) = point.y;
	pointMat.at<float>(2,0) = point.z;

	return pointMat;


}

Point3d Mat2point(Mat pointMat)
{
	Point3d point;
	point.x = double(pointMat.at<float>(0,0));
	point.y =double( pointMat.at<float>(1,0));
	point.z = double(pointMat.at<float>(2,0));

	return point;


}


Matx44d RPYAndPosition2transformationMatrix(Point3d rpy, Point3d position)
{
	
	double roll = rpy.x;
    double pitch = rpy.y;
    double yaw = rpy.z;

    //cout << "roll = " << roll*180/M_PI << "pitch" << pitch*180/M_PI << "yaw" << yaw*180/M_PI <<endl;
    double c1 = cos(roll);
    double s1 = sin(roll);
    double c2 = cos(pitch);
    double s2 = sin(pitch);
    double c3 = cos(yaw);
    double s3 = sin(yaw);
	
	Matx44d transformationMatrix ;
	
	transformationMatrix(0,0) = c3*c2;
	transformationMatrix(0,1) = c3*s2*s1-s3*c1;
	transformationMatrix(0,2) = c3*s2*c1+s3*s1;

	transformationMatrix(1,0) = s3*c2;
	transformationMatrix(1,1) = s3*s2*s1+c3*c1;
	transformationMatrix(1,2) = s3*s2*c1-c3*s1;

	transformationMatrix(2,0) = -s2;
	transformationMatrix(2,1) = c2*s1;
	transformationMatrix(2,2) = c2*c1;


	transformationMatrix(0,3) = position.x;
	transformationMatrix(1,3) = position.y;
	transformationMatrix(2,3) = position.z;

	transformationMatrix(3,3) = 1.0;
	transformationMatrix(3,0) = 0.0;
	transformationMatrix(3,1) = 0.0;
	transformationMatrix(3,2) = 0.0;

	return transformationMatrix;

	

}

Mat RPYWorld2ResidualAngImu(Point3d rpy)
{
 	double roll = rpy.x;
    double pitch = rpy.y;
    double yaw = rpy.z;

    //cout << "roll = " << roll*180/M_PI << "pitch" << pitch*180/M_PI << "yaw" << yaw*180/M_PI <<endl;
    double c1 = cos(roll);
    double s1 = sin(roll);
    double c2 = cos(pitch);
    double s2 = sin(pitch);
	
	Mat rotationMatrix = Mat::zeros(3,3,CV_32FC1);


	rotationMatrix.at<float>(0,0) = 1.0;
	rotationMatrix.at<float>(0,1) = s2*s1/c1;
	rotationMatrix.at<float>(0,2) = c2*s1/c1;

	rotationMatrix.at<float>(1,0) = 0.0;
	rotationMatrix.at<float>(1,1) = c2;
	rotationMatrix.at<float>(1,2) = -s2;

	rotationMatrix.at<float>(2,0) = 0.0;
	rotationMatrix.at<float>(2,1) = s2/c1;
	rotationMatrix.at<float>(2,2) = c2/c1;

	return rotationMatrix;
}
