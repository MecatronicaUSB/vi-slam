#include "../include/Plus.hpp"

Quaterniond toQuaternion(double pitch, double roll, double yaw)
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



Point3d toEulerAngle(const Quaterniond& q)
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

double computeDiffAng(double gt_angle, double gt_est)
{
	double diff;
	if ((gt_angle> 0.0) && (gt_est>0.0) ) diff = gt_angle-gt_est;
	if ((gt_angle< 0.0) && (gt_est<0.0) ) diff = -(gt_angle-gt_est);
	if ((gt_angle> 0.0) && (gt_est<0.0) ) diff = gt_angle+gt_est;
	if ((gt_angle< 0.0) && (gt_est>0.0) ) diff = -(gt_angle+gt_est);

	return abs(diff*180/M_PI);

}