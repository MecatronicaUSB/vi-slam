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

