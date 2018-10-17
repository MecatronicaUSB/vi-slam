#include "../include/Imu.hpp"
#include <cmath>
Imu::Imu(double timestep)
{
    timeStep = timestep;
}

void Imu::setImuData(vector <Point3d> &w_measure, vector <Point3d>  &a_measure)
{
    angularVelocityMeasure.reserve(w_measure.size());
    accelerationMeasure.reserve(a_measure.size());
    copy(w_measure.begin(), w_measure.end(), back_inserter(angularVelocityMeasure));
    copy(a_measure.begin(), a_measure.end(), back_inserter(accelerationMeasure));
    n = angularVelocityMeasure.size();
}


void Imu::computeVelocity()
{
    velocity.x = 0; // velocidad promedio
    velocity.y = 0;
    velocity.z = 0;
    for(int i= 0; i<n;i++)
    {
        velocity.x= velocity.x + (accelerationMeasure[i].x -imuGravity.x-accBias.x)*timeStep;// Se asume velocidad inicial 0
        velocity.y= velocity.y + (accelerationMeasure[i].y -imuGravity.y-accBias.y)*timeStep;
        velocity.z= velocity.z + (accelerationMeasure[i].z -imuGravity.z-accBias.z)*timeStep;
    }
  
}

void Imu::computePosition()
{
    position.x = velocity.x*n*timeStep;//n mediciones despues del frame1
    position.y = velocity.y*n*timeStep;// mas la medicion inicial asumida 0
    position.z = velocity.x*n*timeStep;
}

void Imu::computeAngularVelocity()
{
    angularVelocity.x = 0.0;
    angularVelocity.y = 0.0;
    angularVelocity.z = 0.0;
    for(int i= 0; i<n;i++)
    {
        angularVelocity.x = angularVelocity.x +angularVelocityMeasure[i].x;
        angularVelocity.y = angularVelocity.y +angularVelocityMeasure[i].y;
        angularVelocity.z = angularVelocity.z +angularVelocityMeasure[i].z;
    }
    angularVelocity.x = angularVelocity.x/n;
    angularVelocity.y = angularVelocity.y/n;
    angularVelocity.z = angularVelocity.z/n;
}

void Imu::computeAngularPosition() // rad/s
{
    angularPosition.x = angularVelocity.x*timeStep*n;
    angularPosition.y = angularVelocity.x*timeStep*n;
    angularPosition.z = angularVelocity.x*timeStep*n;
    quaternion = toQuaternion(angularPosition.y, angularPosition.x, angularPosition.z);
    

}

void Imu::computeAcceleration() // Implementar la estimación del bias y ruido
{
    acceleration.x = 0.0;
    acceleration.y = 0.0;
    acceleration.z = 0.0;
    for(int i= 0; i<n;i++)
    {
        acceleration.x = acceleration.x +accelerationMeasure[i].x;
        acceleration.y = acceleration.y +accelerationMeasure[i].y;
        acceleration.z = acceleration.z +accelerationMeasure[i].z;
    }
    acceleration.x = acceleration.x/n - imuGravity.x;
    acceleration.y = acceleration.y/n - imuGravity.y;
    acceleration.z = acceleration.z/n - imuGravity.z;
}

void Imu::estimate()
{
    computeAcceleration();
    computeVelocity();
    computePosition();
    computeAngularVelocity();
    computeAngularPosition();
}