#include "../include/Imu.hpp"
#include <cmath>

Imu::Imu(double timestep)
{
    timeStep = timestep;
    // Crear el publicador con una frecuencia 20 veces mayor a la respuesta de la imu
    createROSPublisher(static_cast<int>(1.0/(timeStep*20))); 
    createROSSubscriber();

}

void Imu::setImuData(vector <Point3d> &w_measure, vector <Point3d>  &a_measure)
{
    angularVelocityMeasure.reserve(w_measure.size());
    accelerationMeasure.reserve(a_measure.size());
    copy(w_measure.begin(), w_measure.end(), back_inserter(angularVelocityMeasure));
    copy(a_measure.begin(), a_measure.end(), back_inserter(accelerationMeasure));
    n = angularVelocityMeasure.size();
}

void Imu::setImuBias(Point3d acc_bias, Point3d ang_bias)
{
    accBias = acc_bias;
    angBias = ang_bias;
}


void Imu::computeGravity() // Calcula la direccion de la gravedad respecto a la imu
{  
    imuFilterGravity.clear();
    Quaterniond orientation;
    Point3d rpyAngles;
    Point3d gravity_imu;
    for (int i = 0; i < n ; i++)
    {
        UpdatePublisher( imuAngularVelocity[i], imuAcceleration[i]); //
        UpdateSubscriber();
        orientation.x = imuFusedData.orientation.x;
        orientation.y = imuFusedData.orientation.y;
        orientation.z = imuFusedData.orientation.z;
        orientation.w = imuFusedData.orientation.w;
        rpyAngles = toRPY(orientation);
        gravity_imu.x = -sin(rpyAngles.y)*9.68; // roll
        gravity_imu.y = sin(rpyAngles.x)*cos(rpyAngles.y)*9.68; // roll, pitch
        gravity_imu.z = cos(rpyAngles.x)*cos(rpyAngles.y)*9.68;
        imuFilterGravity.push_back(gravity_imu);
        
    }
}
void Imu::computeAcceleration() // Implementar la estimación del bias y ruido
{
    acceleration.x = 0.0;
    acceleration.y = 0.0;
    acceleration.z = 0.0;
    for(int i= 0; i<n;i++)
    {
        acceleration.x = acceleration.x +accelerationMeasure[i].x - imuFilterGravity[i].x;
        acceleration.y = acceleration.y +accelerationMeasure[i].y - imuFilterGravity[i].y;
        acceleration.z = acceleration.z +accelerationMeasure[i].z - imuFilterGravity[i].z;
    }
    acceleration.x = acceleration.x/n ; // Aceleracion promedio entre dos frames
    acceleration.y = acceleration.y/n ;
    acceleration.z = acceleration.z/n ;
}

void Imu::computeVelocity()
{
    velocity.x = 0; // velocidad promedio
    velocity.y = 0;
    velocity.z = 0;
    for(int i= 0; i<n;i++)
    {
        velocity.x= velocity.x + (accelerationMeasure[i].x -imuFilterGravity[i].x-accBias.x)*timeStep;// Se asume velocidad inicial 0
        velocity.y= velocity.y + (accelerationMeasure[i].y -imuFilterGravity[i].y-accBias.y)*timeStep;
        velocity.z= velocity.z + (accelerationMeasure[i].z -imuFilterGravity[i].z-accBias.z)*timeStep;
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


void Imu::estimate()
{   
    computeGravity();
    computeAcceleration();
    computeVelocity();
    computePosition();
    computeAngularVelocity();
    computeAngularPosition();
}

ImuFilterNode::ImuFilterNode()
{

}
ImuFilterNode::ImuFilterNode(int rate)
{
    createROSPublisher(rate);
}
void ImuFilterNode::createROSPublisher(int rate)
{
    rateHZ = rate;
    
    ros::NodeHandle n;
    publisher = n.advertise< sensor_msgs::Imu >("imu/data_raw", rateHZ);
}


void ImuFilterNode::createROSSubscriber()
{
    ros::NodeHandle nh;
    int queue_size = 15;
    imu_subscriber_.reset(new ImuSubscriber(
    nh, ros::names::resolve("imu") + "/data", queue_size));
    imu_subscriber_->registerCallback(&ImuFilter::imuCallback, this);
    timeNs = 0;
    timeS = 0;
}

void ImuFilterNode::imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg_raw)
{
    imuFusedData.angular_velocity = imu_msg_raw->angular_velocity;
    imuFusedData.linear_acceleration = imu_msg_raw->linear_acceleration;
    imuFusedData.orientation = imu_msg_raw->orientation;
    /*cout<<"\nReceived : qx=" <<imu_msg_raw->orientation.x
    <<" qy="<<imu_msg_raw->orientation.y
    <<" qz="<<imu_msg_raw->orientation.z
    <<" qw="<<imu_msg_raw->orientation.w
    <<endl;
    */
    


}

void ImuFilterNode::UpdatePublisher( Point3d w_measure, Point3d a_measure)
{
    
    timeNs = timeNs+5000;
    ros::Rate r(rateHZ);
    message.orientation.x = 0.0;
    message.orientation.y = 0.0;
    message.orientation.z = 0.0;
    message.orientation.w = 1.0;
    message.orientation_covariance[0] = -1.0;
    message.angular_velocity.x = w_measure.x;
    message.angular_velocity.y = w_measure.y;
    message.angular_velocity.z = w_measure.z;
    message.linear_acceleration.x = a_measure.x;
    message.linear_acceleration.y = a_measure.y;
    message.linear_acceleration.z = a_measure.z;
    message.header.frame_id = "/imu";
    message.header.stamp.nsec =timeNs;
    message.header.stamp.sec = static_cast <uint32_t> (floor(timeNs/1000000.0));//ros::Time::now().toSec();
    while (publisher.getNumSubscribers() < 1)
    {
        if (!ros::ok())
        {
            exit(1);
        }
        else{
            ROS_WARN_ONCE("Please create a subscriber to Imu data");
            sleep(1);
        }
    }
    /*cout<<" ax = "<< message.linear_acceleration.x 
    <<"ay = "<< message.linear_acceleration.y 
    <<"az = "<< message.linear_acceleration.z
    <<endl;
    */
    
    publisher.publish(message);
    r.sleep();

    
   
}

void ImuFilterNode::UpdateSubscriber()
{
    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
}

string ImuFilterNode::getNodeName(){
    return "ImuFilter" ;
}

double ImuFilterNode::getRateHZ(){
    return rateHZ;
}

