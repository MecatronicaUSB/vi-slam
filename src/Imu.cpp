#include "../include/Imu.hpp"
#include <cmath>
#include <iomanip>
Imu::Imu(double timestep)
{
    timeStep = timestep;
    // Crear el publicador con una frecuencia 10 veces mayor a la respuesta de la imu
    createROSPublisher(static_cast<int>((1.0/timeStep)*20)); 
    createROSSubscriber();

}

void Imu::setImuData(vector <Point3d> &w_measure, vector <Point3d>  &a_measure)
{
    angularVelocityMeasure.clear();
    accelerationMeasure.clear();
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



void Imu::initializate(double gt_yaw)
{
    initialYawGt = gt_yaw;
    Quaterniond orientation;
    for (int i = 0; i < n ; i++)
    {
        UpdatePublisher( angularVelocityMeasure[i], accelerationMeasure[i]); //
        UpdateSubscriber();
    }
    orientation.x = imuFusedData.orientation.x;
    orientation.y = imuFusedData.orientation.y;
    orientation.z = imuFusedData.orientation.z;
    orientation.w = imuFusedData.orientation.w;
    Point3d angle_filter_initial = toRPY(orientation);
    
    initialYawFilter =  angle_filter_initial.z;  // ultima estimacion angulo inicial
        

}


void Imu::computeAcceleration() // Implementar la estimación del bias y ruido
{
    accelerationWorld.clear();
    rpyAnglesWorld.clear();
    quaternionWorld.clear();
    angularVelocityIMUFilter.clear();

    Quaterniond orientation;
    
    Point3d gravity_imu;
    elapsed_filter = 0.0;
    for (int i = 0; i < n ; i++)
    {
        clock_t t1 = clock(); 
        UpdatePublisher( angularVelocityMeasure[i], accelerationMeasure[i]); //
        UpdateSubscriber();
        clock_t t2 = clock(); 
        elapsed_filter= double(t2- t1) / CLOCKS_PER_SEC +elapsed_filter;
        orientation.x = imuFusedData.orientation.x;
        orientation.y = imuFusedData.orientation.y;
        orientation.z = imuFusedData.orientation.z;
        orientation.w = imuFusedData.orientation.w;
        rpyAnglesWorld.push_back(toRPY(orientation));
        // Alineacion del angulo yaw gt inicial
        rpyAnglesWorld[i].z = rpyAnglesWorld[i].z - initialYawFilter+initialYawGt ;
        orientation = toQuaternion(rpyAnglesWorld[i].x, rpyAnglesWorld[i].y, rpyAnglesWorld[i].z );
        Point3d accWorld;
        // restar bias local
        //accelerationMeasure[i].x = accelerationMeasure[i].x; //-accBias.x;
        //accelerationMeasure[i].y = accelerationMeasure[i].y;// -accBias.y;
        //accelerationMeasure[i].z = accelerationMeasure[i].z;// -accBias.z;
        // transformar acelerarion al sistema fijo (world)
        accWorld = transform2World(accelerationMeasure[i], rpyAnglesWorld[i]);
        accWorld.z = accWorld.z - 9.68; // restar la aceleracion de gravedad
        accelerationWorld.push_back(accWorld);
        quaternionWorld.push_back(orientation);
        // Guardar la velocidad angular filtrada
        Point3d angularw;
        angularw.x  = imuFusedData.angular_velocity.x;
        angularw.y  = imuFusedData.angular_velocity.y;
        angularw.z  = imuFusedData.angular_velocity.z;
        angularVelocityIMUFilter.push_back(angularw);
    
    }
    elapsed_filter = elapsed_filter/n;

}


void Imu::computeAcceleration(vector <Point3d> gtRPY) // Implementar la estimación del bias y ruido
{
    accelerationWorld.clear();
    rpyAnglesWorld.clear();
    quaternionWorld.clear();
    angularVelocityIMUFilter.clear();

    Quaterniond orientation;
    
    Point3d gravity_imu;
    elapsed_filter = 0.0;
    int size = gtRPY.size();
    if (size > 10) size = 10;
    for (int i = 0; i < size ; i++)
    {
        clock_t t1 = clock(); 
        UpdatePublisher( angularVelocityMeasure[i], accelerationMeasure[i]); //
        UpdateSubscriber();
        clock_t t2 = clock(); 
        elapsed_filter= double(t2- t1) / CLOCKS_PER_SEC +elapsed_filter;
        orientation.x = imuFusedData.orientation.x;
        orientation.y = imuFusedData.orientation.y;
        orientation.z = imuFusedData.orientation.z;
        orientation.w = imuFusedData.orientation.w;
        Point3d angles = toRPY(orientation);
        //angles.x = gtRPY[i].x;
        //angles.y = gtRPY[i].y;
        double aux_angz = angles.z;
        angles.z = gtRPY[i].z;
        rpyAnglesWorld.push_back(angles);
        // Alineacion del angulo yaw gt inicial

        orientation = toQuaternion(rpyAnglesWorld[i].x, rpyAnglesWorld[i].y, rpyAnglesWorld[i].z );
        Point3d accWorld;
        // restar bias local
        accelerationMeasure[i].x = accelerationMeasure[i].x ;//-accBias.x;
        accelerationMeasure[i].y = accelerationMeasure[i].y ;//-accBias.y;
        accelerationMeasure[i].z = accelerationMeasure[i].z ;//-accBias.z;
        // transformar acelerarion al sistema fijo (world)
        accWorld = transform2World(accelerationMeasure[i], rpyAnglesWorld[i]);
        accWorld.z = accWorld.z-9.68; // restar la aceleracion de gravedad
        rpyAnglesWorld[i].z = aux_angz;
        accelerationWorld.push_back(accWorld);
        quaternionWorld.push_back(orientation);
        // Guardar la velocidad angular filtrada
        Point3d angularw;
        angularw.x  = imuFusedData.angular_velocity.x;
        angularw.y  = imuFusedData.angular_velocity.y;
        angularw.z  = imuFusedData.angular_velocity.z;
        angularVelocityIMUFilter.push_back(angularw);
    
    }
    elapsed_filter = elapsed_filter/n;

}

void Imu::computeVelocity()
{
    velocity.x = 0; // velocidad promedio
    velocity.y = 0;
    velocity.z = 0;
    for(int i= 0; i<n;i++)
    {
        velocity.x= velocity.x + (accelerationWorld[i].x)*timeStep;// Se asume velocidad inicial 0
        velocity.y= velocity.y + (accelerationWorld[i].y)*timeStep;
        velocity.z= velocity.z + (accelerationWorld[i].z)*timeStep;
    }
  
}

void Imu::computePosition()
{
   
    Point3d translationBetweenFrames;
    for(int i= 0; i<n;i++)
    {
        translationBetweenFrames.x = 0.5*(accelerationWorld[i].x)*timeStep*timeStep;
        translationBetweenFrames.y = 0.5*(accelerationWorld[i].y)*timeStep*timeStep;
        translationBetweenFrames.z = 0.5*(accelerationWorld[i].z)*timeStep*timeStep;
    }
    position.x = initialVelocity.x*n*timeStep+translationBetweenFrames.x;//n mediciones despues del frame1
    position.y = initialVelocity.y*n*timeStep+translationBetweenFrames.y;// mas la medicion inicial asumida 0
    position.z = initialVelocity.z*n*timeStep+translationBetweenFrames.z;// revisar n

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
    //quaternion = toQuaternion(angularPosition.y, angularPosition.x, angularPosition.z);
    

}


void Imu::estimate()
{   
    computeAcceleration();
    computeVelocity();
    computePosition();
    computeAngularVelocity();
    computeAngularPosition();
    residualRPY = rpyAnglesWorld[quaternionWorld.size()-1]-rpyAnglesWorld[0];
    residualVelocity =  velocity;
    residualPosition = position;

}

void Imu::estimate(vector <Point3d> gtRPY)
{   
    computeAcceleration(gtRPY);
    computeVelocity();
    computePosition();
    computeAngularVelocity();
    computeAngularPosition();
    residualRPY = rpyAnglesWorld[quaternionWorld.size()-1]-rpyAnglesWorld[0];
    residualVelocity = velocity;
    residualPosition = position;
    /*initialVelocity.x = velocity.x;
    initialVelocity.y = velocity.y;
    initialVelocity.z = velocity.z;
    */
}

void Imu::setImuInitialPosition()
{

}


void Imu::setImuInitialVelocity(Point3d initial_velocity)
{
    initialVelocity = initial_velocity;


}


void Imu::printStatistics()
{
     cout<<"\nESTADISTICAS IMU"
     <<"\nTiempo de filtrado : " << fixed<< setprecision(3) << elapsed_filter*1000<< " ms" <<endl;
}

// Imu filter node
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
    imu_subscriber_->registerCallback(&ImuFilterNode::imuCallback, this);
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

Point3d Imu::transform2World(Point3d acc, Point3d angl)
{
    Point3d worldVector;
    double roll = angl.x;
    double pitch = angl.y;
    double yaw = angl.z;

    //cout << "roll = " << roll*180/M_PI << "pitch" << pitch*180/M_PI << "yaw" << yaw*180/M_PI <<endl;
    double c1 = cos(roll);
    double s1 = sin(roll);
    double c2 = cos(pitch);
    double s2 = sin(pitch);
    double c3 = cos(yaw);
    double s3 = sin(yaw);
    worldVector.x = c3*c2*acc.x + (c3*s2*s1-s3*c1)*acc.y+(c3*s2*c1+s3*s1)*acc.z;
    worldVector.y = s3*c2*acc.x + (s3*s2*s1+c3*c1)*acc.y +(s3*s2*c1-c3*s1)*acc.z;
    worldVector.z = -s2*acc.x +c2*s1*acc.y +c2*c1*acc.z;

    return worldVector;
}
