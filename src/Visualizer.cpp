#include "../include/Visualizer.hpp"


// Visualizer Marker
VisualizerMarker::VisualizerMarker(string markerN, string headerID, double rate, uint32_t shapeForm, int32_t ID, Point3f scale, Point3f color)
{
    createROSMarker(markerN, headerID, rate, shapeForm, ID, scale, color);

}

void VisualizerMarker::createROSMarker(string markerN, string headerID, double rate, 
uint32_t shapeForm, int32_t ID, Point3f scale, Point3f color)
{
    markerName = markerN;
    headerFrameID = headerID;
    markerID = ID;
    rateHZ = rate;
    shape = shapeForm;
    
    //
    ros::NodeHandle n;
    publisher = n.advertise<visualization_msgs::Marker>(markerName.c_str(), 1);
    marker.header.frame_id = headerFrameID.c_str();
    marker.id = markerID;
    marker.header.stamp = ros::Time::now();
    marker.ns = "vi_slam";
    marker.type = shape;   
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration();

     // Ajustes
    marker.scale.x = scale.x;
    marker.scale.y = scale.y;
    marker.scale.z = scale.z;

    marker.color.r = color.x;
    marker.color.g = color.y;
    marker.color.b = color.z;
    marker.color.a = 1.0;

    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 0.0;
    


}

void VisualizerMarker::UpdateMessages(double position[3], double orientation[4]){
    
    ros::Rate r(rateHZ);

    marker.pose.position.x = position[0];
    marker.pose.position.y = position[1];
    marker.pose.position.z = position[2];
    if (orientation[0] != 0.0 && orientation[1] != 0.0 
    && orientation[2]!= 0.0 && orientation[3]!= 0.0 ){
        marker.pose.orientation.x = orientation[0];
        marker.pose.orientation.y = orientation[1];
        marker.pose.orientation.z = orientation[2];
        marker.pose.orientation.w = orientation[3];
    }
    else{
        marker.pose.orientation.w = 0.0;
        cout << "Orientation error! Invalid Quaternion"<<endl;
    }

    while (publisher.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        exit(1);
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    publisher.publish(marker);
    r.sleep();
}


string VisualizerMarker::getMarkerName(){
    return markerName;
}
string VisualizerMarker::getHeaderFrameID(){
    return headerFrameID;
}
double VisualizerMarker::getRateHZ(){
    return rateHZ;
}



// Visualizer Frame Class
VisualizerFrame::VisualizerFrame(string markerN, double rate)
{
    createROSMarker(markerN, rate);

}

void VisualizerFrame::createROSMarker(string markerN, double rate)
{
    markerName = markerN;
    rateHZ = rate;
    //
    ros::NodeHandle n;
    image_transport::ImageTransport node_current_frame(n);
    publisher = node_current_frame.advertise(markerName.c_str(), 50);

}

void VisualizerFrame::UpdateMessages(Mat frame)
{
    ros::Rate r(rateHZ);
    marker= cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
    while (publisher.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        exit(1);
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    publisher.publish(marker);
    r.sleep();

}
string VisualizerFrame::getMarkerName(){
    return markerName;
}

double VisualizerFrame::getRateHZ(){
    return rateHZ;
}

// Visualizer Vector
VisualizerVector3::VisualizerVector3(string markerN, double rate)
{
    createROSMessage(markerN, rate);

}

void VisualizerVector3::createROSMessage(string markerN, double rate)
{
    markerName = markerN;
    rateHZ = rate;
    
    ros::NodeHandle n;
    publisher = n.advertise< geometry_msgs::Vector3 >(markerName.c_str(), 1000);
}

void VisualizerVector3::UpdateMessages(Point3f data){
    ros::Rate r(rateHZ);

    vectorData.x = data.x;
    vectorData.y = data.y;
    vectorData.z = data.z;
    while (publisher.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        exit(1);
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker Vector3");
      sleep(1);
    }
    publisher.publish(vectorData);
    r.sleep();

    
}

string VisualizerVector3::getMarkerName(){
    return markerName;
}

double VisualizerVector3::getRateHZ(){
    return rateHZ;
}

ImuFilter::ImuFilter(double rate)
{
    createROSPublisher(rate);
}
void ImuFilter::createROSPublisher(double rate)
{
    rateHZ = rate;
    
    ros::NodeHandle n;
    publisher = n.advertise< sensor_msgs::Imu >("imu/data_raw", 1000);
}


void ImuFilter::createROSSubscriber()
{
    ros::NodeHandle nh;
    int queue_size = 15;
    imu_subscriber_.reset(new ImuSubscriber(
    nh, ros::names::resolve("imu") + "/data", queue_size));
    imu_subscriber_->registerCallback(&ImuFilter::imuCallback, this);
    timeNs = 0;
    timeS = 0;
}

void ImuFilter::imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg_raw)
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

void ImuFilter::UpdatePublisher( Point3d w_measure, Point3d a_measure)
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

void ImuFilter::UpdateSubscriber()
{
    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
}

string ImuFilter::getNodeName(){
    return "ImuFilter" ;
}

double ImuFilter::getRateHZ(){
    return rateHZ;
}

