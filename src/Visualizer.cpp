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