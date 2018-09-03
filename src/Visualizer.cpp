#include "../include/Visualizer.hpp"


// Visualizer Point
VisualizerPoint::VisualizerPoint(string markerN, string headerID, double rate, uint32_t shapeForm, int32_t ID)
{
    createROSMarker(markerN, headerID, rate, shapeForm, ID);

}

void VisualizerPoint::createROSMarker(string markerN, string headerID, double rate, uint32_t shapeForm, int32_t ID)
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
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 0.0;
    


}

void VisualizerPoint::UpdateMessages(double position[3], double orientation[4]){
    
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


string VisualizerPoint::getMarkerName(){
    return markerName;
}
string VisualizerPoint::getHeaderFrameID(){
    return headerFrameID;
}
double VisualizerPoint::getRateHZ(){
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