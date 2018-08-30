#include "../include/Visualizer.h"

Visualizer::Visualizer(string markerN, string headerID, double rate, uint32_t shapeForm)
{
    createROSMarker(markerN, headerID, rate, shapeForm);

}

void Visualizer::createROSMarker(string markerN, string headerID, double rate, uint32_t shapeForm)
{
    markerName = markerN;
    headerFrameID = headerID;
    rateHZ = rate;
    shape = shapeForm;
    //
    ros::NodeHandle n;
    publisher = n.advertise<visualization_msgs::Marker>(markerName.c_str(), 1);
    marker.header.frame_id = headerFrameID.c_str();
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
    marker.id = 0;


}

void Visualizer::UpdateMessages(double position[3], double orientation[4]){
    
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


string Visualizer::getMarkerName(){
    return markerName;
}
string Visualizer::getHeaderFrameID(){
    return headerFrameID;
}
double Visualizer::getRateHZ(){
    return rateHZ;
}

