#include "../include/Visualizer.hpp"

// Visualizer Pose


VisualizerPose::VisualizerPose(string fixedFrame, string name, double rate)
{
    createVisualizer(fixedFrame, name, rate);
}

void VisualizerPose::createVisualizer(string fixedFrame, string name, double rate)
{
    stamped.frame_id_ = fixedFrame.c_str();
    stamped.child_frame_id_ = name.c_str();
}

void VisualizerPose::UpdateMessages(Point3f position, Quaterniond qOrientation)
{

        tf::Quaternion q;
        q[0] = qOrientation.x;
        q[1] = qOrientation.y;
        q[2] = qOrientation.z;
        q[3] = qOrientation.w;
       
        stamped.stamp_ = ros::Time::now();
        stamped.setOrigin(tf::Vector3(position.x, position.y, position.z));
        stamped.setRotation(q);
        broadcaster.sendTransform(stamped);
}



// Visualizer Pointcloud


VisualizerPointcloud::VisualizerPointcloud(string frameID, string name, double rate)
{
   
    createPointcloudMessage(frameID, name, rate);
}


void VisualizerPointcloud::createPointcloudMessage(string frameID, string name, double rate)
{
    headerFrameID = frameID;
    rateHZ = rate;

    // Crear publisher
    ros::NodeHandle n;
    publisher = n.advertise<sensor_msgs::PointCloud>(name.c_str(), 1);
    pointcloud.header.frame_id = headerFrameID.c_str();
    seq = 0;

}


void VisualizerPointcloud::UpdateMessages(vector<Point3f> landmarks)
{
    ros::Rate r(rateHZ);
    seq++;
    pointcloud.header.seq = seq;
    pointcloud.header.stamp = ros::Time::now();
    pointcloud.points.clear();
    pointcloud.channels.clear();

    geometry_msgs::Point32 point;
    sensor_msgs::ChannelFloat32 channel;
    channel.name = "rgb";
    channel.values.push_back(255);
    //channel.values.push_back(0);
    //channel.values.push_back(0);
    for(int i = 0; i< landmarks.size(); i++)
    {
        point.x = landmarks[i].x;
        point.y = landmarks[i].y;
        point.z = landmarks[i].z;

        pointcloud.points.push_back(point);
        pointcloud.channels.push_back(channel);

    }
    cout << " size pointcloud " << pointcloud.points.size()<<endl;

     while (publisher.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        exit(1);
      }
      ROS_WARN_ONCE("Please create a subscriber to the Pointcloud message");
      sleep(1);
    }
    publisher.publish(pointcloud);





    r.sleep();
}
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

void VisualizerMarker::UpdateMessages(Point3d position, Quaterniond orientation){
    
    ros::Rate r(rateHZ);

    marker.pose.position.x = position.x;
    marker.pose.position.y = position.y;
    marker.pose.position.z = position.z;
    //if (orientation[0] != 0.0 && orientation[1] != 0.0 
    //&& orientation[2]!= 0.0 && orientation[3]!= 0.0 ){
    marker.pose.orientation.x = orientation.x;
    marker.pose.orientation.y = orientation.y;
    marker.pose.orientation.z = orientation.z;
    marker.pose.orientation.w = orientation.w;
   /* }
    else{
        marker.pose.orientation.w = 0.0;
        cout << "Orientation error! Invalid Quaternion"<<endl;
    }
    */
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

