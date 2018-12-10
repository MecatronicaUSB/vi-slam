// Opencv libraries
#include "opencv2/core.hpp"
#include "../include/Plus.hpp"

// ROS libraries
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Vector3.h>


// vi_slam librarie
#include "Plus.hpp"
// C++ libraries
using namespace std;
using namespace cv;



  enum { // Shapes
    ARROW = 0u,
    CUBE = 1u,
    SPHERE = 2u,
    CYLINDER = 3u};
    
class VisualizerMarker
{
    public:
        VisualizerMarker(string marker, string headerID, double rate, uint32_t shape, int32_t ID, Point3f _scale, Point3f _color);
        void createROSMarker(string markerN, string headerID, double rate, uint32_t shape, int32_t ID, Point3f _scale, Point3f _color);
        void UpdateMessages(Point3d position, Quaterniond orientation);
        string getMarkerName();
        string getHeaderFrameID();
        double getRateHZ();
       

    private:
        string markerName;
        int32_t markerID;
        string headerFrameID;
        uint32_t shape;
        double rateHZ;
        ros::Publisher publisher; 
        visualization_msgs::Marker marker;

};

class VisualizerVector3{
    public:
        VisualizerVector3(string marker, double rate);
        void createROSMessage(string markerN, double rate);
        void UpdateMessages(Point3f data);
        string getMarkerName();
        double getRateHZ();
       

    private:
        string markerName;
        double rateHZ;
        ros::Publisher publisher; 
        geometry_msgs::Vector3 vectorData;

};

class VisualizerFrame
{
    public:
        VisualizerFrame(string marker, double rate);
        void createROSMarker(string markerN, double rate);
        void UpdateMessages(Mat frame);
        string getMarkerName();
        double getRateHZ();
       

    private:
        string markerName;
        double rateHZ;
        image_transport::Publisher publisher; 
        sensor_msgs::ImagePtr marker;

};



