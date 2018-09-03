// Opencv libraries
#include "opencv2/core.hpp"

// ROS libraries
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

// C++ libraries
using namespace std;
using namespace cv;

class VisualizerPoint
{
    public:
        VisualizerPoint(string marker, string headerID, double rate, uint32_t shape, int32_t ID);
        void createROSMarker(string markerN, string headerID, double rate, uint32_t shape, int32_t ID);
        void UpdateMessages(double position[3], double orientation[4]);
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