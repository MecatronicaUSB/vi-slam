// ROS libraries
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

// C++ libraries
using namespace std;

class Visualizer
{
    public:
        Visualizer(string marker, string headerID, double rate, uint32_t shape);
        void createROSMarker(string markerN, string headerID, double rate, uint32_t shape);
        void UpdateMessages(double position[3], double orientation[4]);
        string getMarkerName();
        string getHeaderFrameID();
        double getRateHZ();
       

    private:
        string markerName;
        string headerFrameID;
        uint32_t shape;
        double rateHZ;
        ros::Publisher publisher; 
        visualization_msgs::Marker marker;

};