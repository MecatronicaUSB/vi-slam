#include <iostream>
#include <fstream>
#include "Camera.hpp"
#include "Imu.hpp"
#include "Plus.hpp"
#include "CameraModel.hpp"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"
#include <ctime>

#include <ros/ros.h>
#include <ros/console.h>

#include <std_msgs/Int32.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>


// Eigen library
#include <eigen3/Eigen/Core>

// Sophus
#include "sophus/se3.hpp"
#include "sophus/so3.hpp"


using namespace std;
using namespace cv;

namespace vi
{
class CameraModel;
class VISystem
{
    public:
        VISystem();
        VISystem(int argc, char *argv[]);
        ~VISystem();
        
        
        void InitializeSystem(string _outputPath, string _depthPath, string _calPath, Point3d _iniPosition, Point3d _iniVelocity, float _iniYaw, Mat image);
        // Gauss-Newton using Foward Compositional Algorithm - Using features
        void Calibration(string _calibration_path);
        void EstimatePoseFeatures(Frame* _previous_frame, Frame* _current_frame);
        void CalculateROI(Mat image);
        void AddFrame(Mat _currentImage, vector <Point3d> _imuAngularVelocity, vector <Point3d> _imuAcceleration);
        void AddFrame(Mat _currentImage, vector <Point3d> _imuAngularVelocity, vector <Point3d> _imuAcceleration, vector <Point3d> _gtRPY) ;
        void FreeLastFrame();
        
        bool initialized, distortion_valid , depth_available;
        int  num_keyframes;
        int  num_max_keyframes;
        int  min_features;
        int  start_index;
        
        Mat map1, map2;
        int h, w, h_input, w_input;
        float fx, fy, cx, cy;

        Point3d position;
        Point3d velocity;
        Quaterniond qOrientation;
        Point3d RPYOrientation;
        CameraModel* camera_model;
        
        Camera camera;
        Imu imuCore;
        
        Mat K;
        Rect ROI;

        std::ofstream outputFile;
        Mat outputCurrentImage, outputLastImage;

        
    

};

}

