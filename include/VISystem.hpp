#include <iostream>
#include <fstream>
#include "Camera.hpp"
#include "Imu.hpp"
#include "Plus.hpp"
#include "CameraModel.hpp"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/core/eigen.hpp"
#include <ctime>

#include <ros/ros.h>
#include <ros/console.h>

#include <std_msgs/Int32.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>


using namespace std;
using namespace cv;

#define PYRAMID_LEVELS 5

namespace vi
{
class CameraModel;
class VISystem
{
    public:
        VISystem();
        VISystem(int argc, char *argv[]);
        ~VISystem();
        
        
        void InitializeSystem(string _calPath, Point3d _iniPosition, Point3d _iniVelocity, Point3d _iniRPY, Mat image);
        void InitializePyramid(int _width, int _height, Mat _K);
        // Gauss-Newton using Foward Compositional Algorithm - Using features
        void Calibration(string _calibration_path);
        void EstimatePoseFeatures(Frame* _previous_frame, Frame* _current_frame);
        void CalculateROI(Mat image);
        void AddFrame(Mat _currentImage, vector <Point3d> _imuAngularVelocity, vector <Point3d> _imuAcceleration);
        void AddFrame(Mat _currentImage, vector <Point3d> _imuAngularVelocity, vector <Point3d> _imuAcceleration, vector <Point3d> _gtRPY) ;
        void FreeLastFrame();
        void Track();
        Mat IdentityWeights(int _num_residuals) ;
        Mat WarpFunction(Mat _points2warp, SE3 _rigid_transformation, int _lvl);
        
        bool initialized, distortion_valid , depth_available;
        int  num_keyframes;
        int  num_max_keyframes;
        int  min_features;
        int  start_index;
        
        Mat map1, map2;
        int h, w, h_input, w_input;
        float fx, fy, cx, cy;

        Point3d positionImu;
        Point3d velocityImu;
        Quaterniond qOrientationImu;
        Point3d RPYOrientationImu;
        Mat world2imuTransformation;
        Mat world2imuRotation;

        Point3d positionCam;
        Point3d velocityCam;
        Quaterniond qOrientationCam;
        Point3d RPYOrientationCam;
        Mat imu2camTransformation;
        Mat imu2camRotation;
        Point3d imu2camTranslation;

        SE3 final_poseCam;
        SE3 final_poseImu;
        SE3 current_poseCam;
        SE3 current_poseImu;

        CameraModel* camera_model;
        
        Camera camera;
        Imu imuCore;
        
        Mat K;
        Rect ROI;
        

        Mat outputCurrentImage, outputLastImage;

        // Width and height of images for each pyramid level available
        vector<int> w_ = vector<int>(PYRAMID_LEVELS);
        vector<int> h_ = vector<int>(PYRAMID_LEVELS);

        vector<float> fx_ = vector<float>(PYRAMID_LEVELS);
        vector<float> fy_ = vector<float>(PYRAMID_LEVELS);
        vector<float> cx_ = vector<float>(PYRAMID_LEVELS);
        vector<float> cy_ = vector<float>(PYRAMID_LEVELS);
        vector<float> invfx_ = vector<float>(PYRAMID_LEVELS);
        vector<float> invfy_ = vector<float>(PYRAMID_LEVELS);
        vector<float> invcx_ = vector<float>(PYRAMID_LEVELS);
        vector<float> invcy_ = vector<float>(PYRAMID_LEVELS);

        vector<Mat> K_ = vector<Mat>(PYRAMID_LEVELS);


   

        
    

};

}

