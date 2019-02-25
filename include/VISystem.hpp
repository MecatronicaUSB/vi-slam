#include <iostream>
#include <fstream>
#include <string> 
#include "Camera.hpp"
#include "Imu.hpp"
#include "Plus.hpp"
#include "CameraModel.hpp"
#include "opencv2/core.hpp"
#include "opencv2/sfm.hpp"
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
        
        void InitializeCamera(int _detector, int _matcher, int _w_size, int _h_size, int _num_cells, int _length_path);
        void InitializeSystem(string _calPath, Point3d _iniPosition, Point3d _iniVelocity, double _iniYaw, Mat image,  vector <Point3d> _imuAngularVelocity, vector <Point3d> _imuAcceleration );
        void InitializePyramid(int _width, int _height, Mat _K);
        // Gauss-Newton using Foward Compositional Algorithm - Using features
        void Calibration(string _calibration_path);
        void EstimatePoseFeaturesIter(Frame* _previous_frame, Frame* _current_frame);
        void EstimatePoseFeaturesDebug(Frame* _previous_frame, Frame* _current_frame);
        void EstimatePoseFeatures(Frame* _previous_frame, Frame* _current_frame);
        void EstimatePoseFeaturesRansac(Frame* _previous_frame, Frame* _current_frame);
        void CalculateROI(Mat image);
        bool AddFrame(Mat _currentImage, vector <Point3d> _imuAngularVelocity, vector <Point3d> _imuAcceleration);
        bool AddFrame(Mat _currentImage, vector <Point3d> _imuAngularVelocity, vector <Point3d> _imuAcceleration, Point3d _gtPosition) ;
        void ObtainKeypointsTransformation(Mat candidates);
        void MatPoint2Keypoints( Mat _MatPoints, vector<KeyPoint> &_outputKeypoints);
        void FreeLastFrame();
        void Track();
        Mat IdentityWeights(int _num_residuals) ;
        Mat TukeyFunctionWeights(Mat _input) ;
        Mat WarpFunctionSE3(Mat _points2warp, SE3 _rigid_transformation, int _lvl);
        Point3f F2FRansac(vector <KeyPoint> inPoints1, vector <KeyPoint> inPoints2, Matx33f rotationMat, double threshold = 350);
        void FilterKeypoints(vector <KeyPoint> inPoints1, vector <KeyPoint> inPoints2, vector <KeyPoint> &outPoints1, vector <KeyPoint> &outPoints2, double threshold);
        void WarpFunctionRT(vector <KeyPoint> inPoints, Mat rotationMat, Mat translationMat, vector <KeyPoint> &outPoints);
        float MedianAbsoluteDeviation(Mat _input);
        float MedianMat(Mat _input) ;
        float Disparity(vector <KeyPoint> keyPoints, vector <KeyPoint> inPoints );
        void Triangulate(vector <KeyPoint> inPoints1, vector <KeyPoint> inPoints2) ;
        

        void setGtRes(Mat TranslationResGT , Mat RotationGT);
        Mat getProjectionMat(Mat cameraMat, Mat rotationMat, Mat translationMat);
        
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
        Point3d accImu;
        Quaterniond qOrientationImu;
        Point3d RPYOrientationImu;
        Mat world2imuTransformation;
        Matx33f world2imuRotation;

        Point3d positionCam;
        Point3d velocityCam;
        Point3d accCam;
        Quaterniond qOrientationCam;
        Point3d RPYOrientationCam;
        Mat prev_world2camTransformation;
        


        Mat imu2camTransformation;
        Matx33f imu2camRotation;
        Point3d imu2camTranslation;

        SE3 final_poseCam;
        SE3 final_poseImu;
        SE3 current_poseCam;
        SE3 current_poseImu;
        Point3d prev_gtPosition;
        Point3d current_gtPosition;
        Point3d current_gtTraslation;

        CameraModel* camera_model;
        
        Camera camera;
        Imu imuCore;

        Mat currentImage;
        Mat currentImageToShow;
        Mat currentImageDebugToShow;
        Mat prevImage;
        Mat prevImageToShow;
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
        vector<double> Prof;
        Mat TranslationResidual;
        Matx33f RotationResidual;
        Matx33f RotationResCam; // residual de rotacion
        Matx33f init_rotationMatrix, final_rotationMatrix;
        Point3f translationResEst;
        int nPointsLastKeyframe;
        int nPointsCurrentImage;
        bool lastImageWasKeyframe;
        bool currentImageIsKeyframe;


   

        
    

};

}

