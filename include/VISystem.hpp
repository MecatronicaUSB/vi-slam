#include <iostream>
#include <fstream>
#include <string> 
#include "Camera.hpp"
#include "Imu.hpp"
#include "Plus.hpp"
#include "CameraModel.hpp"
#include "Pmvs.hpp"
#include "opencv2/core.hpp"
//#include "opencv2/sfm.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"
//#include "opencv2/core/eigen.hpp"
#include <ctime>

#include <ros/ros.h>
#include <ros/console.h>

#include <std_msgs/Int32.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>



using namespace std;
using namespace cv;




namespace vi
{
class CameraModel;



// 3D point
struct Landmark
{
    cv::Point3f pt;
    int seen = 0; // how many cameras have seen this point

};

class VISystem
{
    public:
        VISystem();
        VISystem(int argc, char *argv[]);
        ~VISystem();
        
        
        // Funciones iniciales
        // Leer archivo de configuracion del sistema
        void setConfig(string _calPath); 
        void readConfig(string _calibration_path);
        void setPmvsBinary(string _pmvsBinary);
        void setOutputDirectory(string _outDirectory);
        // colocar la posicion inicial del sistema
        void setInitPose( Point3d _iniPosition, double _iniYaw);
        void setInitPose( Point3d _iniPosition, Quaterniond _imuQuaternion);
        // colocar data inicial
        void setInitData(Mat image,  vector <Point3d> _imuAngularVelocity, vector <Point3d> _imuAcceleration);
        // colocar parametros de configuracion a la cámara
        void setCameraConfig(int _detector, int _matcher, int _w_size, int _h_size, int _num_cells);


        // Añadir frame
        bool AddFrame(Mat _currentImage, vector <Point3d> _imuAngularVelocity, vector <Point3d> _imuAcceleration);

        // estimar pose actual
        void Track();
        // estimar vector de traslacion
        void EstimateTranslationVector(Frame* _previous_frame, Frame* _current_frame);
        // estimar escala del vector de traslacion
        void EstimatePoseFeaturesDebug(Frame* _previous_frame, Frame* _current_frame);
        void EstimatePoseFeaturesRansac(Frame* _previous_frame, Frame* _current_frame);
        void CalculateROI(Mat image);
        
        
        
        void FreeLastFrame();
        
       
        Point3f F2FRansac(vector <KeyPoint> inPoints1, vector <KeyPoint> inPoints2, Matx33f rotationMat, float threshold = 350);
        Point3f BestRansac(vector <KeyPoint> inPoints1, vector <KeyPoint> inPoints2, Matx33f rotationMat);
        float computeErrorGroup(vector <KeyPoint> inPoints1, vector <KeyPoint> inPoints2);
        void FilterKeypoints(double threshold, vector<bool> &mask, vector <KeyPoint> &outPoints1, vector <KeyPoint> &outPoints2,vector <KeyPoint> &outlier1, vector <KeyPoint> &outlier2);
        void WarpFunctionRT(vector <KeyPoint> inPoints, Mat rotationMat, Mat translationMat, vector <KeyPoint> &outPoints);
        
        
        float Disparity(vector <KeyPoint> keyPoints, vector <KeyPoint> inPoints );
        float Euclidean(vector <KeyPoint> keyPoints, vector <KeyPoint> inPoints );
        void Triangulate(vector <KeyPoint> inPoints1, vector <KeyPoint> inPoints2, Matx33f prevR, Point3f prevt,  Matx33f currR, Point3f currt, vector <Point3f> &points3D);
        

        void setGtTras(Point3d TranslationResGT );
        void setGtRot(Matx33f R_out );
        void saveFrame();
        Mat getProjectionMat(Mat cameraMat, Mat rotationMat, Mat translationMat);
        void addNewLandmarks( vector <Point3f> points3D, vector <bool> mask);

        void getLandmarks(vector<Point3f> &landmarks);
        
        float computeScale(vector<Point3f> points3D, vector <bool> mask);

        void shutdown();
        
        
        int  num_keyframes;
        int  num_max_keyframes;
        int  min_features;
        int  start_index;
        
        Mat map1, map2;
        int h, w, h_input, w_input;
        float fx, fy, cx, cy;

        // IMU
        Point3d positionImu;
        Point3d velocityImu;
        Point3d accImu;
        Quaterniond qOrientationImu;
        Point3d RPYOrientationImu;
        double iniYaw;

        // Cámara
        Point3d positionCam;
        Point3d velocityCam;
        Point3d accCam;
        Quaterniond qOrientationCam;
        Point3d RPYOrientationCam;

        int numSteadyImages;
       
        


        Mat imu2camTransformation;
        Matx33f imu2camRotation;
        Point3d imu2camTranslation;

        Matx44d final_poseCam;
        Matx44d final_poseImu;
        Matx44d current_poseCam;
        Matx44d current_poseImu;
        Matx33f rotation_Cam; // Matrix de rotacion de la cámara
        Matx34d projectionMatrix;
   


        CameraModel* camera_model;
        Pmvs pmvs;
        
        Camera camera;
        Imu imuCore;

        

        Mat currentImage;
        Mat currentImageToShow;
        Mat currentImageDebugToShow;
        Mat K;
        Rect ROI;
        

        
        
        Point3d TranslationResidualGT;
        Matx33f RotationResGT;
        
        Matx33f RotationResCam; // residual de rotacion de camara
        Matx33f RotationResImu; // residual de rotacion de la imu
        Matx33f init_rotationMatrix, final_rotationMatrix;
    
        Point3f translationResEst;
        int nPointsLastKeyframe;
        int nPointsCurrentImage;
        bool lastImageWasKeyframe;
        bool currentImageIsKeyframe;

        // Landmarks

        vector<Landmark> landmarks;

        // Trayectoria

        vector<Point3f> MapPose; // Poses del robot entre keyframes
        vector<Matx33f> MapOrientation; // Poses de la orientacion


        // lista de keyframes
        vector <Frame *> keyFrameList;  // lista de todos los frames
        Frame * currentFrame;
        float scale, scaleGt;



   

        
    

};

}

