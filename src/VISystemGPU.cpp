#include "../include/VISystemGPU.hpp"

namespace vi
{
   
    VISystemGPU::VISystemGPU()
    {
        initialized = false;
        distortion_valid = false;
        depth_available = false;
        num_keyframes = 0;
    }

    VISystemGPU::VISystemGPU(int argc, char *argv[])
    {
        ros::init(argc, argv, "vi_slam");  // Initialize ROS
        initialized = false;
        distortion_valid = false;
        depth_available = false;
        num_keyframes = 0;
    }

    VISystemGPU::~VISystemGPU() 
    {
        cout << "SLAM System shutdown ..." << endl;
        /*
        frames_.clear();
        keyframes_.clear();
        camera_model_->~CameraModel();
        tracker_->~Tracker();
        visualizer_->~Visualizer();

        delete camera_model_;
        delete tracker_;
        delete visualizer_;
        */
    }

    void VISystemGPU::InitializeSystemGPU(string _calPath, Point3d _iniPosition, Point3d _iniVelocity, Point3d _iniRPY, Mat image)
    {
        // Check if depth images are available
        /*if (_depth_path != "")
            depth_available = true;
        */
        Calibration(_calPath);
        
        // Obtain parameters of camera_model
        imu2camTransformation = (camera_model->imu2cam0Transformation);
        imu2camRotation = transformationMatrix2rotationMatrix(imu2camTransformation);
        imu2camTranslation = transformationMatrix2position(imu2camTransformation);

        K = camera_model->GetK();
        w_input = camera_model->GetInputWidth();
        h_input = camera_model->GetInputHeight();

        map1 = camera_model->GetMap1();
        map2 = camera_model->GetMap2();
        fx = camera_model->GetK().at<float>(0,0);
        fy = camera_model->GetK().at<float>(1,1);
        cx = camera_model->GetK().at<float>(0,2);
        cy = camera_model->GetK().at<float>(1,2);
        distortion_valid = camera_model->IsValid();
       

        // Obtain ROI for distorted images
        if (distortion_valid)
        {
            CalculateROI(image);
            cout<< "distortion detected"<<endl;
            cout << "Input width = "<<w_input<<"\t"<< " Output width = "<<  w <<endl;
            cout << "Input height = "<<h_input<<"\t"<< " Output height = "<< h<< endl;
        }
        else{
            w = w_input;
            h = h_input;
        }
            
        InitializePyramid( w, h, K );
        // Initialize tracker system
        /*tracker_ = new Tracker(depth_available_);
        tracker_->InitializePyramid(w_, h_, K_);
        tracker_->InitializeMasks();
        */

        // Initialize map
        //map_ = new Map(); //para el futuro :)

        // Initialize output visualizer


        // Cheking if the number of depth images are greater or lower than the actual number of images

        initialized = true;
        cout << "Initializing system ... done" << endl << endl;
        //outputFile.open(_outputPath); // agregar fecha automticamente

      // Posicion inicial de la imu
        positionImu = _iniPosition;
        velocityImu = _iniVelocity;
        RPYOrientationImu = _iniRPY;
        qOrientationImu = toQuaternion(_iniRPY.x, _iniRPY.y, _iniRPY.z);
        world2imuTransformation = RPYAndPosition2transformationMatrix(RPYOrientationImu, positionImu);
        world2imuRotation = transformationMatrix2rotationMatrix(world2imuTransformation);
        Quaternion quat_initImu(qOrientationImu.w,qOrientationImu.x,qOrientationImu.y,qOrientationImu.z);
        final_poseImu = SE3(quat_initImu, SE3::Point(0.0, 0.0, 0.0));


      // Posicion inicial de la camara;
        positionCam = Mat2point(imu2camTransformation*point2MatPlusOne(positionImu));
        velocityCam = Mat2point(imu2camRotation*point2Mat(velocityImu));
        RPYOrientationCam = rotationMatrix2RPY(imu2camRotation*world2imuRotation);
        qOrientationCam = toQuaternion(RPYOrientationCam.x, RPYOrientationCam.y, RPYOrientationCam.z);
        Quaternion quat_initCam(qOrientationCam.w,qOrientationCam.x,qOrientationCam.y,qOrientationCam.z);
        final_poseCam = SE3(quat_initCam, SE3::Point(-positionCam.x, -positionCam.z, -positionCam.y));

        // IMU 
        imuCore.createPublisher(1.0/(camera_model->imu_frecuency));
        imuCore.initializate(_iniRPY.z); // Poner yaw inicial del gt
        imuCore.setImuInitialVelocity(_iniVelocity);
     
        // Camera
        num_max_keyframes = camera_model->min_features; // Solo almacenar 10 keyframes
        min_features = camera_model->min_features;
        start_index = camera_model->start_index;
        InitializeCameraGPU(camera_model->detector, camera_model->matcher, w, h, camera_model->num_cells, camera_model->length_patch );


    }


    void VISystemGPU::InitializeCameraGPU(int _detector, int _matcher, int _w_size, int _h_size, int _num_cells, int _length_path)
    {
        cameraGPU.initializateCameraGPU(_detector, _matcher, _w_size, _h_size, _num_cells, _length_path );
    }
    
    void VISystemGPU::AddFrameGPU(Mat _currentImage, vector <Point3d> _imuAngularVelocity, vector <Point3d> _imuAcceleration)
    {
        imuCore.setImuData(_imuAngularVelocity, _imuAcceleration); // primeras medidas
        imuCore.estimate();
        cameraGPU.Update(_currentImage);
  
        bool key_added = cameraGPU.addGPUKeyframe();
    
        num_keyframes = cameraGPU.frameList.size();
        if (cameraGPU.frameList.size()> 1) // primera imagen agregada
        {
            //drawKeypoints(camera.frameList[camera.frameList.size()-2]->grayImage, camera.frameList[camera.frameList.size()-2]->nextGoodMatches , outputLastImage, Scalar(0,0, 255), DrawMatchesFlags::DEFAULT);
            //drawKeypoints(camera.frameList[camera.frameList.size()-1]->grayImage, camera.frameList[camera.frameList.size()-1]->prevGoodMatches, outputCurrentImage, Scalar(0,0, 255), DrawMatchesFlags::DEFAULT);
            cameraGPU.printStatistics();
            /*if (camera.frameList[camera.frameList.size()-1]->prevGoodMatches.size() < min_features)
            {
                //min_features = camera.frameList[camera.frameList.size()-1]->prevGoodMatches.size();
                camera.printStatistics();
                
            }
            */
            if (num_keyframes > num_max_keyframes)
            {
                FreeLastFrameGPU();
            }
            
            Track();
            
        }
        
        

    }


    void VISystemGPU::FreeLastFrameGPU()
    {
        cameraGPU.frameList[0]->~Frame();
        cameraGPU.frameList.erase(cameraGPU.frameList.begin());
    }

} 