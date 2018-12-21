#include "../include/VISystem.hpp"

namespace vi
{
     
    VISystem::VISystem()
    {
        initialized = false;
        distortion_valid = false;
        depth_available = false;
        num_keyframes = 0;
    }

    VISystem::VISystem(int argc, char *argv[])
    {
        ros::init(argc, argv, "vi_slam");  // Initialize ROS
        initialized = false;
        distortion_valid = false;
        depth_available = false;
        num_keyframes = 0;
    }

    VISystem::~VISystem() 
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


    void VISystem::InitializeSystem(string _calPath, Point3d _iniPosition, Point3d _iniVelocity, Point3d _iniRPY, Mat image)
    {
        // Check if depth images are available
        /*if (_depth_path != "")
            depth_available = true;
        */
        currentImage = image;
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
        InitializeCamera(camera_model->detector, camera_model->matcher, w, h, camera_model->num_cells, camera_model->length_patch );
        
        camera.Update(currentImage);
        bool key_added = camera.addKeyframe();
        cout << "Camera Initialized"<<endl;


    }


    void VISystem::CalculateROI(Mat image) {
        // Load first image
        Mat distorted, undistorted;
        distorted = image;
        remap(distorted, undistorted, map1, map2, INTER_LINEAR);

        // Find middle x and y of image (supposing a symmetrical distortion)
        int x_middle = (undistorted.cols - 1) * 0.5;
        int y_middle = (undistorted.rows - 1) * 0.5;
        
        Point p1, p2;    
        p1.x = 0;
        p1.y = 0;
        p2.x = undistorted.cols - 1;
        p2.y = undistorted.rows - 1;

        // Search x1_ROI distance to crop
        while (undistorted.at<uchar>(y_middle, p1.x) == 0)
            p1.x++;

        // Search x2_ROI distance to crop
        while (undistorted.at<uchar>(y_middle, p2.x) == 0)
            p2.x--;

        // Search y1_ROI distance to crop
        while (undistorted.at<uchar>(p1.y, x_middle) == 0)
            p1.y++;

        // Search y2_ROI distance to crop
        while (undistorted.at<uchar>(p2.y, x_middle) == 0)
            p2.y--;

        // Considering an error margin
        p1.x += 5;
        p2.x -= 5;
        p1.y += 5;
        p2.y -= 5;

        ROI = Rect(p1,p2);
        
        // Update w_ and h_ with ROI dimentions
        w = p2.x - p1.x;
        h  = p2.y - p1.y;
    }

    void VISystem::Calibration(string _calibration_path) 
    {
        cout << "Reading calibration xml file";
        camera_model = new CameraModel();
        camera_model ->GetCameraModel(_calibration_path);
        w = camera_model->GetOutputWidth();
        h = camera_model->GetOutputHeight();

        if (w%2!=0 || h%2!=0) {
            cout << "Output image dimensions must be multiples of 32. Choose another output dimentions" << endl;
            cout << "Exiting..." << endl;
            exit(0);
        }
    }

    void VISystem::AddFrame(Mat _currentImage, vector <Point3d> _imuAngularVelocity, vector <Point3d> _imuAcceleration)
    {
        prevImage = currentImage;
        currentImage = _currentImage.clone();
       
        
        imuCore.setImuData(_imuAngularVelocity, _imuAcceleration); // primeras medidas
        imuCore.estimate();
        camera.Update(_currentImage);
        bool key_added = camera.addKeyframe();
        num_keyframes = camera.frameList.size();
        if (camera.frameList.size()> 1) // primera imagen agregada
        {
            /*
            drawKeypoints(prevImage, camera.frameList[camera.frameList.size()-2]->nextGoodMatches , prevImageToShow, Scalar(0,0, 255), DrawMatchesFlags::DEFAULT);
            drawKeypoints(currentImage, camera.frameList[camera.frameList.size()-1]->prevGoodMatches, currentImageToShow, Scalar(0,0, 255), DrawMatchesFlags::DEFAULT);
            imshow("prevImage", prevImageToShow);
            imshow("currentImage", currentImageToShow);
            waitKey();
            */
            camera.printStatistics();

            /*if (camera.frameList[camera.frameList.size()-1]->prevGoodMatches.size() < min_features)
            {
                //min_features = camera.frameList[camera.frameList.size()-1]->prevGoodMatches.size();
                camera.printStatistics();
                
            }
            */
            if (num_keyframes > num_max_keyframes)
            {
                FreeLastFrame();
            }
            
            Track();
        }
        
        

    }
    
    void VISystem::AddFrame(Mat _currentImage, vector <Point3d> _imuAngularVelocity, vector <Point3d> _imuAcceleration, vector <Point3d> _gtRPY) 
    {
        currentImage = _currentImage.clone();
        imuCore.setImuData(_imuAngularVelocity, _imuAcceleration); // primeras medidas
        imuCore.estimate(_gtRPY);
        camera.Update(_currentImage);
        camera.addKeyframe();
        num_keyframes = camera.frameList.size();

        if (camera.frameList.size()> 1) // primera imagen agregada y actual
        {
            drawKeypoints(camera.frameList[camera.frameList.size()-2]->grayImage, camera.frameList[camera.frameList.size()-2]->nextGoodMatches , outputLastImage, Scalar(0,0, 255), DrawMatchesFlags::DEFAULT);
            drawKeypoints(camera.frameList[camera.frameList.size()-1]->grayImage, camera.frameList[camera.frameList.size()-1]->prevGoodMatches, outputCurrentImage, Scalar(0,0, 255), DrawMatchesFlags::DEFAULT);
            if (camera.frameList[camera.frameList.size()-1]->prevGoodMatches.size() < min_features)
            {
                //min_features = camera.frameList[camera.frameList.size()-1]->prevGoodMatches.size();
                camera.printStatistics();
                
            }
            if (num_keyframes > num_max_keyframes)
            {
                FreeLastFrame();
                cout << num_keyframes<<endl;
            }
            Track();
        }
       
        
    }

    void VISystem::FreeLastFrame()
    {
        camera.frameList[0]->~Frame();
        camera.frameList.erase(camera.frameList.begin());
    }
    
    // Gauss-Newton using Foward Compositional Algorithm - Using features
    void VISystem::EstimatePoseFeatures(Frame* _previous_frame, Frame* _current_frame) {
        // Gauss-Newton Optimization Options
        float epsilon = 0.001;
        float intial_factor = 10;
        int max_iterations = 10;
        float error_threshold = 0.005;
        int first_pyramid_lvl = 0;
        int last_pyramid_lvl = 0;
        float z_factor = 0.002;

        // Variables initialization
        float error         = 0.0;
        float initial_error = 0.0;    
        float last_error    = 0.0;

        // Initial pose and deltapose (assumes little movement between frames)
        Mat deltaMat = Mat::zeros(6,1,CV_32FC1);
        Sophus::Vector<float, SE3::DoF> deltaVector;
        for (int i=0; i<6; i++)
            deltaVector(i) = 0;

     
        Point3d residualRPYcam = rotationMatrix2RPY(imuCore.residual_rotationMatrix);//rotationMatrix2RPY(imu2camRotationRPY2rotationMatrix(imuCore.residualRPY));
        Quaterniond residualQcam = toQuaternion(residualRPYcam.x, residualRPYcam.y, residualRPYcam.z) ;

        //Quaternion quat_init(residualQcam.w, residualQcam.x, residualQcam.y, residualQcam.z); // w, x, y,
        Quaternion quat_init(1,  0, 0, 0);
        //cout << SO3::exp(SE3::Point(0.0, 0.0, 0.0))<<endl;
        SE3 current_pose = SE3(quat_init, SE3::Point(0.0, 0.0, 0.0));
        
        /*
        cout << current_pose.unit_quaternion().x()<<endl;
        cout << current_pose.unit_quaternion().y()<<endl;
        cout << current_pose.unit_quaternion().z()<<endl;
        cout << current_pose.unit_quaternion().w()<<endl;
        */
       

        // Sparse to Fine iteration
        // Create for() WORKED WITH LVL 2
        for (int lvl = first_pyramid_lvl; lvl>=last_pyramid_lvl; lvl--) {
            // lvl = 0;
            // Initialize error   
            error = 0.0;
            last_error = 50000.0;
            float factor = intial_factor * (lvl + 1);

            // Obtain image 1 and 2
            Mat image1 = _previous_frame->grayImage[lvl].clone();
            Mat image2 = _current_frame->grayImage[lvl].clone();

            // Obtain points and depth of initial frame 
            Mat candidatePoints1  = _previous_frame->candidatePoints[lvl].clone();
            Mat candidatePoints2  = _current_frame->candidatePoints[lvl].clone(); 
            Mat candidateDebugPoints1 =  _previous_frame->candidateDebugPoints[lvl].clone();
            
            //candidatePoints1 = AddPatchPointsFeatures(candidatePoints1, lvl);
            // cout << candidatePoints1.rows << endl;
            // Mat imageWarped = Mat::zeros(image1.size(), CV_8UC1);
            // ObtainImageTransformed(image1, candidatePoints1, candidatePoints1, imageWarped);             
            // DebugShowWarpedPerspective(image1, image2, imageWarped, lvl);

            // Obtain gradients           
            Mat gradientX1 = Mat::zeros(image1.size(), CV_16SC1);
            Mat gradientY1 = Mat::zeros(image1.size(), CV_16SC1);
            gradientX1 = _previous_frame->gradientX[lvl].clone();
            gradientY1 = _previous_frame->gradientY[lvl].clone();

            // Obtain intrinsic parameters 
            Mat K = K_[lvl];
            // Optimization iteration
            for (int k=0; k<max_iterations; k++) {
                
                // Warp points with current pose and delta pose (from previous iteration)
                SE3 deltaSE3;
                Mat warpedPoints = Mat(candidatePoints1.size(), CV_32FC1);
                Mat warpedDebugPoints = Mat(candidateDebugPoints1.size(), CV_32FC1);


                //warpedPoints = WarpFunctionOpenCV(candidatePoints1, current_pose, lvl);
                warpedPoints = WarpFunction(candidatePoints1, current_pose, lvl);
                warpedDebugPoints = WarpFunction(candidateDebugPoints1, current_pose, lvl);

                vector<KeyPoint> warpedDebugKeyPoints ;
                MatPoint2Keypoints(warpedDebugPoints, warpedDebugKeyPoints );
             
      
                if (k == 0){ // primera iteracion

        
                drawKeypoints(currentImage, camera.frameList[camera.frameList.size()-1]->prevGoodMatches , currentImageToShow, Scalar(0,0, 255), DrawMatchesFlags::DEFAULT);
                drawKeypoints(currentImageToShow, warpedDebugKeyPoints, currentImageDebugToShow, Scalar(255,0, 0), DrawMatchesFlags::DEFAULT);
                putText(currentImageDebugToShow,"iter ="+to_string(k), Point2d(10,20), FONT_HERSHEY_SIMPLEX, 1,Scalar(0,255,0),2, LINE_AA);
                imshow("currentDebugImage", currentImageDebugToShow);
                waitKey(500);


                }
                else{

                drawKeypoints(currentImage, camera.frameList[camera.frameList.size()-1]->prevGoodMatches , currentImageToShow, Scalar(0,0, 255), DrawMatchesFlags::DEFAULT);
                drawKeypoints(currentImageToShow, warpedDebugKeyPoints, currentImageDebugToShow, Scalar(0,255, 0), DrawMatchesFlags::DEFAULT);
                putText(currentImageDebugToShow,"iter ="+to_string(k), Point2d(10,20), FONT_HERSHEY_SIMPLEX, 1,Scalar(0,255,0),2, LINE_AA);
                imshow("currentDebugImage", currentImageDebugToShow);
                waitKey(500);


                }
              

                
                // Mat imageWarped = Mat::zeros(image1.size(), CV_8UC1);
                // ObtainImageTransformed(image1, candidatePoints1, warpedPoints, imageWarped);    
                // imshow("warped", imageWarped);
                // waitKey(0);
                // Computation of Jacobian and Residuals
                Mat Jacobians;
                Mat Residuals;      
                int num_valid = 0;
                for (int i=0; i<candidatePoints1.rows; i++) {
                    Mat Residual = Mat(1,1,CV_32FC1);        
                    Mat Jacobian_row = Mat::zeros(1,6,CV_32FC1);
                    Mat Jw = Mat::zeros(2,6,CV_32FC1);
                    Mat Jl = Mat::zeros(1,2,CV_32FC1);
                    
                    // Point in frame 1            
                    float x1 = candidatePoints1.at<float>(i,0);
                    float y1 = candidatePoints1.at<float>(i,1);
                    float z1 = candidatePoints1.at<float>(i,2);
                    // Points of warped frame
                    float x2 = warpedPoints.at<float>(i,0);
                    float y2 = warpedPoints.at<float>(i,1);
                    float z2 = warpedPoints.at<float>(i,2);

                    float inv_z2 = 1 / z2;

                    // Check if warpedPoints are out of boundaries
                    if (y2>0 && y2<image2.rows && x2>0 && x2<image2.cols) {
                        if (z2!=0) {
                            if (inv_z2<0) 
                                inv_z2 = 0;
                            num_valid++;
                            Jw.at<float>(0,0) = fx_[lvl] * inv_z2;
                            Jw.at<float>(0,1) = 0.0;
                            Jw.at<float>(0,2) = -(fx_[lvl] * x2 * inv_z2 * inv_z2) * z_factor;
                            Jw.at<float>(0,3) = -(fx_[lvl] * x2 * y2 * inv_z2 * inv_z2);
                            Jw.at<float>(0,4) = (fx_[lvl] * (1 + x2 * x2 * inv_z2 * inv_z2));   
                            Jw.at<float>(0,5) = - fx_[lvl] * y2 * inv_z2;

                            Jw.at<float>(1,0) = 0.0;
                            Jw.at<float>(1,1) = fy_[lvl] * inv_z2;
                            Jw.at<float>(1,2) = -(fy_[lvl] * y2 * inv_z2 * inv_z2) * z_factor;
                            Jw.at<float>(1,3) = -(fy_[lvl] * (1 + y2 * y2 * inv_z2 * inv_z2));
                            Jw.at<float>(1,4) = fy_[lvl] * x2 * y2 * inv_z2 * inv_z2;
                            Jw.at<float>(1,5) = fy_[lvl] * x2 * inv_z2;

                        
                            // Intensities
                            int intensity1 = image1.at<uchar>(y1,x1);
                            int intensity2 = image2.at<uchar>(round(y2),round(x2));

                            Residual.at<float>(0,0) = intensity2 - intensity1;
                            Jl.at<float>(0,0) = gradientX1.at<short>(y1,x1);
                            Jl.at<float>(0,1) = gradientY1.at<short>(y1,x1);

                            Jacobian_row = Jl * Jw;
                            // cout << "Residual: " << Residual.at<float>(0,0) << endl;
                            // cout << "Jl: " << Jl << endl;                
                            // cout << "Jw: " << Jw << endl;                                
                            // cout << "Jacobian: " << Jacobian_row << endl;
                            // cout << endl;
                            
                            Jacobians.push_back(Jacobian_row);
                            Residuals.push_back(Residual);
                        }
                    }
                }
                // cout << "Valid points found: " << num_valid << endl;
                // DebugShowJacobians(Jacobians, warpedPoints, w_[lvl], h_[lvl]);

                // Computation of Weights (Identity or Tukey function)
                Mat W = IdentityWeights(Residuals.rows);
                //Mat W = TukeyFunctionWeights(Residuals);

                // Computation of error
                float inv_num_residuals = 1.0 / Residuals.rows;
                Mat ResidualsW = Residuals.mul(W);
                Mat errorMat =  inv_num_residuals * Residuals.t() * ResidualsW;
                error = errorMat.at<float>(0,0);
                if (k==0)
                    initial_error = error;

                // Break if error increases
                if (error >= last_error || k == max_iterations-1 || abs(error - last_error) < epsilon) {
                    // cout << "Pyramid level: " << lvl << endl;
                    // cout << "Number of iterations: " << k << endl;
                    // cout << "Initial-Final Error: " << initial_error << " - " << last_error << endl << endl;

                    // if (lvl == last_pyramid_lvl) {
                    //     DebugShowJacobians(Jacobians, warpedPoints, w_[lvl], h_[lvl]);
                    //     Mat imageWarped = Mat::zeros(image1.size(), CV_8UC1);
                    //     ObtainImageTransformed(image1, candidatePoints1, warpedPoints, imageWarped);             
                    //     DebugShowWarpedPerspective(image1, image2, imageWarped, lvl);
                    // }

                    // Reset delta
                    deltaMat = Mat::zeros(6,1,CV_32FC1);
                    for (int i=0; i<6; i++)
                        deltaVector(i) = 0;

                    break;
                }

                last_error = error;

                // Checking dimentions of matrices
                // cout << "Jacobians dimentions: " << Jacobians.size() << endl;
                // cout << "Weights dimentions: " << W.size() << endl;
                // cout << "Residuals dimentions: " << Residuals.size() << endl;
                
                // Computation of new delta (DSO-way)
                // LS ls;
                // ls.initialize(Residuals.rows);
                // for (int i=0; i<Residuals.rows; i++) {
                //     Mat61f jacobian;
                //     cv2eigen(Jacobians.row(i), jacobian);
                    
                //     ls.update(jacobian, Residuals.at<float>(i,0), W.at<float>(i,0));
                // }
                // ls.finish();
                // // Solve LS system
                // float LM_lambda = 0.2;
                // Mat61f b = -ls.b;
                // Mat66f A = ls.A;
                // deltaVector = A.ldlt().solve(b);

                // Computation of new delta (Kerl-way)            
                // Multiplication of W to Jacobian
                for (int i=0; i<Jacobians.rows; i++) {
                    float wi = W.at<float>(i,0);
                    Jacobians.row(i) = wi * Jacobians.row(i);
                }

                Residuals = Residuals.mul(1);  // Workaround to make delta updates larger
                Mat A = Jacobians.t() * Jacobians;                    
                Mat b = -Jacobians.t() * Residuals.mul(W);

                //cout << b << endl;
                deltaMat = A.inv() * b;
                //cout << A.inv() << endl;
                //cout << A << endl;
                
                // Convert info from eigen to cv
                for (int i=0; i<6; i++)
                    deltaVector(i) = deltaMat.at<float>(i,0);

                // Update new pose with computed delta
                current_pose = current_pose * SE3::exp(deltaVector);
                //cout << current_pose.matrix() << endl;
                
            }

            // Scale current_pose estimation to next lvl
            if (lvl !=0) {
                Mat31f t = 2 * current_pose.translation();

                Quaternion quaternion = current_pose.unit_quaternion();

                quaternion.x() = quaternion.x() * 2;
                quaternion.y() = quaternion.y() * 2;
                quaternion.z() = quaternion.z() * 2;
                
                current_pose = SE3(quaternion, t);
            }
            
            //current_pose = SE3(current_pose.unit_quaternion() * 2, current_pose.translation() * 2);
        }

        _previous_frame->rigid_transformation_ = current_pose;
        

    }


    void VISystem::InitializePyramid(int _width, int _height, Mat _K) {
        w_[0] = _width;
        h_[0] = _height;
        K_[0] = _K;
        // invK_[0] = _K.inv();

        fx_[0] = _K.at<float>(0,0);
        fy_[0] = _K.at<float>(1,1);
        cx_[0] = _K.at<float>(0,2);
        cy_[0] = _K.at<float>(1,2);
        
        invfx_[0] = 1 / fx_[0]; 
        invfy_[0] = 1 / fy_[0]; 
        invcx_[0] = 1 / cx_[0]; 
        invcy_[0] = 1 / cy_[0]; 
        int lvl;
        for (lvl = 1; lvl < 5; lvl++) {
            w_[lvl] = _width >> lvl;
            h_[lvl] = _height >> lvl;
            fx_[lvl] = fx_[lvl-1] * 0.5;
            fy_[lvl] = fy_[lvl-1] * 0.5;
            cx_[lvl] = (cx_[0] + 0.5) / ((int)1<<lvl) - 0.5;
            cy_[lvl] = (cy_[0] + 0.5) / ((int)1<<lvl) - 0.5;

            K_[lvl] = Mat::eye(Size(3,3), CV_32FC1);
            K_[lvl].at<float>(0,0) = fx_[lvl];
            K_[lvl].at<float>(1,1) = fy_[lvl];
            K_[lvl].at<float>(0,2) = cx_[lvl];  
            K_[lvl].at<float>(1,2) = cy_[lvl];    

            invfx_[lvl] = 1 / fx_[lvl];
            invfy_[lvl] = 1 / fy_[lvl];
            invcx_[lvl] = 1 / cx_[lvl];
            invcy_[lvl] = 1 / cy_[lvl];
            
            // Needs review
            // invK_[lvl] = K_[lvl].inv();
            // invfx_[lvl] = invK_[lvl].at<float>(0,0); 
            // invfy_[lvl] = invK_[lvl].at<float>(1,1);
            // invcx_[lvl] = invK_[lvl].at<float>(0,2);
            // invcy_[lvl] = invK_[lvl].at<float>(1,2);
        }
    }

    Mat VISystem::WarpFunction(Mat _points2warp, SE3 _rigid_transformation, int _lvl) {
        int lvl = _lvl;

        Mat projected_points = Mat(_points2warp.size(), CV_32FC1);
        projected_points = _points2warp.clone();

        Mat44f rigidEigen = _rigid_transformation.matrix();
        Mat rigid = Mat(4,4,CV_32FC1);
        eigen2cv(rigidEigen, rigid);

        //cout << rigid << endl;

        float fx = fx_[lvl];
        float fy = fy_[lvl];
        float invfx = invfx_[lvl];
        float invfy = invfy_[lvl];
        float cx = cx_[lvl];
        float cy = cy_[lvl];

        // 2D -> 3D

        // X  = (x - cx) * Z / fx 
        projected_points.col(0) = ((projected_points.col(0) - cx) * invfx);
        projected_points.col(0) = projected_points.col(0).mul(projected_points.col(2));

        // Y  = (y - cy) * Z / fy    
        projected_points.col(1) = ((projected_points.col(1) - cy) * invfy);
        projected_points.col(1) = projected_points.col(1).mul(projected_points.col(2));
        //cout << projected_points.row(projected_points.rows-1) << endl;    

        // Z = Z

        // Transformation of a point rigid body motion
        projected_points = rigid * projected_points.t();

        // 3D -> 2D
        // x = (X * fx / Z) + cx
        projected_points.row(0) *= fx;    
        projected_points.row(0) /= projected_points.row(2);
        projected_points.row(0) += cx;
        
        // x = (Y * fy / Z) + cy    
        projected_points.row(1) *= fy;    
        projected_points.row(1) /= projected_points.row(2);
        projected_points.row(1) += cy;

        //cout << projected_points.col(0) << endl;
        
        // Cleaning invalid points
        projected_points.row(0) = projected_points.row(0).mul(projected_points.row(3));
        projected_points.row(1) = projected_points.row(1).mul(projected_points.row(3));
        //

        // Transposing the points due transformation multiplication
        return projected_points.t();
    }

    Mat VISystem::IdentityWeights(int _num_residuals) 
    {
        Mat W = Mat::ones(_num_residuals,1,CV_32FC1);    
        return W;
    }

    void VISystem::Track()
    {
        // Estimar pose de camara con solver
        EstimatePoseFeatures(camera.frameList[camera.frameList.size()-2], camera.frameList[camera.frameList.size()-1]);

        // Cam
        Point3d residualRPYcam = rotationMatrix2RPY(imuCore.residual_rotationMatrix);//rotationMatrix2RPY(imu2camRotationRPY2rotationMatrix(imuCore.residualRPY));
        Quaterniond residualQCam = toQuaternion(residualRPYcam.x, residualRPYcam.y, residualRPYcam.z) ;
        Quaternion quat_initCam(residualQCam.w,  residualQCam.x, residualQCam.y, residualQCam.z); // w, x, y,
        current_poseCam = SE3(quat_initCam, SE3::Point(0.0, 0.0, 0.0));
        //current_poseCam = SE3(camera.frameList[camera.frameList.size()-2]->rigid_transformation_.unit_quaternion(), (camera.frameList[camera.frameList.size()-2]->rigid_transformation_.translation()));
        final_poseCam = final_poseCam*current_poseCam;

        Mat31f t = final_poseCam.translation();
        positionCam.x = -t(0);
        positionCam.y = -t(2);
        positionCam.z = -t(1);
        qOrientationCam.x = final_poseCam.unit_quaternion().x();
        qOrientationCam.y = final_poseCam.unit_quaternion().y();
        qOrientationCam.z = final_poseCam.unit_quaternion().z();
        qOrientationCam.w = final_poseCam.unit_quaternion().w();
        RPYOrientationCam = toRPY(qOrientationCam);

        // Imu
        Point3d residualRPYImu = rotationMatrix2RPY(imuCore.residual_rotationMatrix);//rotationMatrix2RPY(imu2camRotationRPY2rotationMatrix(imuCore.residualRPY));
        Quaterniond residualQImu= toQuaternion(residualRPYImu.x, residualRPYImu.y, residualRPYImu.z) ;
        Quaternion quat_initImu(residualQImu.w,  residualQImu.x, residualQImu.y, residualQImu.z); // w, x, y,
        current_poseImu = SE3(quat_initImu, SE3::Point(0.0, 0.0, 0.0));

        final_poseImu = final_poseImu*current_poseImu;

        Mat31f t2 = final_poseImu.translation();
        positionImu.x = -t2(0);
        positionImu.y = -t2(2);
        positionImu.z = -t2(1);
        qOrientationImu.x = final_poseImu.unit_quaternion().x();
        qOrientationImu.y = final_poseImu.unit_quaternion().y();
        qOrientationImu.z = final_poseImu.unit_quaternion().z();
        qOrientationImu.w = final_poseImu.unit_quaternion().w();
        RPYOrientationImu = toRPY(qOrientationImu);



        

        
    }

    void  VISystem::InitializeCamera(int _detector, int _matcher, int _w_size, int _h_size, int _num_cells, int _length_path)
    {
        camera.initializate(_detector, _matcher, _w_size, _h_size, _num_cells, _length_path );
    }


    void VISystem::MatPoint2Keypoints( Mat _MatPoints, vector<KeyPoint> &_outputKeypoints)
    {
        for (int i = 0; i < _MatPoints.rows ; i++)
        {
            KeyPoint point; // Punto final condicionado al umbral
            point.pt.x = _MatPoints.at<float>(i, 0);
            point.pt.y = _MatPoints.at<float>(i, 1);
            _outputKeypoints.push_back(point);
        }
      
    }

}


