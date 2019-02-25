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


    void VISystem::InitializeSystem(string _calPath, Point3d _iniPosition, Point3d _iniVelocity, double _iniYaw, Mat image, vector <Point3d> _imuAngularVelocity, vector <Point3d> _imuAcceleration )
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

        
         
        current_gtPosition = _iniPosition+imu2camTranslation ;

        K = camera_model->GetOriginalK();
        cout << K << endl;
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

          // IMU 
        imuCore.createPublisher(1.0/(camera_model->imu_frecuency));
        imuCore.initializate(_iniYaw, _iniVelocity,  _imuAngularVelocity, _imuAcceleration); // Poner yaw inicial del gt
       
        // Initialize tracker system
        /*tracker_ = new Tracker(depth_available_);
        tracker_->InitializePyramid(w_, h_, K_);
        tracker_->InitializeMasks();
        */

        // Initialize map
        //map_ = new Map(); //para el futuro :)

        // Initialize output visualizer


        // Cheking if the number of depth images are greater or lower than the actual number of images


        //outputFile.open(_outputPath); // agregar fecha automticamente

      // Posicion inicial de la imu
        positionImu = _iniPosition;
        velocityImu = _iniVelocity;
        RPYOrientationImu = imuCore.rpyAnglesWorld.back();
        cout << "RPYOrientationImu"<< RPYOrientationImu<< endl;
        cout << _iniYaw<<endl;
        qOrientationImu = imuCore.quaternionWorld.back();
        world2imuTransformation = RPYAndPosition2transformationMatrix(RPYOrientationImu, positionImu);
        world2imuRotation = transformationMatrix2rotationMatrix(world2imuTransformation);
        Quaternion quat_initImu(qOrientationImu.w,qOrientationImu.x,qOrientationImu.y,qOrientationImu.z);
        final_poseImu = SE3(quat_initImu, SE3::Point(0.0, 0.0, 0.0));


      // Posicion inicial de la camara;
        positionCam = positionImu+imu2camTranslation;
        velocityCam = velocityImu;
        RPYOrientationCam = rotationMatrix2RPY(world2imuRotation*imu2camRotation);
        qOrientationCam = toQuaternion(RPYOrientationCam.x, RPYOrientationCam.y, RPYOrientationCam.z);
        Mat33f rotationEigen ;
        cv2eigen(world2imuRotation*imu2camRotation, rotationEigen);
        final_poseCam = SE3(rotationEigen, SE3::Point(positionCam.x, positionCam.y, positionCam.z));
        Mat44f rigidEigen = final_poseCam.matrix();
        Mat rigid = Mat(4,4,CV_32FC1);
        eigen2cv(rigidEigen, rigid);
       

        //cout << rigid << endl;
        //cout << positionCam<<endl;

        prev_world2camTransformation = RPYAndPosition2transformationMatrix(RPYOrientationCam, positionCam);;
        //cout << prev_world2camTransformation <<endl;
      
     
        // Camera
        num_max_keyframes = camera_model->min_features; // Solo almacenar 10 keyframes
        min_features = camera_model->min_features;
        start_index = camera_model->start_index;
        InitializeCamera(camera_model->detector, camera_model->matcher, w, h, camera_model->num_cells, camera_model->length_patch );
        
        camera.Update(currentImage);
        nPointsLastKeyframe = camera.detectAndComputeFeatures(); // asumir que la primera imagen es buena y es keyframe
        lastImageWasKeyframe = true;
        camera.saveFrame();
        cout << "Camera Initialized con "<< camera.frameList.back()->keypoints.size() << " features"<<endl;

        initialized = true;
        cout << "Initializing system ... done" << endl << endl;


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



    bool VISystem::AddFrame(Mat _currentImage, vector <Point3d> _imuAngularVelocity, vector <Point3d> _imuAcceleration)
    {
        prevImage = currentImage;
        currentImage = _currentImage.clone();
       
        
        imuCore.setImuData(_imuAngularVelocity, _imuAcceleration); // primeras medidas
        imuCore.estimate();
        velocityCam = velocityCam+imuCore.residualVelocity; // velocidad igual a la de la imu
        velocityImu = velocityCam;
        accCam = imuCore.accelerationWorld.back(); // acceleration igual a la de la imu
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
            // Estimar pose de camara con solver
            //EstimatePoseFeatures(camera.frameList[camera.frameList.size()-2], camera.frameList[camera.frameList.size()-1]);
            //EstimatePoseFeaturesRansac(camera.frameList[camera.frameList.size()-2], camera.frameList[camera.frameList.size()-1]);
            Track();
        }
        
        return false;

    }

    bool VISystem::AddFrame(Mat _currentImage, vector <Point3d> _imuAngularVelocity, vector <Point3d> _imuAcceleration, Point3d _gtPosition)
    {
        prev_gtPosition = current_gtPosition;
        current_gtPosition = _gtPosition+imu2camTranslation ;
        current_gtTraslation = current_gtPosition-prev_gtPosition ; // Traslation entre frames
        prevImage = currentImage;
        currentImage = _currentImage.clone();
       
        
        imuCore.setImuData(_imuAngularVelocity, _imuAcceleration); // Medidas tomadas
        imuCore.estimate();
        velocityCam = velocityCam+imuCore.residualVelocity; // velocidad igual a la de la imu
        velocityImu = velocityCam;
        accCam = imuCore.accelerationWorld.back(); // acceleration igual a la de la imu
     

        // Limpiar basura por favor

        // Analizar imagen nueva
        camera.Update(_currentImage);
        nPointsCurrentImage = camera.detectAndComputeFeatures();
        


        currentImageIsKeyframe = false;

        if (nPointsCurrentImage > int(nPointsLastKeyframe*0.35)) // se considera que es hay suficientes puntos para el match
        {
            camera.computeGoodMatches(); // se calcula el match entre la imagen actual y el ultimo keyframe
            if (lastImageWasKeyframe){
                init_rotationMatrix = RPY2rotationMatrix(imuCore.rpyAnglesWorld[0]);
                final_rotationMatrix = RPY2rotationMatrix(imuCore.rpyAnglesWorld.back());
                RotationResCam =  imu2camRotation.t()* init_rotationMatrix.t()*final_rotationMatrix*imu2camRotation;

            }
            else
            {
                final_rotationMatrix = RPY2rotationMatrix(imuCore.rpyAnglesWorld.back());
                RotationResCam =  imu2camRotation.t()* init_rotationMatrix.t()*final_rotationMatrix*imu2camRotation;
            }

            Point3d rpyResidual = rotationMatrix2RPY(RotationResCam);
             float disparityThreshold = 22;
             double disparityAngThreshold = 5.0;

            double disparityAng = 180/M_PI*sqrt(rpyResidual.x*rpyResidual.x+rpyResidual.y*rpyResidual.y+rpyResidual.z*rpyResidual.z);
            float disparity = Disparity(camera.frameList.back()->nextGoodMatches, camera.currentFrame->prevGoodMatches);

            cout << "Disparity Trans = " << disparity <<endl;
            cout << "Disparity Ang = " << disparityAng <<endl;
            
            //if (disparity > disparityThreshold) currentImageIsKeyframe = true;   
            //if (disparityAng > disparityAngThreshold) currentImageIsKeyframe = true;
            /*
            if (currentImageIsKeyframe)
            {
                camera.saveFrame(); // Guardar frame como keyframe
                num_keyframes = camera.frameList.size();

                if (num_keyframes > num_max_keyframes)
                {
                    FreeLastFrame();
                }
                
                // Estimar traslacion entre keyframes
                EstimatePoseFeaturesDebug(camera.frameList[camera.frameList.size()-2], camera.frameList[camera.frameList.size()-1]);
                Triangulate(camera.frameList[camera.frameList.size()-2]->nextGoodMatches, camera.frameList[camera.frameList.size()-1]->prevGoodMatches);

                // Obtener la nueva posicion de la cámarael
                Track();

                imshow("currentDebugImage1", currentImageDebugToShow);
                imshow("currentDebugImage2", currentImageToShow);
                char c_input = (char) waitKey(1);
                if( c_input == 'q' | c_input == ((char)27) )  {
                        exit(0);
                }
                if( c_input == 'k'  )  {
                        currentImageIsKeyframe = true;
                }
                nPointsLastKeyframe = nPointsCurrentImage;
            }
            */
            vector <KeyPoint>  filter1, filter2;
            //filter1 = camera.frameList[camera.frameList.size()-1]->nextGoodMatches;
            //filter2 = camera.currentFrame->prevGoodMatches;

            FilterKeypoints(camera.frameList[camera.frameList.size()-1]->nextGoodMatches, camera.currentFrame->prevGoodMatches, filter1, filter2, 500.0);
            EstimatePoseFeaturesDebug(camera.frameList[camera.frameList.size()-1], camera.currentFrame);
            Triangulate(filter1, filter2);

                // Obtener la nueva posicion de la cámarael
           
            imshow("currentDebugImage1", currentImageDebugToShow);
            imshow("currentDebugImage2", currentImageToShow);

          
                
            char c_input = (char) waitKey(-1);
            if( c_input == 'q' | c_input == ((char)27) )  {
                    exit(0);
            }
            if( c_input == 'k'  )  {
                    currentImageIsKeyframe = true;
            }

             if (currentImageIsKeyframe)
            {
                camera.saveFrame(); // Guardar frame como keyframe
                num_keyframes = camera.frameList.size();

                if (num_keyframes > num_max_keyframes)
                {
                    FreeLastFrame();
                }
                  nPointsLastKeyframe = nPointsCurrentImage;
            }
          
        

          
            
            

        }
     
        

        lastImageWasKeyframe = currentImageIsKeyframe;

        return lastImageWasKeyframe;
        

    }
    
 

    void VISystem::FreeLastFrame()
    {
        camera.frameList[0]->~Frame();
        camera.frameList.erase(camera.frameList.begin());
    }
    
    // Lujano Algorithm
    void VISystem::setGtRes(Mat TranslationResGT, Mat RotationResGT )
    {
        TranslationResidual = TranslationResGT;
        RotationResidual =  RPY2rotationMatrix (rotationMatrix2RPY(RotationResGT)) ;
    }


  float VISystem::Disparity(vector <KeyPoint> keyPoints, vector <KeyPoint> inPoints )
  {
        int lvl = 0;
        float fx = fx_[lvl];
        float fy = fy_[lvl];
        float cx = cx_[lvl];
        float cy = cy_[lvl];

        float disparitySum = 0;
        float disparity = 0;

        int numkeypoints = keyPoints.size();

        float u1, v1, u2, v2, a, b; // u1, v1 es la reproyeccion de u2,v2 considerando solo rotacion
        float uk1, vk1; // posicion en pixeles del punto del keyframe
        Point3f inPoint3dHomo ; 
        Point3f outPoint3dHomo; 

        for (int index = 0; index< numkeypoints ; index++)
        {
            u2 = inPoints[index].pt.x;
            v2 = inPoints[index].pt.y;

            uk1 = keyPoints[index].pt.x;
            vk1 = keyPoints[index].pt.y;

            a =(u2-cx)/fx;
            b =(v2-cy)/fy;
            inPoint3dHomo.x = a;
            inPoint3dHomo.y = b;
            inPoint3dHomo.z = 1.0;

            outPoint3dHomo = RotationResCam*inPoint3dHomo;
            u1 = fx*outPoint3dHomo.x/outPoint3dHomo.z+cx;
            v1 = fy*outPoint3dHomo.y/outPoint3dHomo.z+cy;
            
            //cout << RotationResCam <<endl;
            //cout << "u2 " << u2 << " v2 "<< v2 << endl;
            //cout << "err" << uk1 << " vk1 "<< vk1 << endl;
            
            disparitySum = disparitySum+ sqrt((uk1-u1)*(uk1-u1) +(vk1-v1)*(vk1-v1) );
            
            

        }

        disparity = disparitySum/numkeypoints; // Disparidad promedio

        return disparity;
  }

  void VISystem::EstimatePoseFeaturesDebug(Frame* _previous_frame, Frame* _current_frame)
    {
        
        Point3f  translationGt;

        Point3d rotationGt = rotationMatrix2RPY(RotationResidual)*180/M_PI;
        Point3d rotationEst = rotationMatrix2RPY(RotationResCam)*180/M_PI;

        
        translationGt.x = TranslationResidual.at<float>(0,0);
        translationGt.y = TranslationResidual.at<float>(1,0);
        translationGt.z = TranslationResidual.at<float>(2,0);

        

        int lvl = 0;

       
        
         
       
        

        
        vector<KeyPoint> warpedDebugKeyPoints;


        //WarpFunctionRT(_current_frame->prevGoodMatches, RotationResidual, TranslationResidual, warpedDebugKeyPoints);
        //WarpFunctionRT(_current_frame->prevGoodMatches, Mat::eye(3, 3, CV_32FC1), Mat::zeros(3, 1, CV_32FC1), warpedDebugKeyPoints);

         // WarpFunctionRT(_current_frame->prevGoodMatches, Mat::eye(3, 3, CV_32FC1), Mat::zeros(3, 1, CV_32FC1), warpedDebugKeyPoints, 1.0);
         //WarpFunctionRT(_current_frame->prevGoodMatches, Mat::eye(3, 3, CV_32FC1), Mat::zeros(3, 1, CV_32FC1), warpedDebugKeyPoints2, 10);
        
        /*
        float u1 = warpedDebugKeyPoints[0].pt.x;
        float v1 = warpedDebugKeyPoints[0].pt.y;
        float u2 = _previous_frame->nextGoodMatches[0].pt.x;
        float v2 = _previous_frame->nextGoodMatches[0].pt.y;
        cout << "u1 " << u1 << " v1 " <<  v1 << " u2 " << u2<< " v2 " << v2 <<endl;
        */
        
        
        drawKeypoints(_previous_frame->grayImage[lvl], _previous_frame->nextGoodMatches, currentImageDebugToShow, Scalar(255,0, 0), DrawMatchesFlags::DEFAULT);
        //drawKeypoints(currentImageDebugToShow, warpedDebugKeyPoints, currentImageDebugToShow, Scalar(0,255, 0), DrawMatchesFlags::DEFAULT);
        drawKeypoints(currentImageDebugToShow, _current_frame->prevGoodMatches, currentImageDebugToShow, Scalar(0,0, 255), DrawMatchesFlags::DEFAULT);
        drawKeypoints(_current_frame->grayImage[lvl], _current_frame->prevGoodMatches, currentImageToShow, Scalar(0,0, 255), DrawMatchesFlags::DEFAULT);

      

        clock_t begin= clock(); 
        translationResEst = F2FRansac(_previous_frame->nextGoodMatches, _current_frame->prevGoodMatches, RotationResCam, 370.0);
        if(translationResEst.dot(translationGt)<0)
        {
            translationResEst = -translationResEst;
        }
        clock_t detect1 = clock(); 
        double elapsed_detect = double(detect1- begin) / CLOCKS_PER_SEC;  

        cout << "elapsed = " << elapsed_detect*1000<< " ms" <<endl; 
        cout << "TransGT" << " tx "<< translationGt.x<< " ty " << translationGt.y<< " tz " << translationGt.z <<endl;
        cout << "TransEst" << " tx "<< translationResEst.x<< " ty " <<translationResEst.y<< " tz " << translationResEst.z <<endl;
       // cout << "RotaGt" << " rx "<< rotationGt.x<< " ry " << rotationGt.y<< " rz " << rotationGt.z <<endl;
        //cout << "RotaEst" << " rx "<< rotationEst.x<< " ry " << rotationEst.y<< " rz " << rotationEst.z <<endl;


        
        
    }

    void VISystem::FilterKeypoints(vector <KeyPoint> inPoints1, vector <KeyPoint> inPoints2, vector <KeyPoint> &outPoints1, vector <KeyPoint> &outPoints2, double threshold)
    {
        int lvl = 0;
        float fx = fx_[lvl];
        float fy = fy_[lvl];
        float cx = cx_[lvl];
        float cy = cy_[lvl];

        Point3d vector1; //Feature Vector en frame 1 
        Point3d vector2; // Feature Vector en frame2
        Point3d normal;  // vector del plano 1 y 2 de cada pareja de features


        int numkeypoints = inPoints1.size();
        Point3f pont;
        

        float u1, v1, u2, v2;
        float u1k, v1k, u2k, v2k; // key

        // Vector unitario de traslacion
        Point3d tVec;

        tVec.x = TranslationResidual.at<float>(0,0);
        tVec.y = TranslationResidual.at<float>(1,0);
        tVec.z = TranslationResidual.at<float>(2,0);
        tVec = tVec/sqrt(tVec.x*tVec.x +tVec.y*tVec.y+tVec.z*tVec.z);

        // Debug string con degeneracion
       

        int count  = 0;
        // Crear vectores normales al plano de correspondecias (epipolar)
        for (int i = 0; i< numkeypoints; i++)
        {
            u1 = inPoints1[i].pt.x;
            v1 = inPoints1[i].pt.y;
            u2 = inPoints2[i].pt.x;
            v2 = inPoints2[i].pt.y;

             // Creacion de vectores
            vector1.x = (u1-cx)/fx;
            vector1.y = (v1-cy)/fy;
            vector1.z = 1.0;

            vector2.x = (u2-cx)/fx;
            vector2.y = (v2-cy)/fy;
            vector2.z = 1.0;

            vector1 = vector1/sqrt(vector1.x*vector1.x +vector1.y*vector1.y+vector1.z*vector1.z);
            vector2 = vector2/sqrt(vector2.x*vector2.x +vector2.y*vector2.y+vector2.z*vector2.z);

            normal = vector1.cross(Matx33d(RotationResidual)*vector2); // Vector normal al plano epipolar
            
            double error = -1000.0/std::log10(abs(tVec.dot(normal)));

             if(error<threshold)
            {
                outPoints1.push_back(inPoints1[i]);
                outPoints2.push_back(inPoints2[i]);
                count++;
            }

        }

        cout << "count filter " <<  count << endl;


    }

    Point3f VISystem::F2FRansac(vector <KeyPoint> inPoints1, vector <KeyPoint> inPoints2, Matx33f rotationMat, double threshold)
    {
        int lvl = 0;
        float fx = fx_[lvl];
        float fy = fy_[lvl];
        float cx = cx_[lvl];
        float cy = cy_[lvl];
        
        
        //double thresholdError; //
        //thresholdError = 350.0;

        Point3d vector1, vector1k; //Feature Vector en frame 1
        Point3d vector2, vector2k; // Feature Vector en frame2
        Point3d normal, n1, n2;  // vector del plano 1 y 2 de cada pareja de features
        Point3d d, dnorm; // vector de dezplazamiento y normalizado

        int numkeypoints = inPoints1.size();
        Point3f pont;
        

        float u1, v1, u2, v2;
        float u1k, v1k, u2k, v2k; // key

        vector <Point3d> normalVectors;
        Mat degenerate = Mat::zeros(1, numkeypoints, CV_32F) ; // vector la degeneracion de los vectores

        float sx = TranslationResidual.at<float>(0,0);
        float sy = TranslationResidual.at<float>(1,0);
        float sz = TranslationResidual.at<float>(2,0);
        float scale = sqrt(sx*sx+sy*sy+sz*sz);

        // Debug string con degeneracion
       


        // Crear vectores normales al plano de correspondecias (epipolar)
        for (int i = 0; i< numkeypoints; i++)
        {
            u1 = inPoints1[i].pt.x;
            v1 = inPoints1[i].pt.y;
            u2 = inPoints2[i].pt.x;
            v2 = inPoints2[i].pt.y;

             // Creacion de vectores
            vector1.x = (u1-cx)/fx;
            vector1.y = (v1-cy)/fy;
            vector1.z = 1.0;

            vector2.x = (u2-cx)/fx;
            vector2.y = (v2-cy)/fy;
            vector2.z = 1.0;

            vector1 = vector1/sqrt(vector1.x*vector1.x +vector1.y*vector1.y+vector1.z*vector1.z);
            vector2 = vector2/sqrt(vector2.x*vector2.x +vector2.y*vector2.y+vector2.z*vector2.z);

            normal = vector1.cross(Matx33d(rotationMat)*vector2); // Vector normal al plano epipolar
            normalVectors.push_back(normal); // 
            degenerate.at<float>(0, i) = sqrt(normal.x*normal.x+normal.y*normal.y+normal.z*normal.z);
             std::ostringstream strs;
             /*
            strs << fixed<< setprecision(2)<< degenerate.at<float>(0, i)*10;
            std::string str = strs.str();
            putText(currentImageDebugToShow,str , Point2d(u1,v1 ), FONT_HERSHEY_SIMPLEX, 0.32,Scalar(50,255,50),0.9, LINE_AA);
            */


        }


        
        // Filtrar los vectores normales en funcion de su degeneracion
        
        vector <Point3d> normalVectorsOk; // Vectores filtrados  
        Mat sortedIndexes; // vector para almacenar los indices de  con las valores ordenados;

        // Ordenar arreglos de coordenadas de forma descendente, y guardar los indices en Sorted;
        cv::sortIdx(degenerate, sortedIndexes, CV_SORT_EVERY_ROW + CV_SORT_DESCENDING); 

        int sizeNewGroup = int(numkeypoints*1.0); // Numero de vectores normales a considerar en el filtro
        int index;
        for (int i = 0; i<sizeNewGroup; i++)
        {
            index = sortedIndexes.at<int>(0, i); // indice
            normalVectorsOk.push_back(normalVectors[index]);
            //cout << degenerate.at<float>(0, index) <<endl;
        }
        
        // Obtener vector de traslacion RANSAC del grupo filtrado
        int index1, index2;
        Point3d dVector;
        float errorMin= 10000000;
        Point3f optimalDistance;
        float errorSum = 0.0;
        float count = 0;
        float countMax = 0;

        int RANSAC_iter = 1000;
        for (int i = 0; i<RANSAC_iter; i++)
        {
            index1 = rand()%(sizeNewGroup -1); // Tomar un feature aleatorio
            index2 = rand()%(sizeNewGroup -1); // Tomar un feature aleatorio
            errorSum = 0.0;
            dVector = normalVectors[index1].cross(normalVectors[index2]);
            if (dVector.x != 0.0 || dVector.y != 0.0 ||  dVector.z!=0.0)
            {
                dVector = dVector/sqrt(dVector.x*dVector.x +dVector.y*dVector.y+dVector.z*dVector.z);
                float errorx = (sx-scale*dVector.x)/sx;
                float errory = (sy-scale*dVector.y)/sy;
                float errorz = (sz-scale*dVector.z)/sz;

                //cout << "ex " << errorx*100 << " ey " << errory*100 << " ez " << errorz*100 <<endl;
                
                count = 0;
                for (int i = 0; i<sizeNewGroup ; i++)
                {
                    
                    double error = -1000.0/std::log10(abs(dVector.dot(normalVectors[i])));
                    //cout<<" e "  << error<<endl;
                    if(error<threshold)
                    {
                        count++;
                    }
                }

                if (count>countMax)
                {
                    countMax =count;
                    optimalDistance = Point3f(dVector);
                }


            }
            /*
            if ( (vector1-rotationMat*vector2).dot(dVector)>0)
            {
                dVector = -dVector;
            }
            */
        }

       cout << "countMax " << countMax << " points " <<  numkeypoints<<endl;


        /*
        cout << "optimal Distance = " <<scale*optimalDistance << endl;
        cout << "error min = "<< errorMin << endl;
        cout << "gt distance" << TranslationResidual <<endl;
        */

        


        return scale*optimalDistance ;


        
    }

    void VISystem::WarpFunctionRT(vector <KeyPoint> inPoints, Mat rotationMat, Mat translationMat, vector <KeyPoint> &outPoints)
    {
        Mat cameraMatrix = Mat::zeros(3,3,CV_32F);
        cameraMatrix.at<float>(0, 0) = fx_[0];
        cameraMatrix.at<float>(0, 2) = cx_[0];
        cameraMatrix.at<float>(1, 1) = fy_[0];
        cameraMatrix.at<float>(1, 2) = cy_[0];
        cameraMatrix.at<float>(2, 2) = 1.0;


        int lvl = 0;
        float fx = fx_[lvl];
        float fy = fy_[lvl];
        float cx = cx_[lvl];
        float cy = cy_[lvl];

        Mat projectionMat;
        hconcat(rotationMat, translationMat, projectionMat);
        //cout << projectionMat <<endl;
 
        //projectionMat = cameraMatrix * transformationMat;



        //cout << projectionMat <<endl;

          
        int numkeypoints = inPoints.size();

        float u1, v1, u2, v2, a, b;
        Mat inPoint3dHomo = Mat::ones(3, 1, CV_32FC1); 
        Mat outPoint3dHomo = Mat::ones(3, 1, CV_32FC1); 

        for (int index = 0; index< numkeypoints ; index++)
        {
            u2 = inPoints[index].pt.x;
            v2 = inPoints[index].pt.y;
            a =(u2-cx)/fx;
            b =(v2-cy)/fy;
            inPoint3dHomo.at<float>(0, 0) = a;
            inPoint3dHomo.at<float>(1, 0) = b;
            inPoint3dHomo.at<float>(2, 0) = 1.0;
            float tx  = translationMat.at<float>(0, 0);
            float ty  = translationMat.at<float>(1, 0);
            float tz  = translationMat.at<float>(2, 0);
            float sx  = a;
            float sy  = b;
            float sz  =  1.0;
            float module =  sqrt(tx*tx+ty*ty+tz*tz);
            float modulein =  sqrt(sx*sx+sy*sy+sz*sz);
            //translationMat = translationMat/module;
            outPoint3dHomo = rotationMat*inPoint3dHomo;//+translationMat/r;
            float c = outPoint3dHomo.at<float>(2, 0);
            float sca =(1-c)/translationMat.at<float>(2, 0);
            //cout << "sca" << sca <<endl;
            //cout << outPoint3dHomo.at<float>(2, 0) <<endl;
            //outPoint3dHomo = outPoint3dHomo+sca*translationMat;


            KeyPoint outKeypoint;
            //cout << a << "fdas " << b <<  " z " << endl;
            outKeypoint.pt.x = fx*outPoint3dHomo.at<float>(0, 0)/outPoint3dHomo.at<float>(2, 0)+cx;
            outKeypoint.pt.y = fy*outPoint3dHomo.at<float>(1, 0)/outPoint3dHomo.at<float>(2, 0)+cy;
            //cout <<  outPoint3dHomo.at<float>(2, 0) <<  " " << outPoint3dHomo.at<float>(3, 0) <<endl;
            if(index == -1)
            {
                cout <<  inPoint3dHomo <<endl;
                cout <<  outPoint3dHomo <<endl;
                cout << "u1 " << u1 << " v1 " <<  v1 << " u2 " << outKeypoint.pt.x << " v2 " << outKeypoint.pt.y <<endl;
            }
            

            outPoints.push_back(outKeypoint);

        }

        /*
        Vec3d rvec(0,0,0); //Rodrigues(R ,rvec);
        Vec3d tvec(0,0,0); // = P.col(3);
        vector <Point2d> reprojected_pt_set1;
        inPoint3dHomo2.convertTo(inPoint3dHomo2, CV_64FC1);
        cameraMatrix.convertTo(cameraMatrix, CV_64FC1);
        projectPoints(inPoint3dHomo2,rvec,tvec,cameraMatrix, Mat(),reprojected_pt_set1);
        */
        


        

    }

    void VISystem::Triangulate(vector <KeyPoint> inPoints1, vector <KeyPoint> inPoints2) 
    {

        Mat cameraMat = Mat::eye(3, 3, CV_32FC1);
        cameraMat.at<float>(0,0) = fx_[0];
        cameraMat.at<float>(0,2) = cx_[0];
        cameraMat.at<float>(1,1) = fy_[0];
        cameraMat.at<float>(1,2) = cy_[0];
        cameraMat.at<float>(2,2) = 1.0;
        Mat P1 = getProjectionMat(cameraMat, Mat::eye(3, 3, CV_32FC1), Mat::zeros(3, 1, CV_32FC1));
        Mat P2 = getProjectionMat(cameraMat, Mat(RotationResidual.t()), Mat(-RotationResidual.t())*TranslationResidual );


        array<vector<Point2f>,2> points;
        KeyPoint::convert( inPoints1 , points[0], vector<int>());
        KeyPoint::convert( inPoints2 , points[1], vector<int>());
                
        // Compute depth of 3D points using triangulation
        Mat mapPoints;
        
        
        if(inPoints1.size()!= 0)
        {
            triangulatePoints(P1, P2,  points[0], points[1],  mapPoints);

            Mat pt_3d; convertPointsFromHomogeneous(Mat(mapPoints.t()).reshape(4, 1),pt_3d);


            Vec3d rvec(0,0,0); //Rodrigues(R ,rvec);
            Vec3d tvec(0,0,0); // = P.col(3);
            vector<Point2f> reprojected_pt_set1;
            projectPoints(pt_3d,rvec,tvec,cameraMat, Mat(),reprojected_pt_set1);
            cout << " nr Triangulated " << inPoints1.size()<<endl;
        
            for (int index = 0; index <inPoints1.size(); index++)
            {

        
                //int index = i;
                float u1 = inPoints1[index].pt.x;
                float v1 = inPoints1[index].pt.y;
                
                
                float x = pt_3d.at<float>(0,index);
                float y = pt_3d.at<float>(1,index);
                float z = pt_3d.at<float>(2,index);
                float z2 = sqrt(x*x+y*y+z*z);
                std::ostringstream strs;
                
                strs << fixed<< setprecision(2)<<z2;
                std::string str = strs.str();
                float errorx = u1 - reprojected_pt_set1[index].x;
                float errory = v1 - reprojected_pt_set1[index].y;
                float errorf = sqrt( errorx*errorx+errory*errory );
            
                if (z>0.0) putText(currentImageDebugToShow, str , Point2d(u1,v1 ), FONT_HERSHEY_SIMPLEX, 0.37,Scalar(10,255,10),0.9, LINE_AA);
                
                
            }
        }

    }

    void VISystem::EstimatePoseFeaturesIter(Frame* _previous_frame, Frame* _current_frame)
    {
        Prof.clear();
        Matx33f Rotation_ResCam = imu2camRotation.t()* imuCore.residual_rotationMatrix*imu2camRotation;
        

        vector<KeyPoint> goodKeypoints1, goodKeypoints2;
        
        goodKeypoints1 = _previous_frame->nextGoodMatches;
        goodKeypoints2 = _current_frame->prevGoodMatches;

        int numkeypoints = goodKeypoints1.size();

        
        
    
       
        int lvl = 0;
        float fx = fx_[lvl];
        float fy = fy_[lvl];
        float cx = cx_[lvl];
        float cy = cy_[lvl];

        Matx33f rotationMatrix = Rotation_ResCam;

        double r11 = rotationMatrix(0,0);
        double r12 = rotationMatrix(0,1);
        double r13 = rotationMatrix(0,2);

        double r21 = rotationMatrix(1,0);
        double r22 = rotationMatrix(1,1);
        double r23 = rotationMatrix(1,2);

        double r31 = rotationMatrix(2,0);
        double r32 = rotationMatrix(2,1);
        double r33 = rotationMatrix(2,2);


        double coeffz;
        double coeffy;

        float sx = TranslationResidual.at<float>(0,0);
        float sy = TranslationResidual.at<float>(1,0);
        float sz = TranslationResidual.at<float>(2,0);

         //cout << "TransGT" << " sx " << sx<< " sy " << sy<< " sz " << sz <<endl;

        

        array<vector<Point2f>,2> points;
        KeyPoint::convert( goodKeypoints1 , points[0], vector<int>());
        KeyPoint::convert( goodKeypoints2 , points[1], vector<int>());
        
        
        
        // Compute current Rotation and Translation
       

        // Obtain projection matrices for the two perspectives
        Mat cameraMat = Mat::eye(3, 3, CV_32FC1);
        cameraMat.at<float>(0,0) = fx;
        cameraMat.at<float>(0,2) = cx;
        cameraMat.at<float>(1,1) = fy;
        cameraMat.at<float>(1,2) = cy;
        cameraMat.at<float>(2,2) = 1.0;
        
        Mat P1 = getProjectionMat(cameraMat, Mat::eye(3, 3, CV_32FC1), Mat::zeros(3, 1, CV_32FC1));
        Mat P2 = getProjectionMat(cameraMat, Mat::eye(3, 3, CV_32FC1), Mat::zeros(3, 1, CV_32FC1));
        //Mat P2 = getProjectionMat(cameraMat, rotationMatrix, TranslationResidual);
        
        // Compute depth of 3D points using triangulation
        Mat mapPoints;
        triangulatePoints(P1, P2, points[0], points[1], mapPoints);

        Mat pt_3d; convertPointsFromHomogeneous(Mat(mapPoints.t()).reshape(4, 1),pt_3d);
        Vec3d rvec(0,0,0); //Rodrigues(R ,rvec);
        Vec3d tvec(0,0,0); // = P.col(3);
        vector<Point2f> reprojected_pt_set1;
        projectPoints(pt_3d,rvec,tvec,cameraMat, Mat(),reprojected_pt_set1);


        

       


        drawKeypoints(currentImage, _current_frame->prevGoodMatches , currentImageDebugToShow, Scalar(50,50, 255), DrawMatchesFlags::DEFAULT);
        cout<< "GTz " << sz<<endl;
        for (int i = 0; i <(10); i++)
        {
            //Mat Traslation_ResCam = Mat::ones(3, 1, CV_32FC1);  
            int index = rand()%(numkeypoints-1); // Tomar un feature aleatorio
            
            //int index = i;
            float u1 = goodKeypoints1[index].pt.x;
            float v1 = goodKeypoints1[index].pt.y;
            float u2 = goodKeypoints2[index].pt.x;
            float v2 = goodKeypoints2[index].pt.y;

            double a = (u1-cx)/fx;
            double b = (v1-cy)/fy;
      
            double c = (u2-cx)/fx;
            double d = (v2-cy)/fy;


            double B1 = r11*c+r12*d+r13;
            double B2 = r21*c+r22*d+r23;
            double B3 = r31*c+r32*d+r33;

            double sigma = (B3*a-B1)/(B3*b-B2);

            coeffz = (b*sigma-a);
            coeffy = (-sigma);
            
            double error = -(sx+sy*coeffy)/coeffz;
            double z2__ = (sx-sz*a)/(B3*a-B1);
            double z2_ = (sy-sz*b)/(B3*b-B2);

            double z2 = 0.0;
            if (z2__> 0.00) z2 = z2__;
            if (z2_> 0.00) z2 = z2_;
            


            cout << " feature " << i << " error " <<  error <<endl;
            //            Prof.push_back(profundidad);
            /*
            z2 = pt_3d.at<float>(2,i);
            std::ostringstream strs;
            strs << fixed<< setprecision(2)<<z2;
            std::string str = strs.str();
            float errorx = (points[0])[index].x - reprojected_pt_set1[index].x;
            float errory = (points[0])[index].y - reprojected_pt_set1[index].y;
            float errorf = sqrt( errorx*errorx+errory*errory );
            if (errorf <5)
            {
                
                putText(currentImageDebugToShow,str , Point2d(u2,v2 ), FONT_HERSHEY_SIMPLEX, 0.32,Scalar(50,255,50),0.9, LINE_AA);
            }
            */
            

            //cout << "feature " << index << " Coeffz = " << coeffz << " Coeffy = " << coeffy <<endl;

            //Traslation_ResCam = (k1-Rotation_ResCam*k2);
            //cout << Traslation_ResCam <<endl;
            //Traslations_ResCam.push_back(Traslation_ResCam);
        }


        putText(currentImageDebugToShow,"tx="+ to_string(sx*100) +" ty="+ to_string(sy*100)+" tz="+ to_string(sz*100), Point2d(10,50), FONT_HERSHEY_SIMPLEX, 1,Scalar(0,255,0),2, LINE_AA);
        imshow("currentDebugImage", currentImageDebugToShow);
        char c_input = (char) waitKey(30);
        if( c_input == 'q' | c_input == ((char)27) )  {
                exit(0);
        }
     
        // Metodo iterativo
        /*
        double xSol, ySol, zSol, x, y, z;


        int num_iterarions = 10;
        cout << " cy1 " << coeffy1 << " cy2 " << coeffy2  << " cy3 " << coeffy3 <<endl;
        cout << " cz1 " << coeffz1 << " cz2 " << coeffz2  << " cz3 " << coeffz3 <<endl;
        for (int i = 0; i< num_iterarions; i++)
        {
            x = -zSol*coeffz1-ySol*coeffy1;
            y = (zSol*coeffz2+xSol)/(-coeffy2);
            z = (xSol+ySol*coeffy3)/(-coeffz3);

            xSol = x;
            ySol = y;
            zSol = z;

            cout << "iteration " << i << " xSol " << xSol << " ySol "<< ySol << " zSol "<< zSol <<endl;
        }
        */
        
        


    


    }
    // Gauss-Newton using Foward Compositional Algorithm - Using features
    void VISystem::EstimatePoseFeatures(Frame* _previous_frame, Frame* _current_frame) {
        // Gauss-Newton Optimization Options
        float epsilon = 0.001;
        //float intial_factor = 10;
        int max_iterations = 10; // 10
        float error_threshold = 0.005;
        int first_pyramid_lvl = 3;
        int last_pyramid_lvl = 0;
        float z_factor = 0.002; // 0.002

        // Variables initialization
        float error         = 0.0;
        float initial_error = 0.0;    
        float last_error    = 0.0;

        // Initial pose and deltapose (assumes little movement between frames)
        Mat deltaMat = Mat::zeros(6,1,CV_32FC1);
        Sophus::Vector<float, SE3::DoF> deltaVector;
        for (int i=0; i<6; i++)
            deltaVector(i) = 0;

        
        Point3d residualRPYcam = rotationMatrix2RPY(imu2camRotation.t()* imuCore.residual_rotationMatrix*imu2camRotation);//rotationMatrix2RPY(imu2camRotationRPY2rotationMatrix(imuCore.residualRPY));
        
        Matx33f world2imuRotation = imuCore.final_rotationMatrix;
        Point3d currentRPYCam = rotationMatrix2RPY(world2imuRotation*imu2camRotation);

        Point3d currentPositionCam = positionCam;
        currentPositionCam.z = currentPositionCam.z+imuCore.residualPosition.z;
        Mat currentTransformationCam = RPYAndPosition2transformationMatrix(currentRPYCam, currentPositionCam );
         
 
        
        Matx33f residualRotation = RPY2rotationMatrix(-residualRPYcam);
        cout << "dale"<<residualRotation<<endl;

       
        float sx = TranslationResidual.at<float>(0,0);
        float sy = TranslationResidual.at<float>(1,0);
        float sz = TranslationResidual.at<float>(2,0);

        cout << "TransGT" << " sx " << sx<< " sy " << sy<< " sz " << sz <<endl;

        Mat33f rotationEigen ;
        cv2eigen(residualRotation, rotationEigen);
        //cv2eigen(imuCore.residual_rotationMatrix, rotationEigen);
       
        //Quaterniond residualQcam = toQuaternion(residualRPYcam.x, residualRPYcam.y, residualRPYcam.z) ;
        //Quaternion quat_init(residualQcam.w, residualQcam.x, residualQcam.y, residualQcam.z); // w, x, y,
        //Quaternion quat_init(1,  0, 0, 0);
        //cout << SO3::exp(SE3::Point(0.0, 0.0, 0.0))<<endl;
        //Point3d t_btw = transformationMatrix2position(prev_world2camTransformation.inv()*currentTransformationCam); // traslacion entre frame pasado y actual respecto al frame pasado
        SE3 current_pose = SE3(rotationEigen, SE3::Point(-sx ,-sy, -sz)); //x, z, y
        //SE3 current_pose = SE3(rotationEigen, SE3::Point(0.0, 0.0, 0.0)); //x, z, y
        //SE3 current_poseDebug = SE3(rotationEigen, SE3::Point(0.0, 0.0, 0.0)); //x, z, y
        SE3 current_poseDebug = SE3(rotationEigen, SE3::Point(-sx, -sy, -sz)); //x, z, y
        //SE3 current_poseDebug = SE3(SO3::exp(SE3::Point(0.0, 0.0, 0.0)), SE3::Point(0.0, 0.0, 0.0)); //x, z, y
        
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
            //float factor = intial_factor * (lvl + 1);

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
                Mat warpedDebugPoints2 = Mat(candidateDebugPoints1.size(), CV_32FC1);


                //warpedPoints = WarpFunctionSE3OpenCV(candidatePoints1, current_pose, lvl);
                warpedPoints = WarpFunctionSE3(candidatePoints1, current_pose, lvl);
                warpedDebugPoints = WarpFunctionSE3(candidateDebugPoints1, current_pose, lvl);

                vector<KeyPoint> warpedDebugKeyPoints ;
                MatPoint2Keypoints(warpedDebugPoints, warpedDebugKeyPoints );


                warpedDebugPoints2 = WarpFunctionSE3(candidateDebugPoints1, current_poseDebug, lvl);

                vector<KeyPoint> warpedDebugKeyPoints2 ;
                MatPoint2Keypoints(warpedDebugPoints2, warpedDebugKeyPoints2 );
             
                
                if (k == 0){ // primera iteracion   
                    //drawKeypoints(_current_frame->grayImage[lvl], _current_frame->prevGoodMatches , currentImageToShow, Scalar(50,50, 255), DrawMatchesFlags::DEFAULT);
                    //drawKeypoints(currentImageToShow, warpedDebugKeyPoints, currentImageDebugToShow, Scalar(0,255, 0), DrawMatchesFlags::DEFAULT);
                    //drawKeypoints(currentImageDebugToShow, _previous_frame->nextGoodMatches, currentImageDebugToShow, Scalar(255,0, 0), DrawMatchesFlags::DEFAULT);
                    
                    drawKeypoints(_current_frame->grayImage[lvl], warpedDebugKeyPoints, currentImageDebugToShow, Scalar(0,255, 0), DrawMatchesFlags::DEFAULT);
                    drawKeypoints(currentImageDebugToShow, warpedDebugKeyPoints2, currentImageDebugToShow, Scalar(255,0, 0), DrawMatchesFlags::DEFAULT);
                    resize(currentImageDebugToShow, currentImageDebugToShow, Size(), pow(2, lvl),pow(2, lvl) );
                    drawKeypoints(currentImageDebugToShow, _current_frame->prevGoodMatches , currentImageDebugToShow, Scalar(50,50, 255), DrawMatchesFlags::DEFAULT);

               }
                else{

                   // drawKeypoints(_current_frame->grayImage[0], _current_frame->prevGoodMatches , currentImageToShow, Scalar(50,50, 255), DrawMatchesFlags::DEFAULT);
                   // drawKeypoints(currentImageToShow, warpedDebugKeyPoints, currentImageDebugToShow, Scalar(0,255, 0), DrawMatchesFlags::DEFAULT);

                   
                    drawKeypoints(_current_frame->grayImage[lvl], warpedDebugKeyPoints, currentImageDebugToShow, Scalar(0,255, 0), DrawMatchesFlags::DEFAULT);
                    drawKeypoints(currentImageDebugToShow, warpedDebugKeyPoints2, currentImageDebugToShow, Scalar(255, 0, 0), DrawMatchesFlags::DEFAULT);
                    resize(currentImageDebugToShow, currentImageDebugToShow, Size(), pow(2, lvl),pow(2, lvl) );
                    drawKeypoints(currentImageDebugToShow, _current_frame->prevGoodMatches , currentImageDebugToShow, Scalar(50,50, 255), DrawMatchesFlags::DEFAULT);

                }
                putText(currentImageDebugToShow,"lvl =" +to_string(lvl)+" iter ="+to_string(k), Point2d(10,20), FONT_HERSHEY_SIMPLEX, 1,Scalar(0,255,0),2, LINE_AA);
                //putText(currentImageDebugToShow,"tx="+ to_string(t_btw.x*100) +" ty="+ to_string(t_btw.y*100)+" tz="+ to_string(t_btw.z*100), Point2d(10,50), FONT_HERSHEY_SIMPLEX, 1,Scalar(0,255,0),2, LINE_AA);
                imshow("currentDebugImage", currentImageDebugToShow);
                char c_input = (char) waitKey(25);
                if( c_input == 'q' | c_input == ((char)27) )  {
                        exit(0);
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
                            Jw.at<float>(1,5) = -fy_[lvl] * x2 * inv_z2; // signo menos

                        
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
                 cout << "lvl = "<< lvl<<"Error it "<< k<< " ="<<error
                 << "    Last Error it "<< " ="<<last_error<<endl;
                if (k==0)
                    initial_error = error;

                // Break if error increases
                if (error >= last_error || k == max_iterations-1 || abs(error - last_error) < epsilon) {
                    //cout << "Pyramid level: " << lvl << endl;
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
                Mat31f t = 2 * current_pose.translation();
                //cout << "t "<< t <<endl;
                //cout << current_pose.matrix() << endl;
                
            }

            // Scale current_pose estimation to next lvl
            /*
            if (lvl !=0) {
                Mat31f t = 2 * current_pose.translation();

                Quaternion quaternion = current_pose.unit_quaternion();

                quaternion.x() = quaternion.x() * 2;
                quaternion.y() = quaternion.y() * 2;
                quaternion.z() = quaternion.z() * 2;
                
                current_pose = SE3(quaternion, t);
            }
            */
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

    Mat VISystem::WarpFunctionSE3(Mat _points2warp, SE3 _rigid_transformation, int _lvl) {
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


        //cout << projected_points.col(0) << endl;
        //cout << projected_points.col(2) << endl;
        //cout << projected_points.row(projected_points.rows-1) << endl;    

        // Z = Z

        // Transformation of a point rigid body motion
        //cout << "trans"<<endl;
        //cout << projected_points.t()<<endl;
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


        Mat33f rotationCamEigen ;
        cv2eigen(RotationResCam, rotationCamEigen);
        SE3::Point tRes;
        tRes.x() = translationResEst.x;
        tRes.y() = translationResEst.y;
        tRes.z() = translationResEst.z;
        
        current_poseCam = SE3(rotationCamEigen, tRes); // residual
        /*
        //current_poseCam = SE3(camera.frameList[camera.frameList.size()-2]->rigid_transformation_.unit_quaternion(), (camera.frameList[camera.frameList.size()-2]->rigid_transformation_.translation()));
        Mat44f rigidEigen = current_poseCam.matrix();
        
        Mat rigid = Mat(4,4,CV_32FC1);
        eigen2cv(rigidEigen, rigid);
       
        
        Point3d rpyCAM = rotationMatrix2RPY( transformationMatrix2rotationMatrix(rigid));
        Mat residualRotation = RPY2rotationMatrix(rpyCAM);
        //Mat residualRotation = RPY2rotationMatrix(rpyCAM);
        Mat33f rotationEigen ;
        cv2eigen(residualRotation, rotationEigen);
        current_poseCam.translation() = current_poseCam.translation();
        current_poseCam = SE3( rotationEigen, -current_poseCam.translation() );

        */
        


        final_poseCam = final_poseCam*current_poseCam;

        Mat31f t = final_poseCam.translation();
        positionCam.x = t(0);
        positionCam.y = t(1);
        positionCam.z = t(2);
        qOrientationCam.x = final_poseCam.unit_quaternion().x();
        qOrientationCam.y = final_poseCam.unit_quaternion().y();
        qOrientationCam.z = final_poseCam.unit_quaternion().z();
        qOrientationCam.w = final_poseCam.unit_quaternion().w();
        RPYOrientationCam = toRPY(qOrientationCam);
        prev_world2camTransformation = RPYAndPosition2transformationMatrix(RPYOrientationCam, positionCam);

        Mat33f rotationImuEigen ;
        cv2eigen(imuCore.residual_rotationMatrix, rotationImuEigen);
        current_poseImu = SE3(rotationImuEigen, SE3::Point(0.0, 0.0, 0.0));

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

    void VISystem::EstimatePoseFeaturesRansac(Frame* _previous_frame, Frame* _current_frame)
    {
        
        Mat cameraMatrix = Mat::zeros(3,3,CV_64F);
        cameraMatrix.at<double>(0, 0) = 458.654; //fx_[0];
        cameraMatrix.at<double>(0, 2) = 367.215; //cx_[0];
        cameraMatrix.at<double>(1, 1) = 457.296; //fy_[0];
        cameraMatrix.at<double>(1, 2) = 248.375 ;//cy_[0];
        cameraMatrix.at<double>(2, 2) = 1.0;


        cout << cameraMatrix << endl;

        std::vector<Point2f> points1_OK, points2_OK; // Puntos finales bajos analisis
        vector<int> point_indexs;

        //cout<< _previous_frame->nextGoodMatches[0].pt.x<<endl;
        //cout<< _current_frame->prevGoodMatches[0].pt.x<<endl;
        cv::KeyPoint::convert(_previous_frame->nextGoodMatches, points1_OK,point_indexs);
        cv::KeyPoint::convert(_current_frame->prevGoodMatches, points2_OK,point_indexs);


        int lvl= 0;
        Mat E; // essential matrix
        double focal = fx;
        E = findEssentialMat(points1_OK, points2_OK,  focal, Point2d(cx, cy), RANSAC, 0.999, 1.0, noArray());

   

        float sx = TranslationResidual.at<float>(0,0);
        float sy = TranslationResidual.at<float>(1,0);
        float sz = TranslationResidual.at<float>(2,0);
        Matx33f Rotation_ResCam = imu2camRotation.t()* imuCore.residual_rotationMatrix*imu2camRotation;
        //Rotation_ResCam.convertTo(Rotation_ResCam, CV_64FC1);
        //Mat trans_skew =  E*Rotation_ResCam.t();


        

        
       
        // Calcular la matriz de rotación y traslación de puntos entre imagenes 
        Mat R_out, t_out;
        
        //recoverPose(E, points1_OK, points1_OK, cameraMatrix, R_out, t_out, noArray());
        int p;
         p = recoverPose(E, points1_OK, points2_OK, R_out, t_out, focal, Point2d(cx, cy), noArray()   );
                R_out.convertTo(R_out, CV_32FC1);
                    t_out.convertTo(t_out, CV_32FC1);

                         float scale = sqrt(sx*sx+sy*sy+sz*sz);


        t_out = t_out*scale;


        vector<KeyPoint> warpedDebugKeyPoints;
        cout << "here"<<endl;
        WarpFunctionRT(_current_frame->prevGoodMatches,  R_out, t_out, warpedDebugKeyPoints);
        //WarpFunctionRT(_previous_frame->nextGoodMatches, Mat::eye(3, 3, CV_32FC1), trakity, warpedDebugKeyPoints);

         // WarpFunctionRT(_current_frame->prevGoodMatches, Mat::eye(3, 3, CV_32FC1), Mat::zeros(3, 1, CV_32FC1), warpedDebugKeyPoints, 1.0);
         //WarpFunctionRT(_current_frame->prevGoodMatches, Mat::eye(3, 3, CV_32FC1), Mat::zeros(3, 1, CV_32FC1), warpedDebugKeyPoints2, 10);
        
        float u1 = warpedDebugKeyPoints[0].pt.x;
        float v1 = warpedDebugKeyPoints[0].pt.y;
        float u2 = _previous_frame->nextGoodMatches[0].pt.x;
        float v2 = _previous_frame->nextGoodMatches[0].pt.y;
        cout << "u1 " << u1 << " v1 " <<  v1 << " u2 " << u2<< " v2 " << v2 <<endl;
        
        drawKeypoints(_previous_frame->grayImage[lvl], warpedDebugKeyPoints, currentImageDebugToShow, Scalar(0,255, 0), DrawMatchesFlags::DEFAULT);
        drawKeypoints(currentImageDebugToShow, _previous_frame->nextGoodMatches, currentImageDebugToShow, Scalar(255,0, 0), DrawMatchesFlags::DEFAULT);
        drawKeypoints(currentImageDebugToShow, _current_frame->prevGoodMatches, currentImageDebugToShow, Scalar(0,0, 255), DrawMatchesFlags::DEFAULT);
        drawKeypoints(_current_frame->grayImage[lvl], _current_frame->prevGoodMatches, currentImageToShow, Scalar(255,0, 0), DrawMatchesFlags::DEFAULT);

        imshow("currentDebugImage1", currentImageDebugToShow);
        imshow("currentDebugImage2", currentImageToShow);
        char c_input = (char) waitKey(-1);
        if( c_input == 'q' | c_input == ((char)27) )  {
                exit(0);
        }
        
        

        Point3d residualRPYcam = rotationMatrix2RPY(R_out);//rotationMatrix2RPY(imu2camRotationRPY2rotationMatrix(imuCore.residualRPY));
        //cout << R_out<<endl;
        //cout << points1_OK.size()<< " " << points2_OK.size()<<endl;
        //cout << "ResidualRPY "<<residualRPYcam<<endl;
        //cout << "Residualtrans "<< t_out<<endl;
       
        /*

        float sx1 = t_out.at<double>(0,0)*scale;
        float sy1 = t_out.at<double>(1,0)*scale;
        float sz1 = t_out.at<double>(2,0)*scale;
        float sx2 = -trans_skew.at<double>(1,2);
        float sy2 = -trans_skew.at<double>(2,0);
        float sz2 = -trans_skew.at<double>(0,1);

        


        //cout << trans_skew <<endl;

        cout << "TransGT" << " sx " << sx<< " sy " << sy<< " sz " << sz <<endl;

        cout << "TransE1" << " sx1 " << sx1<< " sy1 " << sy1<< " sz1 " << sz1 <<endl;

        //cout << "TransE2" << " sx2 " << sx2<< " sy2 " << sy2<< " sz2 " << sz2 <<endl;
        */
         /*
        Quaterniond residualQcam = toQuaternion(residualRPYcam.x, residualRPYcam.y, residualRPYcam.z) ;
        Quaternion quat_init(residualQcam.w, residualQcam.x, residualQcam.y, residualQcam.z); // w, x, y,

        Mat33f rotationEigen ;
        cv2eigen(R_out, rotationEigen);
        current_poseCam = SE3(rotationEigen, SE3::Point(t_out.at<double>(0,0)*0.04, t_out.at<double>(0,1)*0.04, t_out.at<double>(0,2)*0.04)); //x, z, y 0.0, 0.0));
        //current_poseCam = SE3(rotationEigen, SE3::Point(0.0, 0.0, 0.0)); //x, z, y 0.0, 0.0));


        // Show points for debug
        Mat candidateDebugPoints1 =  _previous_frame->candidateDebugPoints[0].clone();
        Mat warpedDebugPoints = WarpFunctionSE3(candidateDebugPoints1, current_poseCam, 0);

        vector<KeyPoint> warpedDebugKeyPoints ;
        MatPoint2Keypoints(warpedDebugPoints, warpedDebugKeyPoints );
        drawKeypoints(currentImage, _current_frame->prevGoodMatches , currentImageToShow, Scalar(50,50, 255), DrawMatchesFlags::DEFAULT);
        drawKeypoints(currentImageToShow, warpedDebugKeyPoints, currentImageDebugToShow, Scalar(255,0, 0), DrawMatchesFlags::DEFAULT);
        //putText(currentImageDebugToShow,"iter ="+to_string(k), Point2d(10,20), FONT_HERSHEY_SIMPLEX, 1,Scalar(0,255,0),2, LINE_AA);
        imshow("currentDebugImage", currentImageDebugToShow);
        char c_input = (char) waitKey(25);
        if( c_input == 'q' | c_input == ((char)27) )  {
                exit(0);
                
        }
    
        waitKey(30);
        */

    }


    Mat VISystem::TukeyFunctionWeights(Mat _input) 
    {
        int num_residuals = _input.rows;
        float b = 4.6851; // Achieve 95% efficiency if assumed Gaussian distribution for outliers
        Mat W = Mat(num_residuals,1,CV_32FC1);    

        // Computation of scale (Median Absolute Deviation)
        float MAD = MedianAbsoluteDeviation(_input);       

        if (MAD == 0) {
            cout << "Warning: MAD = 0." << endl;
            MAD = 1;
        }
        float inv_MAD = 1.0 / MAD;
        float inv_b2 = 1.0 / (b * b);

        for (int i=0; i<num_residuals; i++) {
            float prueba = _input.at<float>(i,0);
            float x = _input.at<float>(i,0) * inv_MAD;

            if (abs(x) <= b) {
                float tukey = (1.0 - (x * x) * inv_b2);
                W.at<float>(i,0) = tukey * tukey;
            } else {
                W.at<float>(i,0) = 0.0;
            }
        }

        return W;
    }


    float VISystem::MedianAbsoluteDeviation(Mat _input)
    {
        float c = 1.4826;

        Mat deviation = Mat(_input.rows, _input.cols, CV_32FC1);
        float median = MedianMat(_input);
        // Absolute Deviation from the _input's median
        deviation = abs(_input - median);

        // Median of deviation
        float MAD = MedianMat(deviation);

        return c * MAD;
    }



    float VISystem::MedianMat(Mat _input) 
    {
        Mat channel = Mat(_input.rows, _input.cols, CV_8UC1);
        _input.convertTo(channel, CV_8UC1);

        float m = (channel.rows*channel.cols) / 2;
        int bin = 0;
        float med = -1.0;

        int histSize = 256;
        float range[] = { 0, 256 };
        const float* histRange = { range };
        bool uniform = true;
        bool accumulate = false;
        Mat hist;
        calcHist(&channel, 1, 0, Mat(), hist, 1, &histSize, &histRange, uniform, accumulate);

        for (int i = 0; i < histSize && med < 0.0; ++i) {
            bin += cvRound(hist.at<float>(i));
            if (bin > m && med < 0.0)
                med = i;
        }

        return med;
    }

    Mat VISystem::getProjectionMat(Mat cameraMat, Mat rotationMat, Mat translationMat){
    // ProjectionMat = cameraMat * [Rotation | translation]
    Mat projectionMat;
    hconcat(rotationMat, translationMat, projectionMat);
 
    projectionMat = cameraMat * projectionMat;

    return projectionMat;
}




}





