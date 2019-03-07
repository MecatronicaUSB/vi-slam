#include "../include/VISystem.hpp"


namespace vi
{
     
    VISystem::VISystem()
    {
        num_keyframes = 0;
    }

    VISystem::VISystem(int argc, char *argv[])
    {
        ros::init(argc, argv, "vi_slam");  // Initialize ROS

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

    /// Inicializacion
    void VISystem::setConfig(string _calPath)
    {   
        
        readConfig(_calPath); // Leer archivo de configuracion (en camera_model)

        K = camera_model->GetOriginalK(); // Matriz de parametros intrisecos de la camara
    
        // Parametros intrisecos de la cámara
        w_input = camera_model->GetInputWidth();
        h_input = camera_model->GetInputHeight();
        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);

        // Parametros relacionados a la cámara
        num_max_keyframes = camera_model->min_features; // Solo almacenar 10 keyframes
        min_features = camera_model->min_features;
        start_index = camera_model->start_index;

        // Transformacion de acople entre la imu y la cámara (imu-> cámara)
        imu2camTransformation = (camera_model->imu2cam0Transformation);
        imu2camRotation = transformationMatrix2rotationMatrix(imu2camTransformation);
        imu2camTranslation = transformationMatrix2position(imu2camTransformation);

        // Configurar objeto cámara
        camera.initializate(camera_model->detector, camera_model->matcher, w_input, h_input, camera_model->num_wcells, camera_model->num_hcells);

        
        

        // Configura objeto imu
        imuCore.createPublisher(1.0/(camera_model->imu_frecuency));
        

        


    }


    void VISystem::readConfig(string _calibration_path) 
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

    void VISystem::setInitPose(Point3d _iniPosition, double _iniYaw)
    {
        
        // Posicion inicial de la imu
        positionImu = _iniPosition;
        iniYaw = _iniYaw;

        cout << "** Posicion Inicial del sistema **"<<endl;
        cout << "Yaw inicial = " <<_iniYaw*180/M_PI<<endl;
        cout << "Pos = " <<positionImu<<endl; 

     

    }

    void VISystem::setInitData(Mat image,  vector <Point3d> _imuAngularVelocity, vector <Point3d> _imuAcceleration)
    {

        currentImage = image;
        // Colocar medidas iniciales de IMU
        Point3d _iniVelocity = Point3d(0.0, 0.0, 0.0); // Asumir velocidad inicial cero
        imuCore.initializate(iniYaw, _imuAngularVelocity, _imuAcceleration); // Poner yaw inicial del gt
        currentFrame = new Frame();
        image.copyTo(currentFrame->grayImage); // Copiar imagen
        
        
        
        camera.setCurrentFrame(currentFrame);
        nPointsLastKeyframe = camera.detectAndComputeFeatures(); // asumir que la primera imagen es buena y es keyframe
        lastImageWasKeyframe = true;
        saveFrame();
        
        cout << "Camera Initialized con "<< currentFrame->keypoints.size() << " features"<<endl;
        


        // Calcular posiciones
        velocityImu = Point3d(0.0, 0.0, 0.0);
        RPYOrientationImu = imuCore.rpyAnglesWorld.back();
       

        qOrientationImu = imuCore.quaternionWorld.back();
        

        // Posicion inicial de la camara;
        positionCam = positionImu+imu2camTranslation;
        velocityCam = velocityImu;
        
        RPYOrientationCam = rotationMatrix2RPY(RPY2rotationMatrix(RPYOrientationImu)*imu2camRotation);
        qOrientationCam = toQuaternion(RPYOrientationCam.x, RPYOrientationCam.y, RPYOrientationCam.z);


        final_poseCam =PAndR2T(RPY2rotationMatrix(RPYOrientationImu)*imu2camRotation, positionCam );
        cout << "Orientacion inicial = " <<RPYOrientationCam*180/M_PI<<endl;
       // Matx44d eye (Matx44d::eye());
        //final_poseCam = eye;//(Mat::eye(4, 4, CV_64F));

        //cout << "init T" << final_poseCam<<endl;

        
        
        
        cout << "*** Sistema Inicializado ***" << endl << endl;

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


    bool VISystem::AddFrame(Mat _currentImage, vector <Point3d> _imuAngularVelocity, vector <Point3d> _imuAcceleration)
    {
        
       
        currentFrame = new Frame();
        
        currentImage = _currentImage.clone();
        currentImage.copyTo(currentFrame->grayImage); // Copiar imagen
       
        
        imuCore.setImuData(_imuAngularVelocity, _imuAcceleration); // Medidas tomadas
        imuCore.estimate();
        velocityCam = velocityCam+imuCore.residualVelocity; // velocidad igual a la de la imu
        velocityImu = velocityCam;
        accCam = imuCore.accelerationWorld.back(); // acceleration igual a la de la imu
     



        // Analizar imagen nueva
        camera.setPreviousFrame(keyFrameList.back());
        camera.setCurrentFrame(currentFrame);
        nPointsCurrentImage = camera.detectAndComputeFeatures();
        
        
    

        currentImageIsKeyframe = false;

        if (nPointsCurrentImage > int(nPointsLastKeyframe*0.35)) // se considera que es hay suficientes puntos para el match
        {
            clock_t begin = clock();
            camera.computeFastMatches(); // se calcula el match entre la imagen actual y el ultimo keyframe
            clock_t computeMatches = clock(); 
        
            if (lastImageWasKeyframe){
                
                init_rotationMatrix = RPY2rotationMatrix(imuCore.rpyAnglesWorld[0]);
                final_rotationMatrix = RPY2rotationMatrix(imuCore.rpyAnglesWorld.back());
             

            }
            else
            {
                final_rotationMatrix = RPY2rotationMatrix(imuCore.rpyAnglesWorld.back());
                
            }
            RotationResImu = init_rotationMatrix.t()*final_rotationMatrix;
            RotationResCam =  imu2camRotation.t()*RotationResImu*imu2camRotation;

            Point3d rpyResidual = rotationMatrix2RPY(RotationResCam);
             float disparityThreshold = 30;
             double disparityAngThreshold = 10.0;

            double disparityAng = 180/M_PI*sqrt(rpyResidual.x*rpyResidual.x+rpyResidual.y*rpyResidual.y+rpyResidual.z*rpyResidual.z);
            float disparity = Disparity(keyFrameList.back()->nextGoodMatches, currentFrame->prevGoodMatches);
            clock_t disparidad = clock();
            if (disparity > disparityThreshold) {
                currentImageIsKeyframe = true;
            }
            if (disparityAng > disparityAngThreshold) currentImageIsKeyframe = true;
          
            //cout << " Points detected = " << nPointsCurrentImage<<endl;
            /*
            if (disparity < 15) {
                disparityAngThreshold = 5.0;
            }
            if (disparity>=15 && disparity < 25) {
                disparityAngThreshold = 2.0;
            }
             if (disparity >= 25 && disparity<37) {
                disparityAngThreshold = 1.2;
            }
            if (disparity >= 37 && disparity<47) {
                disparityAngThreshold = 0.7;
            }
            if (disparity >47) {
                disparityAngThreshold = 0.3;
            }
            
            
            */
            if (currentImageIsKeyframe)
            {
           
                
                scale = norm(TranslationResidualGT); // escala inicial
                EstimatePoseFeaturesDebug(keyFrameList.back(), currentFrame);
                
                clock_t ransac = clock();
                vector<bool> mask;
                vector<KeyPoint> filter1, filter2, outlier1, outlier2, ess1, ess2;
                vector <Point3f> points3D;
                float thresholdGroup = computeErrorGroup(keyFrameList.back()->nextGoodMatches, currentFrame->prevGoodMatches);
                cout<< "ERRORGROUP = " << thresholdGroup<<endl; 
                if(1000.0< thresholdGroup*0.7)
                {
                    thresholdGroup = thresholdGroup*0.5;
                }
                cout<< "ERRORGROUPCORREGIDO = " << thresholdGroup*0.85<<endl; 
                FilterKeypoints(thresholdGroup*0.85, mask, filter1, filter2, outlier1, outlier2); // outlier Rejection

                //
                Mat E;
                std::vector<Point2f> points1_OK, points2_OK; // Puntos finales bajos analisis
                vector<int> point_indexs;

                //cout<< _previous_frame->nextGoodMatches[0].pt.x<<endl;
                //cout<< _current_frame->prevGoodMatches[0].pt.x<<endl;
                cv::KeyPoint::convert(keyFrameList.back()->nextMatches, points1_OK,point_indexs);
                cv::KeyPoint::convert(currentFrame->prevMatches, points2_OK,point_indexs);
                clock_t essential = clock();
                Mat maskE;
                E = findEssentialMat(points1_OK, points2_OK,  K, RANSAC, 0.999, 1.0, maskE);
                int countE = 0;
                for (size_t i = 0; i< points1_OK.size();i++)
                {
                    if(maskE.at<uchar>(i))
                    {
                        countE++;
                        ess1.push_back(keyFrameList.back()->nextMatches[i]);
                        ess2.push_back(currentFrame->prevMatches[i]);
                    }

                }
                
                clock_t essentialfinal = clock();
                /*
                 Mat R_out, t_out;

                //recoverPose(E, points1_OK, points1_OK, cameraMatrix, R_out, t_out, noArray());
                 int p;
                p = recoverPose(E, points1_OK, points2_OK,K,  R_out, t_out, maskE );
                clock_t recover = clock();
                R_out.convertTo(R_out, CV_32FC1);
                t_out.convertTo(t_out, CV_32FC1);

                cout << "matches" << currentFrame->prevMatches.size() <<endl;
                cout << "Eout " << countE<<endl;
                cout << "toutRecover "<< t_out*scale<<endl;
                
                */
                


                
                //translationResEst  = TranslationResidualGT;

                
                
                clock_t outlierRejection = clock();

                        
                Matx44d local_poseCam = PAndR2T(RotationResCam , translationResEst ); // residual
                Matx44d curr_poseCam = final_poseCam*local_poseCam;

                Matx33f prevR, currR; // Rotaciones
                Point3f prevt, currt; // posiciones

                prevR = transformationMatrix2rotationMatrix(final_poseCam);
                currR = transformationMatrix2rotationMatrix(curr_poseCam);


                prevt.x = final_poseCam(0, 3);
                prevt.y = final_poseCam(1, 3);
                prevt.z = final_poseCam(2, 3);

      


                currt.x = curr_poseCam(0, 3);
                currt.y = curr_poseCam(1, 3);
                currt.z = curr_poseCam(2, 3);
                /*

                cout << "final pose cam " << final_poseCam <<endl;
                cout << "prevR " << prevR <<endl;
                cout << "prevt " << prevt <<endl;
                cout << "curr pose cam " << curr_poseCam <<endl;
                cout << "currR " << currR <<endl;
                cout << "currt " << currt <<endl;
                */
                
                
                
                // Triangular inliers excepto por el factor de escala de la ultima estimacion de traslacion
                Triangulate(ess1, ess2, prevR, prevt, currR, currt, points3D ); 

                
                clock_t triangulate = clock();

                double elapsed_essential = double(essentialfinal-essential ) / CLOCKS_PER_SEC;
                //double elapsed_recover = double(recover -essentialfinal ) / CLOCKS_PER_SEC;
                double elapsed_matches = double(computeMatches- begin) / CLOCKS_PER_SEC;  
                double elapsed_disparidad = double(disparidad- computeMatches) / CLOCKS_PER_SEC;  
                double elapsed_ransac = double(ransac- disparidad) / CLOCKS_PER_SEC;  
                double elapsed_outlier = double(outlierRejection-ransac) / CLOCKS_PER_SEC;  
                double elapsed_triangulate = double(triangulate-outlierRejection) / CLOCKS_PER_SEC;  




        
                if (keyFrameList.size() != 0 ) // segundo keyframe
                {
                    landmarks.clear();
                    for (int i = 0; i< ess1.size(); i++)
                    {
                        Landmark landmark;

                        landmark.pt = points3D[i];
                        landmark.seen = 2;

                        landmarks.push_back(landmark);

                    }
                    // Add new 3d point
                  
                    //addNewLandmarks(points3D, mask);
                    

   

                    
                    

                }
                /*
                else
                {
                    //scale = computeScale(points3D, mask); // Y landmarks previamente calculadops
                    scale = norm(TranslationResidualGT); // escala inicial
                    Matx44d local_poseCam = PAndR2T(RotationResCam , scale*translationResEst ); // residual
                    Matx44d curr_poseCam = final_poseCam*local_poseCam;

                    Matx33f prevR, currR; // Rotaciones
                    Point3f prevt, currt; // posiciones

                    prevR = transformationMatrix2rotationMatrix(final_poseCam);
                    currR = transformationMatrix2rotationMatrix(curr_poseCam);


                    prevt.x = final_poseCam(0, 3);
                    prevt.y = final_poseCam(1, 3);
                    prevt.z = final_poseCam(2, 3);


                    currt.x = curr_poseCam(0, 3);
                    currt.y = curr_poseCam(1, 3);
                    currt.z = curr_poseCam(2, 3);

                    Triangulate(filter1, filter2, prevR, prevt, currR, currt, points3D ); 

                    addNewLandmarks(points3D, mask);



                }
                */
                /*
                cout<<"\nESTADISTICAS"
                <<"\nTiempo de matches: " << fixed<< setprecision(3) << elapsed_matches*1000<<" ms"
                <<"\tTiempo de disparidad: " << fixed<< setprecision(3) << elapsed_disparidad*1000<<" ms"
                <<"\nTiempo de RANSAC: " << fixed<< setprecision(3) << elapsed_ransac*1000<<" ms"
                <<"\tTiempo de outlierRej: " << fixed<< setprecision(3) << elapsed_outlier*1000<<" ms"
                <<"\nTiempo de triangulate: " << fixed<< setprecision(3) << elapsed_triangulate*1000<<" ms"
                << " Escala = " << scale 
                <<endl;
                ;
                */
                
                Mat celdas1, celdas2, essential1, essential2;
                drawKeypoints(keyFrameList.back()->grayImage, filter1, currentImageDebugToShow, Scalar(0,255, 0), DrawMatchesFlags::DEFAULT);
                drawKeypoints(keyFrameList.back()->grayImage, keyFrameList.back()->nextMatches, celdas1, Scalar(0,255, 0), DrawMatchesFlags::DEFAULT);
                drawKeypoints(keyFrameList.back()->grayImage, ess1, essential1, Scalar(0,255, 0), DrawMatchesFlags::DEFAULT);
                //drawKeypoints(currentImageDebugToShow, warpedDebugKeyPoints, currentImageDebugToShow, Scalar(0,255, 0), DrawMatchesFlags::DEFAULT);
                //drawKeypoints(currentImageDebugToShow, _current_frame->prevGoodMatches, currentImageDebugToShow, Scalar(0,0, 255), DrawMatchesFlags::DEFAULT);
                drawKeypoints(currentFrame->grayImage, filter2, currentImageToShow, Scalar(0,0, 255), DrawMatchesFlags::DEFAULT);
                drawKeypoints(currentFrame->grayImage, currentFrame->prevMatches, celdas2, Scalar(0,0, 255), DrawMatchesFlags::DEFAULT);
                drawKeypoints(currentFrame->grayImage, ess2, essential2, Scalar(0,0, 255), DrawMatchesFlags::DEFAULT);


                // Obtener la nueva posicion de la cámarael
                 Mat img_inlier1, img_inlier2;
                resize(currentImageDebugToShow, img_inlier1, Size( 640,300));//resize image
                resize(currentImageToShow, img_inlier2, Size(640, 300));//resize image
                resize(celdas1, celdas1, Size( 640,300));//resize image
                resize(celdas2, celdas2, Size(640, 300));//resize image
                resize(essential1, essential1, Size(640, 300));//resize image
                resize(essential2, essential2, Size(640, 300));//resize image
                 Mat img_inliers;
                 Mat img_celdas;
                 Mat img_essential;
                vconcat(img_inlier1, img_inlier2, img_inliers);
                vconcat(celdas1, celdas2, img_celdas);
                vconcat(essential1, essential2, img_essential);
                
        
                imshow("inliers", img_inliers);
                imshow("celdas", img_celdas);
                imshow("essential", img_essential);
                saveFrame();
                num_keyframes = keyFrameList.size();

                    // Estimar traslacion entre keyframes
                cout<<"\nTiempo de essential: " << fixed<< setprecision(3) << elapsed_essential*1000<<" ms"<<endl;
                //cout<<"\nTiempo de recover: " << fixed<< setprecision(3) << elapsed_recover*1000<<" ms"<<endl;
                cout << "*** DISPARIDAD ***" <<endl;
                cout << "Disparity Trans = " << disparity <<endl;
                cout << "Disparity Ang = " << disparityAng <<endl;

                if (num_keyframes > num_max_keyframes)
                {
                    FreeLastFrame();
                }
                currentImageIsKeyframe = true;
                nPointsLastKeyframe = nPointsCurrentImage;
                Track();
            
            
                
                
                
        

              
            }

            
                
             
            /*
            vector <KeyPoint>  filter1, filter2;
            //filter1 = camera.frameList[camera.frameList.size()-1]->nextGoodMatches;
            //filter2 = camera.currentFrame->prevGoodMatches;

            
            EstimatePoseFeaturesDebug(camera.frameList[camera.frameList.size()-1], camera.currentFrame);
            FilterKeypoints(camera.frameList[camera.frameList.size()-1]->nextGoodMatches, camera.currentFrame->prevGoodMatches, filter1, filter2, 500.0);
            Triangulate(filter1, filter2);

            // Obtener la nueva posicion de la cámara
           
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
            */
          
        

          
            
            

        }
     
        

        lastImageWasKeyframe = currentImageIsKeyframe;

        return lastImageWasKeyframe;
        

    }
    
    void VISystem::getLandmarks(vector<Point3f> &_landmarks)
    {
        for(int i = 0; i < landmarks.size(); i++)
        {
            _landmarks.push_back(landmarks[i].pt/(landmarks[i].seen-1)); //promedio del landmark
        }
    }

    void VISystem::FreeLastFrame()
    {
        keyFrameList[0]->~Frame();
        keyFrameList.erase(keyFrameList.begin());
    }

    void VISystem::saveFrame()
{
    currentFrame->isKeyFrame = true;
    keyFrameList.push_back(currentFrame);

}

    
    // Lujano Algorithm
    void VISystem::setGtTras(Point3d TranslationResGT )
    {
        TranslationResidualGT = TranslationResGT;
 
    }

  void VISystem::addNewLandmarks( vector <Point3f> points3D, vector <bool> mask)
  {
      

        // Debug string con degeneracion
        auto & prevKey = keyFrameList.back();
        auto & currKey = currentFrame;
        int indexInlier = 0;

        int numkeypoints =  prevKey->keypoints.size();
        
        landmarks.clear();
       
        
        for (int i = 0; i< numkeypoints; i++)
        {
            if(mask[i]) // si el keypoint es inlier
            {
               
            
                size_t match_idx = prevKey->kp_next_idx(i);
                if ( false/*prevKey->kp_3d_exist(i)*/) { // si existe el landmark
                    // Found a match with an existing landmark
                    
                    currKey->kp_3d_idx(match_idx) = prevKey->kp_3d_idx(i); // como hay match, el landmark se hereda
                    

                    // sumar nuevo punto a la medida del landmark
                    landmarks[prevKey->kp_3d_idx(i)].pt += points3D[indexInlier]; // suma de landmarks
                    landmarks[prevKey->kp_3d_idx(i)].seen++;
                } 
                else  // No existe el landmark observado
                {
                    // Add new 3d point
                    Landmark landmark;

                    landmark.pt = points3D[indexInlier];
                    landmark.seen = 2;

                    landmarks.push_back(landmark);

                    prevKey->kp_3d_idx(i) = landmarks.size() - 1; // guardar el indice del vector de landmark
                    currKey->kp_3d_idx(match_idx) = landmarks.size() - 1;// guardar el indice
                }

                indexInlier++;

            }
            

        }




  }

  float VISystem::computeScale(vector<Point3f> points3D, vector <bool> mask)
  {
        

        // Debug string con degeneracion
        auto & prevKey = keyFrameList.back();
        auto & currKey = currentFrame;
        float scaleSum = 0.0;

        int numkeypoints =  prevKey->keypoints.size();
       
      
        vector <Point3f> existing_landmarks;
        vector <Point3f> new_landmarks;
        int indexInlier = 0;
        int num_common_landmarks = 0;
        
        for (int i = 0; i< numkeypoints; i++)
        {
            if(mask[i]) // si el keypoint es inlier y tiene match
            {
                // Hallar los puntos comunes entre la triangulacion actual y los landmarks
                if(prevKey->kp_3d_exist(i)  ) // Si el keypoint tiene match con alguno del keyframe siguiente
                {
                    size_t idx = prevKey->kp_3d_idx(i); // landmark del keypoint
                    //cout << "i = " << i <<endl;
                    
                    //cout <<"pt " << SFM.landmark[idx].pt <<endl;
                    //cout <<" seen" <<SFM.landmark[idx].seen <<endl;
                    Point3f avg_landmark = landmarks[idx].pt / (landmarks[idx].seen - 1); // landmark promedio

                    // Landmarks actuales  (puntos triangulados actualmente)
                    new_landmarks.push_back(points3D[indexInlier]);
                    // Landmarks del sistema 
                    existing_landmarks.push_back(avg_landmark); // puntos existentes (3D)

                    // Estos landmarks son los vistos por el mismo keypoint
                    num_common_landmarks++;
                }
                indexInlier++; // numero de point3d


            }

           

        }

        // Tomar indices aleatorios entre los keypoints para tomar el promedio de la escala 
        int ii, jj;
       
        int count = 0;
        cout <<"num_common_landmarks  " << num_common_landmarks<<endl;
        int maxIter = 1000;

        for (int i = 0; i< maxIter ; i++)
        {
            ii = rand()%(num_common_landmarks  -1); // Tomar un feature aleatorio
            jj = rand()%(num_common_landmarks  -1); // Tomar un feature aleatorio

            if( norm(new_landmarks[ii] - new_landmarks[jj])!= 0.0)
            { 
                float s = norm(existing_landmarks[ii] - existing_landmarks[jj]) / norm(new_landmarks[ii] - new_landmarks[jj]);
                //cout << "s " << s <<endl;

                scaleSum += s; // suma de escala de puntos tomados
                count++;
            
            }
            


        }
        
        return scaleSum/count; // retornar el promedio de la escala entre landmarks

         
        

  }

  float VISystem::Disparity(vector <KeyPoint> keyPoints, vector <KeyPoint> inPoints )
  {
        
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
        
        Point3f  translationGt = Point3f(TranslationResidualGT);

        
        

        
        //vector<KeyPoint> warpedDebugKeyPoints;


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
        
        
       

      


        translationResEst = F2FRansac(_previous_frame->nextGoodMatches, _current_frame->prevGoodMatches, RotationResCam, 800.0);
        if(translationResEst.dot(translationGt)<0)
        {
            translationResEst = -translationResEst;
        }
        translationResEst = translationResEst*scale;

        cout << "TransGT" << " tx "<< translationGt.x<< " ty " << translationGt.y<< " tz " << translationGt.z <<endl;
        cout << "TransEst" << " tx "<< translationResEst.x<< " ty " <<translationResEst.y<< " tz " << translationResEst.z <<endl;
       // cout << "RotaGt" << " rx "<< rotationGt.x<< " ry " << rotationGt.y<< " rz " << rotationGt.z <<endl;
        //cout << "RotaEst" << " rx "<< rotationEst.x<< " ry " << rotationEst.y<< " rz " << rotationEst.z <<endl;


        
        
    }

    void VISystem::FilterKeypoints(double threshold, vector <bool> &mask, vector <KeyPoint> &outPoints1, vector <KeyPoint> &outPoints2,vector <KeyPoint> &outlier1, vector <KeyPoint> &outlier2)
    {
        Point3d vector1; //Feature Vector en frame 1
        Point3d vector2; // Feature Vector en frame2
        Point3d normal;  // vector del plano 1 y 2 de cada pareja de features        

        float u1, v1, u2, v2;


        // Vector unitario de traslacion
        Point3d tVec = TranslationResidualGT;

 
        tVec = tVec/norm(tVec);

        // Debug string con degeneracion
        auto & prevKey = keyFrameList.back();
        auto & currKey = currentFrame;

         int numkeypoints =  prevKey ->keypoints.size();
       

        int count  = 0;
        int count_matches  = 0;
        Point3f dVector = translationResEst;
        // Crear vectores normales al plano de correspondecias (epipolar)
        
        for (int i = 0; i< numkeypoints; i++)
        {
            if(prevKey->kp_next_exist(i)) // Si el keypoint tiene match con alguno del keyframe siguiente
            {
                count_matches++;
                u1 = prevKey->keypoints[i].pt.x;
                v1 = prevKey->keypoints[i].pt.y;

                u2 = currKey->keypoints[prevKey->kp_next_idx(i)].pt.x;
                v2 = currKey->keypoints[prevKey->kp_next_idx(i)].pt.y;

                // Creacion de vectores
                vector1.x = (u1-cx)/fx;
                vector1.y = (v1-cy)/fy;
                vector1.z = 1.0;

                vector2.x = (u2-cx)/fx;
                vector2.y = (v2-cy)/fy;
                vector2.z = 1.0;

                vector1 = vector1/sqrt(vector1.x*vector1.x +vector1.y*vector1.y+vector1.z*vector1.z);
                vector2 = vector2/sqrt(vector2.x*vector2.x +vector2.y*vector2.y+vector2.z*vector2.z);

                normal = vector1.cross(Matx33d(RotationResCam)*vector2); // Vector normal al plano epipolar
                normal = normal/sqrt(normal.x*normal.x +normal.y*normal.y+normal.z*normal.z); // normalizacion
                dVector = dVector/sqrt(dVector.x*dVector.x +dVector.y*dVector.y+dVector.z*dVector.z);
                
                double error = -1000.0/std::log10(abs(dVector.dot(normal)));

                if(error<threshold) 
                {
                    mask.push_back(true); 
                    count++;
                    outPoints1.push_back(prevKey->keypoints[i]);
                    outPoints2.push_back(currKey->keypoints[prevKey->kp_next_idx(i)]);
                    
                }
                else 
                {
                    mask.push_back(false);
                    outlier1.push_back(prevKey->keypoints[i]);
                    outlier2.push_back(currKey->keypoints[prevKey->kp_next_idx(i)]);
                }
                
                
            }
            else
            {
                mask.push_back(false);
            }
          

        }

        cout << count << " inliers of " << count_matches << " matches, of " << numkeypoints << " keypoints" <<endl;

        


    }

    Point3f VISystem::F2FRansac(vector <KeyPoint> inPoints1, vector <KeyPoint> inPoints2, Matx33f rotationMat, float threshold)
    {

        
        //double thresholdError; //
        //thresholdError = 350.0;
    

        Point3f vector1, vector1k; //Feature Vector en frame 1
        Point3f vector2, vector2k; // Feature Vector en frame2
        Point3f normal, n1, n2;  // vector del plano 1 y 2 de cada pareja de features
        Point3f d, dnorm; // vector de dezplazamiento y normalizado

        int numkeypoints = inPoints1.size();
  
        

        float u1, v1, u2, v2;
        float u1k, v1k, u2k, v2k; // key

        vector <Point3f> normalVectors;
        Mat degenerate = Mat::zeros(1, numkeypoints, CV_32F) ; // vector la degeneracion de los vectores


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

            normal = vector1.cross(rotationMat*vector2); // Vector normal al plano epipolar
            normal = normal/sqrt(normal.x*normal.x +normal.y*normal.y+normal.z*normal.z); // normalizacion
            normalVectors.push_back(normal); // 
            
            std::ostringstream strs;
             /*
            strs << fixed<< setprecision(2)<< degenerate.at<float>(0, i)*10;
            std::string str = strs.str();
            putText(currentImageDebugToShow,str , Point2d(u1,v1 ), FONT_HERSHEY_SIMPLEX, 0.32,Scalar(50,255,50),0.9, LINE_AA);
            */


        }


        
        // Filtrar los vectores normales en funcion de su degeneracion
        
        vector <Point3f> normalVectorsOk; // Vectores filtrados  
        Mat sortedIndexes; // vector para almacenar los indices de  con las valores ordenados;

        // Ordenar arreglos de coordenadas de forma descendente, y guardar los indices en Sorted;
        //cv::sortIdx(degenerate, sortedIndexes, CV_SORT_EVERY_ROW + CV_SORT_DESCENDING); 

        int sizeNewGroup = int(numkeypoints*1.0); // Numero de vectores normales a considerar en el filtro
        int index;
        /*
        for (int i = 0; i<sizeNewGroup; i++)
        {
            index = sortedIndexes.at<int>(0, i); // indice
            normalVectorsOk.push_back(normalVectors[index]);
            //cout << degenerate.at<float>(0, index) <<endl;
        }
        */
        
        // Obtener vector de traslacion RANSAC del grupo filtrado
        int index1, index2;
        Point3f dVector;
        float errorMin= 10000000;
        Point3f optimalDistance;
        float errorSum = 0.0;
        float count = 0;
        float countMax = 0;
        float errorGroup= 0;

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

                
                
                count = 0;
                for (int i = 0; i<sizeNewGroup ; i++)
                {
                    
                    double error = -1000.0/std::log10(abs(dVector.dot(normalVectors[i])));
                    errorSum +=error;
                        // cout<<" e "  << error<<endl;
                    if(error<threshold)
                    {
                        count++;
                    }
                }

                if (count>countMax)

                {
                    countMax =count;
                    errorGroup = errorSum/sizeNewGroup;
                    optimalDistance = dVector;
                }


            }
            /*
            if ( (vector1-rotationMat*vector2).dot(dVector)>0)
            {
                dVector = -dVector;
            }
            */
        }

       cout << "countMaxF2F " << countMax << " points " <<  numkeypoints<<endl;
      

        /*
        cout << "optimal Distance = " <<scale*optimalDistance << endl;
        cout << "error min = "<< errorMin << endl;
        cout << "gt distance" << TranslationResidual <<endl;
        */

        


        return optimalDistance ;


        
    }


    float VISystem::computeErrorGroup(vector <KeyPoint> inPoints1, vector <KeyPoint> inPoints2)
    {
        Point3d vector1; //Feature Vector en frame 1
        Point3d vector2; // Feature Vector en frame2
        Point3d normal;  // vector del plano 1 y 2 de cada pareja de features        

        float u1, v1, u2, v2;


        // Vector unitario de traslacion
        Point3d tVec = TranslationResidualGT;

 
        tVec = tVec/norm(tVec);


         int numkeypoints =  inPoints1.size();
       

        int count  = 0;
        int count_matches  = 0;
        Point3f dVector = translationResEst;
        float errorSum;
        // Crear vectores normales al plano de correspondecias (epipolar)ç
        dVector = dVector/norm(dVector);
        
        for (int i = 0; i< numkeypoints; i++)
        {

            count_matches++;
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

            normal = vector1.cross(Matx33d(RotationResCam)*vector2); // Vector normal al plano epipolar
            normal = normal/norm(normal); // normalizacion
            
            
            float error = -1000.0/std::log10(abs(dVector.dot(normal)));

            errorSum += error;

          

        }

        return errorSum/numkeypoints;
        

    }

   

    void VISystem::Triangulate(vector <KeyPoint> inPoints1, vector <KeyPoint> inPoints2, Matx33f prevR, Point3f prevt,  Matx33f currR, Point3f currt, vector <Point3f> &points3D)  
    {

        Mat P1 = getProjectionMat(K, Mat(prevR.t()), Mat(-prevR.t()*prevt ));
        Mat P2 = getProjectionMat(K, Mat(currR.t()), Mat(-currR.t()*currt ));


        vector<Point2f> src, dst;


        // Crear vectores
       
        //KeyPoint::convert( inPoints1 , points[0]);
        //KeyPoint::convert( inPoints2 , points[1]);

        for(int i = 0; i < inPoints1.size(); i++)
        {
            src.push_back(inPoints1[i].pt);
            dst.push_back(inPoints2[i].pt);
        }

                
        // Compute depth of 3D points using triangulation
        Mat mapPoints;
        
        
        
        if(inPoints1.size()!= 0)
        {
            Mat points4D;
            triangulatePoints(P1, P2, src, dst,  points4D);
           

            //Mat pt_3d; convertPointsFromHomogeneous(Mat(mapPoints.t()).reshape(4, 1),pt_3d);

            /*
            Vec3d rvec(0,0,0); //Rodrigues(R ,rvec);
            Vec3d tvec(0,0,0); // = P.col(3);
            vector<Point2f> reprojected_pt_set1;
            projectPoints(pt_3d,rvec,tvec, K, Mat(),reprojected_pt_set1);
            cout << " nr Triangulated " << inPoints1.size()<<endl;
            */
            
            for (int j = 0; j <inPoints1.size(); j++)
            {

        
                //int index = i;
                float u1 = inPoints1[j].pt.x;
                float v1 = inPoints1[j].pt.y;

                Point3f pt3d; // punto 3d

                pt3d.x = points4D.at<float>(0, j) / points4D.at<float>(3, j);
                pt3d.y = points4D.at<float>(1, j) / points4D.at<float>(3, j);
                pt3d.z = points4D.at<float>(2, j) / points4D.at<float>(3, j);
                //cout << "z" <<pt3d.z <<endl;

                points3D.push_back(pt3d);
                /*

                std::ostringstream strs;
                
                strs << fixed<< setprecision(2)<<pt3d.z;
                std::string str = strs.str();

                
                
                float x = pt_3d.at<float>(0,index);
                float y = pt_3d.at<float>(1,index);
                float z = pt_3d.at<float>(2,index);
                float z2 = sqrt(x*x+y*y+z*z);
             
                float errorx = u1 - reprojected_pt_set1[index].x;
                float errory = v1 - reprojected_pt_set1[index].y;
                float errorf = sqrt( errorx*errorx+errory*errory );
                
            
                putText(currentImageDebugToShow, str , Point2d(u1,v1 ), FONT_HERSHEY_SIMPLEX, 0.37,Scalar(10,255,10),0.9, LINE_AA);

                */
                
                
            }
        }

        

    }
     


    void VISystem::Track()
    {
        
        //current_poseCam = PAndR2T(RotationResCam , scale*translationResEst ); // residual
         current_poseCam = PAndR2T(RotationResCam , translationResEst  ); // residual
        

        
        //qOrientationImu = imuCore.quaternionWorld.back();
        final_poseCam = final_poseCam*current_poseCam;
        
        
        
 
        positionCam.x = final_poseCam(0, 3);
        positionCam.y = final_poseCam(1, 3);
        positionCam.z = final_poseCam(2, 3);
        
        
        RPYOrientationCam = transformationMatrix2RPY(final_poseCam);
        qOrientationCam = toQuaternion( RPYOrientationCam.x ,RPYOrientationCam.y, RPYOrientationCam.z);

        
        qOrientationImu = imuCore.quaternionWorld.back();
        
        

 
 

        
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

   

        float sx = TranslationResidualGT.x;
        float sy = TranslationResidualGT.y;
        float sz = TranslationResidualGT.z;
        //Matx33f Rotation_ResCam = imu2camRotation.t()* imuCore.residual_rotationMatrix*imu2camRotation;
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
        //WarpFunctionRT(_current_frame->prevGoodMatches,  R_out, t_out, warpedDebugKeyPoints);
        //WarpFunctionRT(_previous_frame->nextGoodMatches, Mat::eye(3, 3, CV_32FC1), trakity, warpedDebugKeyPoints);

         // WarpFunctionRT(_current_frame->prevGoodMatches, Mat::eye(3, 3, CV_32FC1), Mat::zeros(3, 1, CV_32FC1), warpedDebugKeyPoints, 1.0);
         //WarpFunctionRT(_current_frame->prevGoodMatches, Mat::eye(3, 3, CV_32FC1), Mat::zeros(3, 1, CV_32FC1), warpedDebugKeyPoints2, 10);
        
        float u1 = warpedDebugKeyPoints[0].pt.x;
        float v1 = warpedDebugKeyPoints[0].pt.y;
        float u2 = _previous_frame->nextGoodMatches[0].pt.x;
        float v2 = _previous_frame->nextGoodMatches[0].pt.y;
        cout << "u1 " << u1 << " v1 " <<  v1 << " u2 " << u2<< " v2 " << v2 <<endl;
        
        //drawKeypoints(_previous_frame->grayImage, warpedDebugKeyPoints, currentImageDebugToShow, Scalar(0,255, 0), DrawMatchesFlags::DEFAULT);
        drawKeypoints(_previous_frame->grayImage, _previous_frame->nextGoodMatches, currentImageDebugToShow, Scalar(255,0, 0), DrawMatchesFlags::DEFAULT);
        drawKeypoints(currentImageDebugToShow, _current_frame->prevGoodMatches, currentImageDebugToShow, Scalar(0,0, 255), DrawMatchesFlags::DEFAULT);
        drawKeypoints(_current_frame->grayImage, _current_frame->prevGoodMatches, currentImageToShow, Scalar(255,0, 0), DrawMatchesFlags::DEFAULT);

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




    Mat VISystem::getProjectionMat(Mat cameraMat, Mat rotationMat, Mat translationMat){
    // ProjectionMat = cameraMat * [Rotation | translation]
    Mat projectionMat;
    hconcat(rotationMat, translationMat, projectionMat);
 
    projectionMat = cameraMat * projectionMat;

    return projectionMat;
}




}





