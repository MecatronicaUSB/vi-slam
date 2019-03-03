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
        cout << "Yaw inicial = " <<_iniYaw<<endl;
        cout << "Pos = " <<positionImu<<endl; 
       
        

     

    }

    void VISystem::setInitData(Mat image,  vector <Point3d> _imuAngularVelocity, vector <Point3d> _imuAcceleration)
    {

        currentImage = image;
        // Colocar medidas iniciales de IMU
        Point3d _iniVelocity = Point3d(0.0, 0.0, 0.0); // Asumir velocidad inicial cero
        imuCore.initializate(iniYaw, _imuAngularVelocity, _imuAcceleration); // Poner yaw inicial del gt
             
        camera.Update(currentImage);
        nPointsLastKeyframe = camera.detectAndComputeFeatures(); // asumir que la primera imagen es buena y es keyframe
        lastImageWasKeyframe = true;
        
        camera.saveFrame();
        cout << "Camera Initialized con "<< camera.frameList.back()->keypoints.size() << " features"<<endl;

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
        
       
        
        prevImage = currentImage;
        currentImage = _currentImage.clone();
       
        
        imuCore.setImuData(_imuAngularVelocity, _imuAcceleration); // Medidas tomadas
        imuCore.estimate();
        velocityCam = velocityCam+imuCore.residualVelocity; // velocidad igual a la de la imu
        velocityImu = velocityCam;
        accCam = imuCore.accelerationWorld.back(); // acceleration igual a la de la imu
     



        // Analizar imagen nueva
        camera.Update(_currentImage);
        nPointsCurrentImage = camera.detectAndComputeFeatures();
        
        
    

        currentImageIsKeyframe = false;

        if (nPointsCurrentImage > int(nPointsLastKeyframe*0.35)) // se considera que es hay suficientes puntos para el match
        {
            camera.computeFastMatches(); // se calcula el match entre la imagen actual y el ultimo keyframe
           
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
             float disparityThreshold = 22;
             double disparityAngThreshold = 5.0;

            double disparityAng = 180/M_PI*sqrt(rpyResidual.x*rpyResidual.x+rpyResidual.y*rpyResidual.y+rpyResidual.z*rpyResidual.z);
            float disparity = Disparity(camera.frameList.back()->nextGoodMatches, camera.currentFrame->prevGoodMatches);

            cout << "Disparity Trans = " << disparity <<endl;
            cout << "Disparity Ang = " << disparityAng <<endl;
            
            if (disparity > disparityThreshold) currentImageIsKeyframe = true;   
            if (disparityAng > disparityAngThreshold) currentImageIsKeyframe = true;
            
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
                //Triangulate(camera.frameList[camera.frameList.size()-2]->nextGoodMatches, camera.frameList[camera.frameList.size()-1]->prevGoodMatches);

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
    
 

    void VISystem::FreeLastFrame()
    {
        camera.frameList[0]->~Frame();
        camera.frameList.erase(camera.frameList.begin());
    }
    
    // Lujano Algorithm
    void VISystem::setGtTras(Point3d TranslationResGT )
    {
        TranslationResidualGT = TranslationResGT;
 
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

        
        Point3d rotationEst = rotationMatrix2RPY(RotationResCam)*180/M_PI;

        

        
        

        
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
        
        
        drawKeypoints(_previous_frame->grayImage, _previous_frame->nextGoodMatches, currentImageDebugToShow, Scalar(255,0, 0), DrawMatchesFlags::DEFAULT);
        //drawKeypoints(currentImageDebugToShow, warpedDebugKeyPoints, currentImageDebugToShow, Scalar(0,255, 0), DrawMatchesFlags::DEFAULT);
        drawKeypoints(currentImageDebugToShow, _current_frame->prevGoodMatches, currentImageDebugToShow, Scalar(0,0, 255), DrawMatchesFlags::DEFAULT);
        drawKeypoints(_current_frame->grayImage, _current_frame->prevGoodMatches, currentImageToShow, Scalar(0,0, 255), DrawMatchesFlags::DEFAULT);

      

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


        Point3d vector1; //Feature Vector en frame 1 
        Point3d vector2; // Feature Vector en frame2
        Point3d normal;  // vector del plano 1 y 2 de cada pareja de features


        int numkeypoints = inPoints1.size();
        Point3f pont;
        

        float u1, v1, u2, v2;
        float u1k, v1k, u2k, v2k; // key

        // Vector unitario de traslacion
        Point3d tVec = TranslationResidualGT;

 
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

            normal = vector1.cross(Matx33d(RotationResCam)*vector2); // Vector normal al plano epipolar
            
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

        
        //double thresholdError; //
        //thresholdError = 350.0;
    

        Point3f vector1, vector1k; //Feature Vector en frame 1
        Point3f vector2, vector2k; // Feature Vector en frame2
        Point3f normal, n1, n2;  // vector del plano 1 y 2 de cada pareja de features
        Point3f d, dnorm; // vector de dezplazamiento y normalizado

        int numkeypoints = inPoints1.size();
        Point3f pont;
        

        float u1, v1, u2, v2;
        float u1k, v1k, u2k, v2k; // key

        vector <Point3f> normalVectors;
        Mat degenerate = Mat::zeros(1, numkeypoints, CV_32F) ; // vector la degeneracion de los vectores

        float sx = TranslationResidualGT.x;
        float sy = TranslationResidualGT.y;
        float sz = TranslationResidualGT.z;
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

            normal = vector1.cross(rotationMat*vector2); // Vector normal al plano epipolar
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

       cout << "countMax " << countMax << " points " <<  numkeypoints<<endl;


        /*
        cout << "optimal Distance = " <<scale*optimalDistance << endl;
        cout << "error min = "<< errorMin << endl;
        cout << "gt distance" << TranslationResidual <<endl;
        */

        


        return scale*optimalDistance ;


        
    }

   

    void VISystem::Triangulate(vector <KeyPoint> inPoints1, vector <KeyPoint> inPoints2) 
    {


        Mat P1 = getProjectionMat(K, Mat::eye(3, 3, CV_32FC1), Mat::zeros(3, 1, CV_32FC1));
        Mat P2 = getProjectionMat(K, Mat(RotationResCam.t()), Mat(-RotationResCam.t()*Point3f(TranslationResidualGT)) );


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
            projectPoints(pt_3d,rvec,tvec, K, Mat(),reprojected_pt_set1);
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
     


    void VISystem::Track()
    {
        
        current_poseCam = PAndR2T(RotationResCam , translationResEst ); // residual
        

        
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





