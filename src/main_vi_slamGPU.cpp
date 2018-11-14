#include <iostream>
#include "DataReader.hpp"
#include "Visualizer.hpp"
#include "CameraGPU.hpp"
#include "Imu.hpp"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"
#include <ctime>

#include <ros/ros.h>
#include <ros/console.h>

#include <std_msgs/Int32.h>
#include <geometry_msgs/Vector3.h>

using namespace cv;
using namespace std;

int main( int argc, char** argv ){
    
    const String keys =
        "{help h usage ? |      | print this message   }"
        "{filePath     |       | data input directory} "
        "{imagePath   |       | data image directory} "
        "{imuPath   |       | data imu directory} ";

    cv::CommandLineParser parser(argc, argv, keys);
        if (parser.has("help"))
        {
            cout<< "parse the file path"<<endl;
            return 0;
        }
    string filePath = parser.get<string>("filePath");
    string imagePath = parser.get<string>("imagePath");
    string imuPath = parser.get<string>("imuPath");
    char separator = ',';


    // Detecting CUDA Device
    int nCuda = cuda::getCudaEnabledDeviceCount();
    cuda::DeviceInfo deviceInfo;
    if (nCuda > 0){
        std::cout << "CUDA enabled devices detected: " << deviceInfo.name() << endl;
        cuda::setDevice(0);
    }
    else {
        std::cout << "No CUDA device detected" << endl;
        return -1;
    }
    std::cout << "***************************************" << endl;

    cout<<"File "<<filePath<<endl;

    DataReader Data(imagePath, imuPath, filePath, separator);

    /*
    GroundTruth groundTruth(filePath, separator);

    cout<< groundTruth.getFileName()<<"\n"
    << groundTruth.getCharSeparator()<<"\n"
    <<"número de filas"<<groundTruth.getRows()
    <<"número de columnas"<<groundTruth.getCols()<< endl;

    cout <<"Data ="<< groundTruth.getGroundTruthData(0, 1)<<endl;;
    ImageReader imageReader(imagePath);
    imageReader.searchImages();
    */

    ros::init(argc, argv, "vi_slam");  // Initialize ROS

    //VisualizerVector3 rqt_error("error", 10.0);
    VisualizerMarker visualizer_gt("gt_poses", "/my_frame", 2000, CUBE, 0, Point3f(1.0, 1.0, 1.0),Point3f(0.0, 0.0, 1.0));
    VisualizerMarker visualizer_est("est_poses", "/my_frame", 2000, CUBE, 0, Point3f(1.0, 1.0, 1.0),Point3f(0.0, 1.0, 0.0));
    VisualizerFrame visualizerFrame("current_frame", 90);
    VisualizerFrame visualizerFrame2("current_frame2", 90);
    VisualizerVector3 velocidad_groundtruth("velocidad_groundtruth", 1200);
    VisualizerVector3 velocidad_estimado("velocidad_estimado", 1200);
    Mat frame1;
    Mat frame2;
    Point3f error;
    double position[3];
    double orientation[4];

    double position2[3];
    double orientation2[4];
    orientation[3] = 1.0;

    Imu imuCore(Data.timeStepImu/1000000000.0);
    imuCore.setImuInitialVelocity();

    //-- Paso 4: Calcular la matriz Esencial
    // Parametros intrisecos de la camara
    double fx, fy, focal, cx, cy;
    fx = 458.654;
    fy = 457.296;
    cx =  367.215;
    cy = 248.375;
    /*
    fx = 7.188560000000e+02;

    fy = 7.188560000000e+02;
    cx =  6.071928000000e+02;
    cy =  1.852157000000e+02;
    */
    focal = fx;
    Mat E, R, t; // matriz esencial

  
    std::vector<Point2f> points1_OK, points2_OK; // Puntos finales bajos analisis
    std::vector<Point2f> aux_points1_OK, aux_points2_OK;
    vector<int> point_indexs;
    Mat odometry = Mat::zeros(1, 3, CV_64F); // Matriz vacia de vectores de longitud 3 (comienza con 000)
    Mat R_p ; // matriz de rotacion temporal
    Mat traslation; 

    int i = 0;
    vector<KeyPoint> vectorMatches;
    Quaterniond qGt_initial;

    Data.UpdateDataReader(0);
    int n = Data.gtPosition.size()-1;
    position2[0] = Data.gtPosition[n].x;   //x
    position2[1] = Data.gtPosition[n].y;   //x
    position2[2] = Data.gtPosition[n].z;
    qGt_initial.x = Data.gtQuaternion[n].x;   //x
    qGt_initial.y = Data.gtQuaternion[n].y; // y
    qGt_initial.z = Data.gtQuaternion[n].z; // z
    qGt_initial.w = Data.gtQuaternion[n].w; // w
    Point3d angle_gt_initial = toRPY(qGt_initial);
    imuCore.setImuData(Data.imuAngularVelocity, Data.imuAcceleration); // primeras medidas
    imuCore.initializate(angle_gt_initial.z); // Poner yaw inicial del gt

    
    Point3d velocity ;//= Data.gtLinearVelocity[9];
    velocity.x = 0.0;
    velocity.y = 0.0;
    velocity.z = 0.0;
    
    CameraGPU camera(USE_SURF, USE_BRUTE_FORCE_GPU, Data.image1.cols, Data.image1.rows);
    for (int j = 0;  j <Data.indexLastData; j++)
    {  // Cambiar por constante
        Mat finalImage, finalImage2;
        Data.UpdateDataReader(j);
 

        camera.Update(Data.image1);
        camera.addGPUKeyframe();
        if (camera.frameList.size()> 1) // primera imagen agregada
        {
            
        //drawKeypoints( frame1, aux, frame1, Scalar(0, 0, 255), DrawMatchesFlags::DEFAULT);
        drawKeypoints(camera.frameList[camera.frameList.size()-2]->grayImage, camera.frameList[camera.frameList.size()-2]->nextGoodMatches , finalImage, Scalar(0,0, 255), DrawMatchesFlags::DEFAULT);
        drawKeypoints(camera.frameList[camera.frameList.size()-1]->grayImage, camera.frameList[camera.frameList.size()-1]->prevGoodMatches, finalImage2, Scalar(0,0, 255), DrawMatchesFlags::DEFAULT);
        visualizerFrame.UpdateMessages(finalImage);
        visualizerFrame2.UpdateMessages(finalImage2);
        //imwrite("/home/lujano/Documents/Imagen1.png", finalImage);
        //imwrite("/home/lujano/Documents/Imagen2.png", finalImage2);
       
         /*
        E = findEssentialMat(points1_OK, points2_OK, focal, Point2d(cx, cy), RANSAC, 0.999, 1.0, noArray());
        int p;
        p = recoverPose(E, points1_OK, points2_OK, R, t, focal, Point2d(cx, cy), noArray()   );
        int k = 0;
        if (j == 0){
            traslation = Mat::zeros(3, 1, CV_64F);
            R_p = Mat::eye(Size(3, 3), CV_64F);
        }
        else{
            traslation = traslation +R_p*t;
            R_p = R*R_p;
            */
        }

       
        
      
       


        Point3d angle_diff;
        Point3d angle_gt, angle_ft, gravity_imu;
        double elapsed_f;
        
        
        imuCore.setImuData(Data.imuAngularVelocity, Data.imuAcceleration);
        clock_t t1 = clock(); 
        imuCore.setImuBias(Data.accBias, Data.angBias);
        imuCore.estimate(Data.gtRPY);
        imuCore.printStatistics();
        //imuCore.printStatistics(); // imprime el tiempo de computo del filtro
        
        
        
        for ( int ii = 0 ; ii< 10;ii++)
        {
            orientation2[0] = imuCore.quaternionWorld[ii].x;
            orientation2[1] = imuCore.quaternionWorld[ii].y;
            orientation2[2] = imuCore.quaternionWorld[ii].z;
            orientation2[3] = imuCore.quaternionWorld[ii].w;
            position2[0] = Data.gtPosition[ii].x;   //x
            position2[1] = Data.gtPosition[ii].y;   //x
            position2[2] = Data.gtPosition[ii].z;   //x
         
            position[0] = Data.gtPosition[ii].x;   //x
            position[1] = Data.gtPosition[ii].y;   //x
            position[2] = Data.gtPosition[ii].z;   //x
            orientation[0] = Data.gtQuaternion[ii].x;   //x
            orientation[1] = Data.gtQuaternion[ii].y; // y
            orientation[2] = Data.gtQuaternion[ii].z; // z
            orientation[3] = Data.gtQuaternion[ii].w; // w
           

           

            angle_diff.x = computeDiff(Data.gtRPY[ii].x, imuCore.rpyAnglesWorld[ii].x);
            angle_diff.y = computeDiff(Data.gtRPY[ii].y, imuCore.rpyAnglesWorld[ii].y);
            angle_diff.z = computeDiff(Data.gtRPY[ii].z, imuCore.rpyAnglesWorld[ii].z);

            angle_diff = angle_diff*180/M_PI;

            Point3d f_angles = toRPY360(imuCore.rpyAnglesWorld[ii]); // pasar a representacion 0-360
            Point3d gt_angles = toRPY360(Data.gtRPY[ii]);
            //angulo_estimado.UpdateMessages(f_angles*180/M_PI );
            //angulo_groundtruth.UpdateMessages(gt_angles*180/M_PI );
            //error_angular.UpdateMessages(angle_diff);
            visualizer_gt.UpdateMessages(position, orientation);
            visualizer_est.UpdateMessages(position2, orientation2);
            
            velocity = velocity+imuCore.velocity;

                
            //vector3d.UpdateMessages(acc_diff);
                
        }
        velocidad_groundtruth.UpdateMessages(Data.gtLinearVelocity[9]);
        velocidad_estimado.UpdateMessages(velocity/10);
        clock_t t2 = clock(); 
        double elapsed_time= double(t2- t1) / CLOCKS_PER_SEC;
        /*
        cout << " diffx = " << imuCore.accelerationWorld[9].x
        <<" diffy = "<<imuCore.accelerationWorld[9].y
        <<" diffz = "<<imuCore.accelerationWorld[9].z
        << " Current time = "<< Data.currentTimeMs <<" ms "
        << "accx = "  << Data.accBias.x
        << "timestep = "<< imuCore.timeStep

        
        
	<<endl;*/
        



        

        
        
      
       

       
        
       
        
        
     }
    
    


  

    return 0;

}

// COMPILE COMMAND
//g++ -g main_vi_slam.cpp GroundTruth.cpp -o main_vi_slam.out `pkg-config opencv --cflags --libs`

// RUN COMMAND
//./main_vi_slam.out -filePath=../../../../Documents/EuroDataset/state_groundtruth_estimate0/data.csv
// rosrun vi_slam vi_slam -filePath=../../../../Documents/EuroDataset/state_groundtruth_estimate0/data.csv 