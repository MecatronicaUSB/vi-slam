#include <iostream>
#include "DataReader.hpp"
#include "Visualizer.hpp"
#include "Matcher.hpp"
#include "Imu.hpp"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"
#include <ctime>

#include <ros/ros.h>
#include <ros/console.h>

#include <std_msgs/Int32.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>

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
    VisualizerMarker visualizer_gt("gt_poses", "/my_frame", 700, CUBE, 0, Point3f(1.0, 1.0, 1.0),Point3f(0.0, 0.0, 1.0));
    VisualizerMarker visualizer_est("est_poses", "/my_frame", 700, CUBE, 0, Point3f(1.0, 1.0, 1.0),Point3f(0.0, 1.0, 0.0));
    //VisualizerFrame visualizerFrame("current_frame", 90);
    //VisualizerFrame visualizerFrame2("current_frame2", 90);
    VisualizerVector3 vector3d ("vector3d", 700);
    ImuFilter imuFilter(1000);
    imuFilter.createROSSubscriber();
    Mat frame1;
    Mat frame2;
    Point3f error;
    double position[3];
    double orientation[4];

    double position2[3];
    double orientation2[4];
    orientation[3] = 1.0;

    Imu imuCore(Data.timeStepImu/1000000000);

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
    
    Data.UpdateDataReader(0);
    position2[0] = Data.gtPosition[Data.gtPosition.size()-1].x;   //x
    position2[1] = Data.gtPosition[Data.gtPosition.size()-1].y;   //x
    position2[2] = Data.gtPosition[Data.gtPosition.size()-1].z;

    for (int j = 0;  j <Data.indexLastData; j++)
    {  // Cambiar por constante
        Mat finalImage, finalImage2;
        Data.UpdateDataReader(j);
       
        //imuCore.setImuData(Data.imuAngularVelocity, Data.imuAcceleration);
        
       
        vector<KeyPoint> matched1, matched2;
        vector<KeyPoint> aux1, aux2, grid;
        
        Matcher matcher(USE_SIFT, USE_BRUTE_FORCE);
        frame1 = Data.image1;
        frame2 = Data.image2;
        /*
        matcher.setFrames(frame1, frame2);
        matcher.detectFeatures();
        matcher.computeMatches();
        matcher.computeBestMatches(12*12);
        matcher.printStatistics();

        matcher.getGoodMatches(matched1, matched2);
       // cout<<"Size"<<Data.imuAcceleration.size()<<endl;

        cv::KeyPoint::convert(matched1, points1_OK,point_indexs);
        cv::KeyPoint::convert(matched2, points2_OK, point_indexs);
      
        //drawKeypoints( frame1, aux, frame1, Scalar(0, 0, 255), DrawMatchesFlags::DEFAULT);
        */
        drawKeypoints( frame1, matched1, finalImage, Scalar(0,0, 255), DrawMatchesFlags::DEFAULT);
        drawKeypoints( frame2, matched2, finalImage2, Scalar(0,0, 255), DrawMatchesFlags::DEFAULT);
        //imwrite("/home/lujano/Documents/Imagen1.png", finalImage);
        //imwrite("/home/lujano/Documents/Imagen2.png", finalImage2);
       
        

       // visualizerFrame.UpdateMessages(finalImage);
        //visualizerFrame2.UpdateMessages(finalImage2);
        
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
        }

        
        
        position2[0] = traslation.at<double>(0,0);   //x
        position2[1] = traslation.at<double>(2,0);   //y
        position2[2] = traslation.at<double>(1,0);
        orientation2[0] = 1.0;
        */
        //cout << "position x" << imuCore.position.x<<endl;
        //cout << "position y" << imuCore.position.y<<endl;
        //cout << "position z" << imuCore.position.z<<endl;
        Point3d angle_diff;
        Point3d angle_gt, angle_ft;
        for (int ii = 0 ; ii< Data.gtQuaternion.size();ii++)
        {
            Quaterniond qFilter;
            Quaterniond qGt;
            
            //imuCore.estimate();
            imuFilter.UpdatePublisher(Data.imuAngularVelocity[ii], Data.imuAcceleration[ii]);
            position2[0] = Data.gtPosition[ii].x;   //x
            position2[1] = Data.gtPosition[ii].y;   //x
            position2[2] = Data.gtPosition[ii].z;   //x
            imuFilter.UpdateSubscriber();
            qFilter.x = orientation2[1] = imuFilter.imuFusedData.orientation.x;   //x
           qFilter.y = orientation2[2] = imuFilter.imuFusedData.orientation.y;  
            qFilter.z = orientation2[3] = imuFilter.imuFusedData.orientation.z;  
           qFilter.w = orientation2[0] =imuFilter.imuFusedData.orientation.w;  
      
            visualizer_est.UpdateMessages(position2, orientation2);

         
        position[0] = Data.gtPosition[ii].x;   //x
        position[1] = Data.gtPosition[ii].y;   //x
        position[2] = Data.gtPosition[ii].z;   //x
        qGt.x = orientation[1] = Data.gtQuaternion[ii].x;   //x
        qGt.y = orientation[2] = Data.gtQuaternion[ii].y; // y
        qGt.z = orientation[3] = Data.gtQuaternion[ii].z; // z
        qGt.w = orientation[0] = Data.gtQuaternion[ii].w; // w
        visualizer_gt.UpdateMessages(position, orientation);

        
        
        angle_ft = toEulerAngle(qFilter);
        angle_gt = toEulerAngle(qGt);
        

        angle_diff.x = computeDiffAng(angle_gt.x, angle_ft.x);
        angle_diff.y = computeDiffAng(angle_gt.y, angle_ft.y);
        angle_diff.z = computeDiffAng(angle_gt.z-M_PI/2, angle_ft.z);

      
        vector3d.UpdateMessages(angle_diff);
            

            
        }

        cout << " diffx = " << angle_diff.x
        <<" diffy = "<<angle_diff.y
        <<" diffz = "<< angle_diff.z
        << " Current time = "<< Data.currentTimeMs <<" ms "
        <<endl;
        
        
        
      
       

       
        
        
     }
    
    


  

    return 0;

}

// COMPILE COMMAND
//g++ -g main_vi_slam.cpp GroundTruth.cpp -o main_vi_slam.out `pkg-config opencv --cflags --libs`

// RUN COMMAND
//./main_vi_slam.out -filePath=../../../../Documents/EuroDataset/state_groundtruth_estimate0/data.csv
// rosrun vi_slam vi_slam -filePath=../../../../Documents/EuroDataset/state_groundtruth_estimate0/data.csv 