#include <iostream>
#include "GroundTruth.hpp"
#include "Visualizer.hpp"
#include "ImageReader.hpp"
#include "Matcher.hpp"
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
        "{imagePath   |       | data image directory} ";

    cv::CommandLineParser parser(argc, argv, keys);
        if (parser.has("help"))
        {
            cout<< "parse the file path"<<endl;
            return 0;
        }
    string filePath = parser.get<string>("filePath");
    string imagePath = parser.get<string>("imagePath");
    char separator = ',';

    cout<<"File "<<filePath<<endl;
    GroundTruth groundTruth(filePath, separator);

    cout<< groundTruth.getFileName()<<"\n"
    << groundTruth.getCharSeparator()<<"\n"
    <<"número de filas"<<groundTruth.getRows()
    <<"número de columnas"<<groundTruth.getCols()<< endl;

    ImageReader imageReader(imagePath);

    

    ros::init(argc, argv, "vi_slam");  // Initialize ROS

    //VisualizerVector3 rqt_error("error", 10.0);
    VisualizerMarker visualizer_gt("gt_poses", "/my_frame", 1000.0, CUBE, 0, Point3f(1.0, 1.0, 1.0),Point3f(0.0, 0.0, 1.0));
    VisualizerMarker visualizer_est("est_poses", "/my_frame", 1000.0, CUBE, 0, Point3f(1.0, 1.0, 1.0),Point3f(0.0, 1.0, 0.0));
    VisualizerFrame visualizerFrame("current_frame", 100);
    VisualizerFrame visualizerFrame2("current_frame2", 100);
    Mat frame1;
    Mat frame2;
    Point3f error;
    double position[3];
    double orientation[4];
    orientation[3] = 1.0;

    
    //

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
    Mat finalImage, finalImage2;
    Mat odometry = Mat::zeros(1, 3, CV_64F); // Matriz vacia de vectores de longitud 3 (comienza con 000)
    Mat R_p ; // matriz de rotacion temporal
    Mat traslation; 

    int i = 0;
    vector<KeyPoint> vectorMatches;

    
    for (int j = 0;  j < imageReader.getSize()-1; j++)
    {  // Cambiar por constante
     Mat finalImage, finalImage2;
        vector<KeyPoint> matched1, matched2;
        vector<KeyPoint> aux;
        
        Matcher matcher(USE_SIFT, USE_BRUTE_FORCE);
        frame1 = imageReader.getImage(j);
        frame2 = imageReader.getImage(j+1);
        
        clock_t begin = clock(); // Tiempo de inicio del codigo
        matcher.setFrames(frame1, frame2);
        
        matcher.computeSymMatches();
        clock_t comp = clock(); 
        matcher.sortMatches();
        clock_t sort = clock(); 
        int n_features = matcher.bestMatchesFilter(144);
        clock_t best = clock(); 
        matcher.getGoodMatches(matched1, matched2);
        clock_t get = clock(); 

        double elapsed_comp = double(comp- begin) / CLOCKS_PER_SEC;
        double elapsed_sort = double(sort- comp) / CLOCKS_PER_SEC;
        double elapsed_best = double(best- sort) / CLOCKS_PER_SEC;
        double elapsed_get = double(get- best) / CLOCKS_PER_SEC;

        cout<<"comp, sort, best, get = \n"
        <<fixed<<elapsed_comp<<"\t"<<elapsed_sort<<"\t"<<elapsed_best<<"\t"<<elapsed_get<<endl;

        matcher.getGrid(144 , aux);

        cv::KeyPoint::convert(matched1, points1_OK,point_indexs);
        cv::KeyPoint::convert(matched2, points2_OK, point_indexs);
        std::cout << "Puntos totales = "<<matched1.size()<< endl;
        //drawKeypoints( frame1, aux, frame1, Scalar(0, 0, 255), DrawMatchesFlags::DEFAULT);
        drawKeypoints( frame1, matched1, finalImage, Scalar(255,0, 0), DrawMatchesFlags::DEFAULT);
        drawKeypoints( frame2, matched2, finalImage2, Scalar(255,0, 0), DrawMatchesFlags::DEFAULT);
        imwrite("/home/lujano/Documents/Imagen1.png", finalImage);
        imwrite("/home/lujano/Documents/Imagen2.png", finalImage2);
        visualizerFrame.UpdateMessages(finalImage);
        visualizerFrame2.UpdateMessages(finalImage2);
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

        
        position[0] = traslation.at<double>(0,0);   //x
        position[1] = traslation.at<double>(2,0);   //y
        position[2] = traslation.at<double>(1,0);
        orientation[0] = 1.0;
        
        visualizer_est.UpdateMessages(position, orientation);
        
       */
      /*
        for (int j = i;  j < i+10; j++)
        {  
            position[0] = groundTruth.getGroundTruthData(j, 1);   //x
            position[1] = groundTruth.getGroundTruthData(j, 2);   //y
            position[2] = groundTruth.getGroundTruthData(j, 3);   // z
            orientation[1] = groundTruth.getGroundTruthData(j, 4); // x
            orientation[2] = groundTruth.getGroundTruthData(j, 5); // y
            orientation[3] = groundTruth.getGroundTruthData(j, 6); // z
            orientation[0] = groundTruth.getGroundTruthData(j, 7); // w
            visualizer_gt.UpdateMessages(position, orientation);
            
        }
        i = i+10;
       */
        
        
     }
    
    


  

    return 0;

}

// COMPILE COMMAND
//g++ -g main_vi_slam.cpp GroundTruth.cpp -o main_vi_slam.out `pkg-config opencv --cflags --libs`

// RUN COMMAND
//./main_vi_slam.out -filePath=../../../../Documents/EuroDataset/state_groundtruth_estimate0/data.csv
// rosrun vi_slam vi_slam -filePath=../../../../Documents/EuroDataset/state_groundtruth_estimate0/data.csv 