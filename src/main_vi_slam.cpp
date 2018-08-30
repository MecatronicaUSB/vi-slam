#include <iostream>
#include "../include/GroundTruth.h"
#include "Visualizer.h"
#include "opencv2/core.hpp"

int main( int argc, char** argv ){
    
    const String keys =
        "{help h usage ? |      | print this message   }"
        "{filePath     |       | data input directory} ";

    cv::CommandLineParser parser(argc, argv, keys);
        if (parser.has("help"))
        {
            cout<< "parse the file path"<<endl;
            return 0;
        }
    string filePath = parser.get<string>("filePath");
    char separator = ',';
    GroundTruth groundTruth(filePath, separator);

    cout<< groundTruth.getFileName()<<"\n"
    << groundTruth.getCharSeparator()<<"\n"
    <<"número de filas"<<groundTruth.getRows()
    <<"número de columnas"<<groundTruth.getCols()<< endl;


    double position[3];
    double orientation[4];
    orientation[3] = 1.0;

    ros::init(argc, argv, "vi_slam");  // Initialize ROS

    Visualizer visualizer("gt_poses", "/my_frame", 1000.0, 1);
    

    for (int j = 0;  j < groundTruth.getRows(); j++){  
        position[0] = groundTruth.getGroundTruthData(j, 1);   //x
        position[1] = groundTruth.getGroundTruthData(j, 2);   //y
        position[2] = groundTruth.getGroundTruthData(j, 3);   // z
        orientation[1] = groundTruth.getGroundTruthData(j, 4); // x
        orientation[2] = groundTruth.getGroundTruthData(j, 5); // y
        orientation[3] = groundTruth.getGroundTruthData(j, 6); // z
        orientation[0] = groundTruth.getGroundTruthData(j, 7); // w
        visualizer.UpdateMessages(position, orientation);
    }
    

    return 0;

}

// COMPILE COMMAND
//g++ -g main_vi_slam.cpp GroundTruth.cpp -o main_vi_slam.out `pkg-config opencv --cflags --libs`

// RUN COMMAND
//./main_vi_slam.out -filePath=../../../../Documents/EuroDataset/state_groundtruth_estimate0/data.csv
// rosrun vi_slam vi_slam -filePath=../../../../Documents/EuroDataset/state_groundtruth_estimate0/data.csv 