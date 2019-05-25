#include <iostream>
#include <fstream>
#include "DataReader.hpp"
#include "Visualizer.hpp"
#include "VISystem.hpp"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"
#include <ctime>
#include <string>     // std::string, std::to_string
#include <ros/ros.h>
#include <ros/console.h>


using namespace cv;
using namespace std;
using namespace vi;

int main( int argc, char** argv ){
    
    const String keys =
        "{help h usage ? |      | print this message   }"
        "{gtFile     |       |  groundtruth file} "
        "{imagesPath   |       | data image directory} "
        "{imuFile   |       | data imu directory} "
        "{calibrationFile   |       | settings} "
        "{outputDirectory   |       | output directory} "
        "{pmvsBinary   |       | pmvs pmvsBinary} "
        "{startIndex   |       | index of images} "
        "{finalIndex   |       | index of images} ";

    cv::CommandLineParser parser(argc, argv, keys);
        if (parser.has("help"))
        {
            cout<< "parse the file path"<<endl;
            return 0;
        }
    string gtFile = parser.get<string>("gtFile");
    string imagesPath = parser.get<string>("imagesPath");
    string imuFile = parser.get<string>("imuFile");
    string calibrationFile = parser.get<string>("calibrationFile");
    string outputDirectory = parser.get<string>("outputDirectory");
    string pmvsBinary = parser.get<string>("pmvsBinary");
    string startIndex = parser.get<string>("startIndex");
    string finalIndex = parser.get<string>("finalIndex");
    char separator = ',';

    // Leer Dataset
    cout << "Gt file "<< imuFile<<endl;
    DataReader Data(imagesPath, imuFile, gtFile, separator);

    int i = stoi(startIndex);   
    int j = i+1;
    
    Data.UpdateDataReader(i, j);

    VISystem visystem(argc, argv);
    visystem.setConfig(calibrationFile);
    visystem.setPmvsBinary(pmvsBinary);
    visystem.setOutputDirectory(outputDirectory);
    //visystem.setInitPose(Data.gtPosition.back(), Data.gtRPY.back().z);
    visystem.setInitPose(Data.gtPosition.back(), Data.gtQuaternion.back());
   
    visystem.setInitData( Data.image2, Data.imuAngularVelocity, Data.imuAcceleration);



    // Visualizadores en ROS
    VisualizerPointcloud pointcloud("world", "pointcloudVISLAM", 1200);
    VisualizerPose poseCamEst("world", "CamEst", 1200);
    VisualizerPose poseCamGt("world", "CamGt", 1200);

    Point3f error;
    double position[3];
    double orientation[4];

    double position2[3];
    double orientation2[4];

   

    
    //Gt para camara 
    Quaterniond qOrientationCamGT;
    Point3d RPYOrientationCamGT,RPYOrientationCamGTprev;
    Point3d positionCamGT, positionCamGTprev;
    Point3d velocityCamGT;




    std::ofstream outputFilecsv;

    

    
    positionCamGTprev = Data.gtPosition.back()+visystem.imu2camTranslation;
    RPYOrientationCamGTprev = rotationMatrix2RPY(RPY2rotationMatrix(toRPY(Data.gtQuaternion.back()) )*visystem.imu2camRotation);
    cout << "RPY orientation cam " << RPYOrientationCamGTprev*180/M_PI <<endl;
    
    outputFilecsv.open(outputDirectory+"/outputVISlam.csv", std::ofstream::out | std::ofstream::trunc);

    clock_t begin1= clock(); 

    int finalImageIndex = Data.indexLastData;
    if (finalImageIndex>stod(finalIndex)) finalImageIndex = stoi(finalIndex);
    while(j < finalImageIndex) // j <data.lastindex
    {  // Cambiar por constant
        
        Data.UpdateDataReader(j, j+1);
        j = j+1;
         
        positionCamGT = Data.gtPosition.back()+visystem.imu2camTranslation;
        
        velocityCamGT  = Data.gtLinearVelocity.back();
        RPYOrientationCamGT =rotationMatrix2RPY(RPY2rotationMatrix(toRPY(Data.gtQuaternion.back()) )*visystem.imu2camRotation);
        Matx44d transformationResidual = RPYAndPosition2transformationMatrix(RPYOrientationCamGTprev, positionCamGTprev).inv()*RPYAndPosition2transformationMatrix(RPYOrientationCamGT, positionCamGT);


        Matx33f rotationResidualGt = transformationMatrix2rotationMatrix(transformationResidual);
        
        qOrientationCamGT = toQuaternion(RPYOrientationCamGT.x, RPYOrientationCamGT.y, RPYOrientationCamGT.z);
        
        

        Point3f residualTrans;
        residualTrans.x = transformationResidual(0,3);
        residualTrans.y = transformationResidual(1,3);
        residualTrans.z = transformationResidual(2,3);
 
        
        visystem.setGtTras(residualTrans);
        visystem.setGtRot(rotationResidualGt);
        bool disparityFound =  visystem.AddFrame(Data.image2, Data.imuAngularVelocity, Data.imuAcceleration);

       


         
        cout<< " Current time = "<< Data.currentTimeMs <<" ms " <<endl;
   

        
     
        
        if (disparityFound)
        {
           
            outputFilecsv <<  Data.currentTimeMs/1000.0 <<  ","  // indice de tiempo
            << visystem.positionCam.x<<","
            <<visystem.positionCam.y<<","
            <<visystem.positionCam.z<<","
            <<visystem.velocityCam.x<<","
            <<visystem.velocityCam.y<<","
            <<visystem.velocityCam.z<<","
            <<visystem.accCam.x<<","
            <<visystem.accCam.y<<","
            <<visystem.accCam.z<<","
            <<visystem.qOrientationCam.x <<","
            <<visystem.qOrientationCam.y <<","
            <<visystem.qOrientationCam.z <<","
            <<visystem.qOrientationCam.w <<","
            <<  positionCamGT.x <<","
            <<  positionCamGT.y <<","
            <<  positionCamGT.z <<","
            <<  velocityCamGT.x <<","
            <<  velocityCamGT.y <<","
            <<  velocityCamGT.z <<","
            <<  qOrientationCamGT.x <<","        
            <<  qOrientationCamGT.y <<","
            <<  qOrientationCamGT.z <<","
            <<  qOrientationCamGT.w <<","
            <<visystem.imuCore.angularVelocityIMUFilter.back().x<<","
            <<visystem.imuCore.angularVelocityIMUFilter.back().y<<","
            <<visystem.imuCore.angularVelocityIMUFilter.back().z
            <<endl;
            
            /*
        outputFilecsv <<  Data.currentTimeMs/1000.0 <<  ","  // indice de tiempo
        <<  visystem.positionImu.x<<","
        <<visystem.positionImu.y<<","
        <<visystem.positionImu.z<<","
        <<visystem.velocityImu.x<<","
        <<visystem.velocityImu.y<<","
        <<visystem.velocityImu.z<<","
        <<visystem.accImu.x<<","
        <<visystem.accImu.y<<","
        <<visystem.accImu.z<<","
        <<visystem.qOrientationImu.x <<","
        <<visystem.qOrientationImu.y <<","
        <<visystem.qOrientationImu.z <<","
        <<visystem.qOrientationImu.w <<","
        <<  Data.gtPosition.back().x <<","
        <<  Data.gtPosition.back().y <<","
        <<  Data.gtPosition.back().z <<","
        <<  Data.gtLinearVelocity.back().x <<","
        <<  Data.gtLinearVelocity.back().y <<","
        << Data.gtLinearVelocity.back().z<<","
        <<  Data.gtQuaternion.back().x <<","        
        <<  Data.gtQuaternion.back().y <<","
        <<  Data.gtQuaternion.back().z<<","
        <<  Data.gtQuaternion.back().w <<","
        <<visystem.imuCore.angularVelocityIMUFilter.back().x<<","
        <<visystem.imuCore.angularVelocityIMUFilter.back().y<<","
        <<visystem.imuCore.angularVelocityIMUFilter.back().z
        <<endl;
        
        */
    
            

            // Actualizar mensajes
           
            poseCamGt.UpdateMessages(positionCamGT, qOrientationCamGT);
            poseCamEst.UpdateMessages(visystem.positionCam, visystem.qOrientationCam);


            // Update pointcloud
            vector <Point3f> landmarks;
            visystem.getLandmarks(landmarks);
            pointcloud.UpdateMessages(landmarks);
                    

            positionCamGTprev =  positionCamGT;
            RPYOrientationCamGTprev =RPYOrientationCamGT ;
              char c_input = (char) waitKey(1);
                if( c_input == 'q' | c_input == ((char)27) )  {
                        
                        break;
                }
                if( c_input == 'k'  )  {
                  
                   
                }
            
            

        }

        

      


      
            
           
    }
    visystem.shutdown();


       


        


    
    

    clock_t detect1 = clock(); 
    double elapsed_detect = double(detect1- begin1) / CLOCKS_PER_SEC;  

    cout << "elapsed Final = " << elapsed_detect*1000<< " ms" <<endl; 


    return 0;

}

// COMPILE COMMAND
//g++ -g main_vi_slam.cpp GroundTruth.cpp -o main_vi_slam.out `pkg-config opencv --cflags --libs`

// RUN COMMAND
//./main_vi_slam.out -filePath=../../../../Documents/EuroDataset/state_groundtruth_estimate0/data.csv
// rosrun vi_slam vi_slam -filePath=../../../../Documents/EuroDataset/state_groundtruth_estimate0/data.csv 
