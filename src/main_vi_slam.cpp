#include <iostream>
#include <fstream>
#include "DataReader.hpp"
#include "Visualizer.hpp"
#include "VISystem.hpp"
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
using namespace vi;

int main( int argc, char** argv ){
    
    const String keys =
        "{help h usage ? |      | print this message   }"
        "{gtFile     |       |  groundtruth file} "
        "{imagesPath   |       | data image directory} "
        "{imuFile   |       | data imu directory} "
        "{calibrationFile   |       | settings} "
        "{outputFile   |       | outputFile} ";

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
    string outputFile = parser.get<string>("outputFile");
    char separator = ',';

    // Leer Dataset
    cout << "Gt file "<< imuFile<<endl;
    DataReader Data(imagesPath, imuFile, gtFile, separator);
    Data.UpdateDataReader(0, 1);

    VISystem visystem(argc, argv);
    visystem.InitializeSystem( outputFile, "", calibrationFile, Data.gtPosition[0], Data.gtLinearVelocity[0], Data.gtRPY[0].z, Data.image1);
    cout << "Initializate System"<<endl;

    //VisualizerVector3 rqt_error("error", 10.0);
    
    VisualizerMarker visualizer_gt("gt_poses", "/my_frame", 2000, ARROW, 0, Point3f(1.0, 1.0, 1.0),Point3f(0.0, 0.0, 1.0));
    //VisualizerMarker visualizer_est("est_poses", "/my_frame", 2000, ARROW, 0, Point3f(1.0, 1.0, 1.0),Point3f(0.0, 1.0, 0.0));
    //VisualizerFrame visualizerFrame("current_frame", 90);
    //VisualizerFrame visualizerFrame2("current_frame2", 90);
    //VisualizerVector3 velocidad_groundtruth("velocidad_groundtruth", 1200);
    //VisualizerVector3 velocidad_estimado("velocidad_estimado", 1200);
    //VisualizerVector3 error_angular ("error_angular", 1200);
    //VisualizerVector3 error_velocidad ("error_velocidad", 1200);
    //VisualizerVector3 error_posicion ("error_posicion", 1200);
    //VisualizerVector3 posicion_estimada ("posicion_estimada", 1200);
    //VisualizerVector3 posicion_groundtruth ("posicion_gt", 1200);
    //VisualizerVector3 angulo_estimado ("angulo_estimado", 1200);
   // VisualizerVector3 residual_angEst ("residual_angEst", 1200);
    //VisualizerVector3 residual_angGt ("residual_angGt", 1200);
    //VisualizerVector3 angulo_groundtruth ("angulo_groundtruth",1200);
    Mat frame1;
    Mat frame2;
    Point3f error;
    double position[3];
    double orientation[4];

    double position2[3];
    double orientation2[4];
   
   
    vector<KeyPoint> vectorMatches;
    Quaterniond qGt_initial;


   
    int j = 210;
    std::ofstream outputFilecsv;;
    outputFilecsv.open("/home/lujano/Documents/outputVISlam.csv", std::ofstream::out | std::ofstream::trunc);
    while(j <Data.indexLastData)
    {  // Cambiar por constant
        Mat finalImage, finalImage2;
        Data.UpdateDataReader(j, j+1);
        j = j+1;
        visystem.AddFrame(Data.image2, Data.imuAngularVelocity, Data.imuAcceleration);
        Mat31f t = visystem.final_pose.translation();
        Quaternion quaternion = visystem.final_pose.unit_quaternion();

        
       /*
       
        //drawKeypoints( frame1, aux, frame1, Scalar(0, 0, 255), DrawMatchesFlags::DEFAULT);
        

       visualizerFrame.UpdateMessages(finalImage);
        }
        //visualizerFrame2.UpdateMessages(finalImage2);
     
       
        */
      
       

        Point3d angle_diff;
        Point3d angle_gt, angle_ft, gravity_imu;
        double elapsed_f;


        orientation[0] = Data.gtQuaternion[Data.gtQuaternion.size()-1].x;   //x
        orientation[1] = Data.gtQuaternion[Data.gtQuaternion.size()-1].y; // y
        orientation[2] = Data.gtQuaternion[Data.gtQuaternion.size()-1].z; // z
        orientation[3] = Data.gtQuaternion[Data.gtQuaternion.size()-1].w; // w

        position[0] = Data.gtPosition[Data.gtPosition.size()-1].x;   //x
        position[1] = Data.gtPosition[Data.gtPosition.size()-1].y;   //x
        position[2] = Data.gtPosition[Data.gtPosition.size()-1].z;   //x

        //imuCore.printStatistics(); // imprime el tiempo de computo del filtro
        
        
       
        visualizer_gt.UpdateMessages(Data.gtPosition.back(), Data.gtQuaternion.back());
        cout<< " Current time = "<< Data.currentTimeMs <<" ms " <<endl;


        outputFilecsv <<  -t(0)<<","
        <<-t(2)<<","
        <<-t(1)<<","
        <<quaternion.x() <<","
        <<quaternion.y()<<","
        <<quaternion.z()<<","
        <<quaternion.w()<<","
        <<Data.gtPosition[Data.gtPosition.size()-1].x <<","
        << Data.gtPosition[Data.gtPosition.size()-1].y<<","
        << Data.gtPosition[Data.gtPosition.size()-1].z<<","
        <<Data.gtQuaternion[Data.gtQuaternion.size()-1].x<<","
        <<Data.gtQuaternion[Data.gtQuaternion.size()-1].y<<","
        <<Data.gtQuaternion[Data.gtQuaternion.size()-1].z<<","
        <<Data.gtQuaternion[Data.gtQuaternion.size()-1].w
        <<endl;

        //visualizer_est.UpdateMessages(position2, orientation2);
            
            

                
            //vector3d.UpdateMessages(acc_diff);
                
        }
        /*
        //cout << "tamData"<< Data.gtRPY.size()<<endl;
        Point3d error_ang_est, error_ang_gt, error_ang;
        Point3d error_vel_est, error_vel_gt, error_vel;
        Point3d error_pos_est, error_pos_gt, error_pos;
        error_ang_est.x = imuCore.rpyAnglesWorld[imuCore.quaternionWorld.size()-1].x-imuCore.rpyAnglesWorld[0].x;
        error_ang_est.y = imuCore.rpyAnglesWorld[imuCore.quaternionWorld.size()-1].y-imuCore.rpyAnglesWorld[0].y;
        error_ang_est.z = imuCore.rpyAnglesWorld[imuCore.quaternionWorld.size()-1].z-imuCore.rpyAnglesWorld[0].z;
        
        error_ang_gt.x = Data.gtRPY[Data.gtRPY.size()-1].x-Data.gtRPY[0].x;
        error_ang_gt.y = Data.gtRPY[Data.gtRPY.size()-1].y-Data.gtRPY[0].y;
        error_ang_gt.z = Data.gtRPY[Data.gtRPY.size()-1].z-Data.gtRPY[0].z;

        error_ang = error_ang_est-error_ang_gt;

        //Velocidad
        //velocity = velocity+imuCore.velocity;
       // velocidad_groundtruth.UpdateMessages(Data.gtLinearVelocity[0]);
       // velocidad_estimado.UpdateMessages(velocity);
        error_vel_gt = Data.gtLinearVelocity[Data.gtLinearVelocity.size() -1]-Data.gtLinearVelocity[0];
        error_vel_est = imuCore.velocity;
        error_vel =  error_vel_gt - error_vel_est;
        int n = Data.gtPosition.size()-1;
        error_pos_est = imuCore.position;
        error_pos_gt =  Data.gtPosition[Data.gtPosition.size()-1] - Data.gtPosition[0] ; 
        //cout << "Size" <<Data.gtPosition.size() <<endl;
        error_pos = error_pos_est-error_pos_gt;
        int size = Data.gtPosition.size();
        if (size > 10) size = 10;
        for (int jj = 0; jj < size ; jj++)
        {
           accx << imuCore.accelerationWorld[jj].x<<endl;
           accy << imuCore.accelerationWorld[jj].y<<endl;
           accz << imuCore.accelerationWorld[jj].z<<endl;
           angxGt << Data.gtRPY[jj].x<<endl;
           angyGt << Data.gtRPY[jj].y<<endl;
           angzGt << Data.gtRPY[jj].z<<endl;
           angxEst << imuCore.rpyAnglesWorld[jj].x<<endl;
           angyEst << imuCore.rpyAnglesWorld[jj].y<<endl;
           angzEst << imuCore.rpyAnglesWorld[jj].z<<endl;
           biasx << Data.accBias[jj].x<<endl;
           biasy << Data.accBias[jj].y<<endl;
           biasz << Data.accBias[jj].z<<endl;
           velx << Data.gtLinearVelocity[jj].x << endl;
           vely << Data.gtLinearVelocity[jj].y << endl;
           velz << Data.gtLinearVelocity[jj].z << endl;
           posx << Data.gtPosition[jj].x <<endl;
           posy << Data.gtPosition[jj].y <<endl;
           posz << Data.gtPosition[jj].z <<endl;

        }

        
       */


	/*
	if(error_vel_gt.x>0.01)
		error_vel.x = error_vel.x/error_vel_gt.x*100;
	else
		error_vel.x = 0.0;
	if(error_vel_gt.x>0.01)
		error_vel.y = error_vel.y/error_vel_gt.y*100;
	else
		error_vel.y = 0.0;
	if(error_vel_gt.x>0.01)
		error_vel.z = error_vel.z/error_vel_gt.z*100;
	else
		error_vel.z = 0.0;
*/
        //error_angular.UpdateMessages(error_ang*180/M_PI);
        //error_velocidad.UpdateMessages(error_vel);
        //error_posicion.UpdateMessages(error_pos);
       
        //posicion_estimada.UpdateMessages(error_pos_est);
        //posicion_groundtruth.UpdateMessages(error_pos_gt);



        
        //visualizer_gt.UpdateMessages(position, orientation);
        //visualizer_est.UpdateMessages(position, orientation2);

       
        /*
        cout << " diffx = " << imuCore.accelerationWorld[9].x
        <<" diffy = "<<imuCore.accelerationWorld[9].y
        <<" diffz = "<<imuCore.accelerationWorld[9].z
        << " Current time = "<< Data.currentTimeMs <<" ms "
        << "accx = "  << Data.accBias.x
        << "timestep = "<< imuCore.timeStep

        
        
	<<endl;*/
      


        

        
        
      
       

    
    




    return 0;

}

// COMPILE COMMAND
//g++ -g main_vi_slam.cpp GroundTruth.cpp -o main_vi_slam.out `pkg-config opencv --cflags --libs`

// RUN COMMAND
//./main_vi_slam.out -filePath=../../../../Documents/EuroDataset/state_groundtruth_estimate0/data.csv
// rosrun vi_slam vi_slam -filePath=../../../../Documents/EuroDataset/state_groundtruth_estimate0/data.csv 
