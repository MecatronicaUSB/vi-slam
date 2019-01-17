#include "Plus.hpp"
#include "opencv2/core.hpp"
#include <sensor_msgs/Imu.h>
#include <message_filters/subscriber.h>
#include <ros/callback_queue.h>

using namespace std;
using namespace cv;

class ImuFilterNode{
    typedef sensor_msgs::Imu              ImuMsg;
    typedef message_filters::Subscriber<ImuMsg> ImuSubscriber;
    public:
        ImuFilterNode();
        ImuFilterNode(int rate);
        void createROSPublisher(int rate);
        void createROSSubscriber();
        void UpdatePublisher(Point3d w_measure, Point3d a_measure);
        void UpdateSubscriber();
        void imuCallback(const ImuMsg::ConstPtr& imu_msg_raw);
        
        string getNodeName();
        double getRateHZ();

        ImuMsg imuFusedData;
        uint timeNs;
        uint timeS;
        

    private:
        int rateHZ;
        ros::Publisher publisher; 
        boost::shared_ptr<ImuSubscriber> imu_subscriber_;
        sensor_msgs::Imu message;
        

};

 class Imu: public ImuFilterNode
 {
    public: 
        Imu();
        Imu(double timestep);//revisar el paso por size
        void createPublisher(double _timeStep);
        void setImuData(vector <Point3d> &w_measure,vector <Point3d>  &a_measure);
        void setImuBias(Point3d acc_Bias, Point3d ang_Bias);
        void setImuInitialVelocity(Point3d initial_velocity);
        void setImuInitialPosition();
        void initializate(double gt_yaw, Point3d gt_velocity, vector <Point3d> &w_measure,vector <Point3d>  &a_measure );
        void estimate();
        void estimateOrientation();
        void computeGravity();
        void computePosition();
        void computeVelocity();
        void computeAcceleration();
        //void computeAcceleration(vector <Point3d> gtRPY);
        void computeAngularVelocity();
        void computeAngularPosition();

        void calibrateAng(int axis); // calibración angular considerando velocidad 0
        void calibrateAcc(int axis); // calibración acelaracion considerando velocidad 0
        void detectAngBias(); // Detectar medidas para la calibración ang
        void detectAccBias(); // Detectar medidas para la calibración acc
        void printStatistics();
        Point3d transform2World(Point3d acc, Point3d localAngles);
        Point3d angularPosition;
        Point3d angularVelocity;
        Point3d initialVelocity;
        double initialYawGt;
        double initialYawFilter;
        double YawGt;
        Point3d position; // posicion en x, y, z
        Point3d velocity; // velocidad en x, y, z
        Point3d accBias; // Bias aceleracion
        Point3d angBias; // Bias angular
        vector <Point3d> angularVelocityIMUFilter; // velocidad angular
        vector <Quaterniond> quaternionWorld;
        vector <Point3d> rpyAnglesWorld; // orientacion del robot en rpy respecto al mundo 
        vector <Point3d> accelerationWorld; // aceleracion del robot respecto al mundo

        Mat init_rotationMatrix;
        Mat final_rotationMatrix;
        Mat residual_rotationMatrix;

        vector <Mat> world2imuRotation;

        // Residuales
        Point3d residualRPY;
        Point3d residualPosition;
        Point3d residualVelocity;
        double timeStep;
    private:
        double elapsed_filter;
        vector <Point3d> angularVelocityMeasure; // Imu measurements x, y, z
        vector <Point3d> accelerationMeasure;    // Imu measurements x, y, z
        int n; // numero de mediciones
        int n_total; // numero total de mediciones
        double currentTimeMs; // tiempo actual en ms
       
        



 
 };