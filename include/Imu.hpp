#include <vector>
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
        Imu(double timestep);//revisar el paso por size
        void setImuData(vector <Point3d> &w_measure,vector <Point3d>  &a_measure);
        void setImuBias(Point3d acc_Bias, Point3d ang_Bias);
        void setImuInitialVelocity();
        void setImuInitialPosition();
        void initializate(double gt_yaw);
        void estimate();
        void computeGravity();
        void computePosition();
        void computeVelocity();
        void computeAcceleration();
        void computeAngularVelocity();
        void computeAngularPosition();
        void printStatistics();
        Point3d transform2World(Point3d acc, Point3d localAngles);
        Point3d angularPosition;
        Point3d angularVelocity;
        Point3d initialVelocity;
        double initialYawGt;
        double initialYawFilter;
        Point3d position; // posicion en x, y, z
        Point3d velocity; // velocidad en x, y, z
        Point3d accBias;
        Point3d angBias;
        vector <Point3d> angularVelocityIMUFilter; // velocidad angular
        vector <Quaterniond> quaternionWorld;
        vector <Point3d> rpyAnglesWorld; // orientacion del robot en rpy respecto al mundo 
        vector <Point3d> accelerationWorld; // celeracion del robot respecto al mundo

    private:
        double elapsed_filter;
        vector <Point3d> angularVelocityMeasure; // Imu measurements x, y, z
        vector <Point3d> accelerationMeasure;    // Imu measurements x, y, z
        int n; // numero de mediciones
       
        double timeStep;



 
 };