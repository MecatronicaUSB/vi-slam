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
        ImuFilterNode(double rate);
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
        void estimate();
        void computeGravity();
        void computePosition();
        void computeVelocity();
        void computeAcceleration();
        void computeAngularVelocity();
        void computeAngularPosition();
        Quaterniond quaternion;
        Point3d angularPosition;
        Point3d angularVelocity;
        Point3d position; // posicion en x, y, z
        Point3d velocity; // velocidad en x, y, z
        Point3d acceleration; // promedio de acceleracion en x, y, z
        Point3d accBias;
        Point3d angBias;
        vector <Point3d> imuFilterGravity;

    private:
        vector <Point3d> angularVelocityMeasure; // Imu measurements x, y, z
        vector <Point3d> accelerationMeasure;    // Imu measurements x, y, z
        int n; // numero de mediciones
       
        double timeStep;



 
 };