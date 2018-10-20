#include <vector>
#include "Plus.hpp"
#include "opencv2/core.hpp"

using namespace std;
using namespace cv;
 class Imu
 {
    public: 
        Imu(double timestep);//revisar el paso por size
        void setImuData(vector <Point3d> &w_measure,vector <Point3d>  &a_measure);
        void setImuBias(Point3d acc_Bias, Point3d ang_Bias);
        void estimate();
        void computePosition();
        void computeVelocity();
        void computeAcceleration();
        void computeAngularVelocity();
        void computeAngularPosition();
        void setImuGravity(Point3d gravity);
        Quaterniond quaternion;
        Point3d angularPosition;
        Point3d angularVelocity;
        Point3d position; // posicion en x, y, z
        Point3d velocity; // velocidad en x, y, z
        Point3d acceleration; // promedio de acceleracion en x, y, z
        Point3d accBias;
        Point3d angBias;
        Point3d imuGravity;

    private:
        vector <Point3d> angularVelocityMeasure; // Imu measurements x, y, z
        vector <Point3d> accelerationMeasure;    // Imu measurements x, y, z
        int n; // numero de mediciones
       
        double timeStep;



 
 };