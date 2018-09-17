#include <vector>
#include "opencv2/core.hpp"
using namespace std;
using namespace cv;
 class Imu
 {
    public: 
        Imu(double timestep);//revisar el paso por size
        void setImuData(vector <Point3d> &w_measure,vector <Point3d>  &a_measure);
        void estimate();
        void computePosition();
        void computeVelocity();
        void computeAcceleration();
        void computeAngularVelocity();
        void computeAngularPosition();
        Point3d angularPosition;
        Point3d angularVelocity;
        Point3d position; // posicion en x, y, z
        Point3d velocity; // velocidad en x, y, z
        Point3d acceleration; // promedio de acceleracion en x, y, z

    private:
        vector <Point3d> angularVelocityMeasure; // Imu measurements x, y, z
        vector <Point3d> accelerationMeasure;    // Imu measurements x, y, z
        int n; // numero de mediciones
       
        double timeStep;



 
 };