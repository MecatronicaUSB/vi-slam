#include <iostream>
#include <fstream>
#include "VISystem.hpp"
#include "CameraGPU.hpp"




using namespace std;
using namespace cv;

namespace vi
{
class VISystemGPU : public VISystem
{
    public:
        VISystemGPU();
        VISystemGPU(int argc, char *argv[]);
        ~VISystemGPU();
        void InitializeSystemGPU(string _calPath, Point3d _iniPosition, Point3d _iniVelocity, Point3d _iniRPY, Mat image);
        void InitializeCameraGPU(int _detector, int _matcher, int _w_size, int _h_size, int _num_cells, int _length_path);
        void AddFrameGPU(Mat _currentImage, vector <Point3d> _imuAngularVelocity, vector <Point3d> _imuAcceleration);
        void FreeLastFrameGPU();

        CameraGPU cameraGPU;
        
       


   

        
    

};

}

