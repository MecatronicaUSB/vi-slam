#ifndef CAMERAGPU_H_
#define CAMERAGPU_H_
#include "Camera.hpp"
#include "MatcherGPU.hpp"
#include "opencv2/cudafeatures2d.hpp"
#include "opencv2/xfeatures2d/cuda.hpp"

using namespace cv; 
using namespace cv::xfeatures2d;
using namespace std;

// Optimizar destructores y creadores
//Utiliza la clase Frame

class CameraGPU : public Camera
{
    public:
        CameraGPU(int _detector, int _matcher, int _w_size, int _h_size);
        void initializateCameraGPU(int _detector, int _matcher, int _w_size, int _h_size);
        void setGPUDetector(int _detector);
        void detectGPUFeatures();
        void setGPUMatcher (int _matcher);
        int detectAndComputeGPUFeatures(); // Retorna el numero de features detectados
        void computeGPUGoodMatches();
        bool addGPUKeyframe();

        MatcherGPU matcherGPU;
        cuda::GpuMat frameGPU;
        cuda::GpuMat keypointsGPU;
        cuda::GpuMat descriptorsGPU ;
        bool useGPU;
   
};
#endif