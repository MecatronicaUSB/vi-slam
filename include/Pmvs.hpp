
#include "opencv2/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include <iostream>

using namespace cv;
using namespace std;

class Pmvs
{
    public:
        Pmvs();
        void setBinary(string _binary);
        void setOutPath(string _outPath);
        void createPmvs();
        void AddFrame(Mat image, Matx34d ProjectionMatrix);
        void createOptions();
        void createOutputDirectories();
        void RunPMVS();
    private:
        string binary;
        string outPath;
        int numImages;
        



};