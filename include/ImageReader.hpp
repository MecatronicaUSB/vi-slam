///Basic C and C++ libraries
#include <iostream>
#include <dirent.h>
#include <vector>
#include <algorithm> // sort function


#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"

using namespace std;
using namespace cv;

 class ImageReader{
     public:
        ImageReader();
        ImageReader(string _directory);
        void setPath(string _directory);
        string getImageName(int index);
        long int getImageTime(int index);
        void searchImages();
        Mat getImage(int index);
        size_t splitStrings(const std::string &txt, std::vector<std::string> &strs, char separator);
        size_t getSize();
        void computeTimeStep();
        double TimeStep;
    private:
        string path;
        vector<string> file_names;  
        size_t size; // Numero de imagenes
        DIR *dir;
        struct dirent *ent;

 };

