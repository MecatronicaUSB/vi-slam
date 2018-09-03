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
        ImageReader(string _directory);
        void searchImages();
        Mat getImage(int index);
        size_t getSize();
    private:
        string path;
        vector<string> file_names;  
        size_t size; // Numero de imagenes
        DIR *dir;
        struct dirent *ent;

 };

