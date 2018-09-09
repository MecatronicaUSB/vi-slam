#include "ImageReader.hpp"

ImageReader::ImageReader(string _directory)
{
    path = _directory;
    searchImages();
}

void ImageReader::searchImages(){
    cout << "Searching images files in directory ... ";
    if ((dir = opendir(path.c_str())) != NULL) {
        while ((ent = readdir (dir)) != NULL) {
            file_names.push_back(path + string(ent->d_name));
        }
        closedir (dir);
    } else {
        // If the directory could not be opened
        cout << "Could not open directory of images: " << path << endl;
        cout << "Exiting..." << endl;
        exit(0);
    }
    // Sorting the vector of strings so it is alphabetically ordered
    sort(file_names.begin(), file_names.end());
    file_names.erase(file_names.begin(), file_names.begin()+2);

    if (file_names.size() < 15) {
        cout << "\nInsufficient number of images found. Please use a larger dataset" << endl;
        cout << "Exiting..." << endl;
        exit(0);
    }
    cout << file_names.size() << " found"  << endl;

}

Mat ImageReader::getImage(int index){
    return imread(file_names[index], CV_LOAD_IMAGE_GRAYSCALE);
}

size_t ImageReader::getSize(){
   return file_names.size();
}