#include "ImageReader.hpp"

ImageReader::ImageReader()
{
    setPath("");
}

ImageReader::ImageReader(string _directory)
{
    setPath(_directory);
}


void ImageReader::setPath(string _directory)
{
   path = _directory; 
}

string ImageReader::getImageName(int index)
{
    string long_name; // incluye el path
    vector <string> short_name; // Solo nombre de la imagen
    string imageName;

    long_name = file_names[index].c_str();
    splitStrings(long_name, short_name,'/');
    imageName = short_name[short_name.size()-1];
    
    vector <string> name; // Solo nombre de la imagen
    if (imageName.find(".") )
    {
        
        splitStrings(imageName, name,'.');
        imageName = name[0];
    }
    return imageName;
}

long int ImageReader::getImageTime(int index)
{
    string name = (getImageName(index)).c_str();
    char imageName[name.size()];
    strcpy(imageName, name.c_str());
    return atol(imageName);
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

size_t ImageReader::splitStrings(const std::string &txt, std::vector<std::string> &strs, char separator)
{
    size_t pos = txt.find( separator );
    size_t initialPos = 0;
    strs.clear();

    // Decompose statement
    while( pos != std::string::npos ) {
        strs.push_back( txt.substr( initialPos, pos - initialPos ) );
        initialPos = pos + 1;

        pos = txt.find( separator, initialPos );
    }

    // Add the last one
    strs.push_back( txt.substr( initialPos, std::min( pos, txt.size() ) - initialPos + 1 ) );

    return strs.size();

}