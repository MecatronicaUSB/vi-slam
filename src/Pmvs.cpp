#include "../include/Pmvs.hpp" 
#include <iostream>
#include <fstream>


template < typename T > std::string int2string( const T& n ) // funcion para transformar entero en string
{
    std::ostringstream stm ;
    stm << n ;
    if (n>=10 & n < 100){
        return "00"+stm.str();
    }
    if (n >= 100 & n<1000){
        return "0"+stm.str();
    }
        
    return "000"+stm.str() ;
}




Pmvs::Pmvs()
{
    numImages = 0;
    binary = "";
    outPath = "";
}

void Pmvs::setBinary(string _binary)
{
    binary = _binary;
}

void Pmvs::setOutPath(string _outPath)
{
    outPath = _outPath;
}

void Pmvs::AddFrame(Mat image, Matx34d ProjectionMatrix)
{
   
    imwrite(outPath+"/root/visualize/"+int2string(numImages)+".jpg", image);
    ofstream outTxt(outPath+"/root/txt/"+int2string(numImages)+".txt");

    outTxt << "CONTOUR" << endl;

    for (int j=0; j < 3; j++) {
        for (int k=0; k < 4; k++) {
            outTxt << ProjectionMatrix(j, k) << " ";
        }
        outTxt << endl;
    }
    outTxt.close();
    numImages++;
}
                
              
    
void Pmvs::createOptions()
{
    ofstream option(outPath+"/root/options.txt",std::ofstream::out | std::ofstream::trunc);

    option << "timages  -1 " << 0 << " " << numImages-1<< endl;;
    option << "oimages 0" << endl;
    option << "level 1" << endl;

    option.close();
}

void Pmvs::createOutputDirectories()
{
    string deleteOutputDirectory = "rm -r "+outPath+ "/root";


    string createVisualize = "mkdir -p "+outPath+ "/root/visualize";
    string createTxt = "mkdir -p "+outPath+ "/root/txt";
    string createModels = "mkdir -p "+outPath+ "/root/models";



    // Delete old directories
    system(deleteOutputDirectory.c_str());


    // Create new directories
    system(createVisualize.c_str());
    system(createTxt.c_str());
    system(createModels.c_str());
}


void Pmvs::RunPMVS()
{
    string runCommand = binary+ " " +outPath +"/root/" +" options.txt";
    cout << "runCommand " << runCommand <<endl;
    system(runCommand.c_str());

    
    
}