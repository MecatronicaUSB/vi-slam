#include <fstream>
#include "../include/GroundTruth.hpp"

using namespace std;


GroundTruth::GroundTruth()
{
    data = Mat::zeros(1, 1, CV_64F); // inicializar matriz de datos
}

GroundTruth::GroundTruth(string file, char separator)
{   
    data = Mat::zeros(1, 1, CV_64F); // inicializar matriz de datos
    setFileProperties(file, separator);
}

void GroundTruth::setFileProperties(string file, char separator)
{
    fileName = file;
    charSeparator = separator;
    data = Mat::zeros(1, 1, CV_64F); // Inicializar data como cero
}


int GroundTruth::getLines(){
    ifstream inFile;
    string line;
    int numberLines = 0;
    inFile.open(fileName.c_str());
    if (!inFile) {
        cerr << "Unable to open file datafile.txt";
        exit(1);   // call system to stop
    }

    while(getline(inFile, line)){
        numberLines++;
    }

    return numberLines;

}
int GroundTruth::getRows() // Numero de filas de datos
{
    return rows;
}

int GroundTruth::getCols() // Numero de columnas de datos
{ 
    return cols;

}

void GroundTruth::getDataFromFile()
{
    ifstream inFile;
    string line;
    int counterLine;
    std::vector<std::string> vectorString;
    bool commentFound;

    //Abrir archivo
    inFile.open(fileName.c_str());
    if (!inFile) {
        cerr << "Unable to open file datafile.txt";
        exit(1);   // call system to stop
    }
    
    commentFound = 1;  // Evitar comentarios al comienzo del archivo
    while(commentFound){
        getline(inFile, line);
        if (line.find("#")!= -1) commentFound = 1;
        else commentFound = 0;
        //counterLine++;
    }
    counterLine = 0;  // Numero de linea leida
    splitStrings(line, vectorString, ',' ); // size 12
    rows = getLines();
    cols = vectorString.size();
    if (cols!= 0){ // archivo no vacio
        // Inicializacion
        data = Mat::zeros(rows, cols, CV_64F);
        for (int i = 0 ;i < cols; i++){
             data.at<double>(counterLine, i) =  atof(vectorString[i].data());  // convertir string en double(DATA)
            }
    }
    else{
        cout<<"Error! Empty GroundTruth file!"<<endl;
        exit(1);
    }

    while(!inFile.eof()) { // leer hasta el final del archivo
            getline(inFile, line);
            counterLine++;
            splitStrings(line, vectorString, ',' ); // size 12
            for (int i = 0 ;i < cols; i++){
                data.at<double>(counterLine, i) =  atof(vectorString[i].data());  // convertir string en double(DATA)
            }

        }

      inFile.close();
}

size_t GroundTruth::splitStrings(const std::string &txt, std::vector<std::string> &strs, char separator)
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

string GroundTruth::getFileName()
{
    return fileName;
}

char GroundTruth::getCharSeparator()
{
    return charSeparator;
}

double GroundTruth::getGroundTruthData(int line, int colData)
{
    return data.at<double>(line, colData);
}
