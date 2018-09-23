#include "opencv2/core.hpp"
#include <iostream>
using namespace std;
using namespace cv;

class GroundTruth{
public:
    GroundTruth();
    GroundTruth(string file, char separator);
    void setFileProperties(string file, char separator);
    void getDataFromFile(); //Lee el archivo y lo pasa a Mat data
    int getLines(); // retorna el numero de lineas del archivo
    int getRows(); // retorna el numero de filas de la data
    int getCols(); // retorna el numero de columnas de la data
    size_t splitStrings(const std::string &txt, std::vector<std::string> &strs, char separator);
    string getFileName();
    char getCharSeparator();
    double getGroundTruthData(int line, int colData); // linea del archivo y columna de datos
    void computeTimeStep(); 
    double TimeStep; // tiempo entre dos medidas consecutivas
    
private:
    char charSeparator; // caracter que separa los datos en una linea del archivo
    string fileName; // Nombre del archivo de groundtruth
    Mat data ;       // Datos del archivo referentes a posicion,
    int cols;
    int rows;
};