/**
 * @file Camera.hpp
 * @brief Description of Camera(CPU) Class
 * @version 0.2
 * @date 10/11/2018
 * @author Luis Lujano
 */
#ifndef CAMERA_H_
#define CAMERA_H_

#include "Matcher.hpp"


#include <iostream>
#include <iomanip>
#include <opencv2/imgproc.hpp>

using namespace cv;
using namespace cv::xfeatures2d;

/// Tipos de detectores soportados
enum detectorType
{
    USE_KAZE,
    USE_AKAZE,
    USE_ORB,
    USE_SIFT,
    USE_SURF
};




class Camera
{
    public:
        Camera();
        Camera(int _detector, int _matcher,int _w_size, int _h_size, int _num_wcells, int _num_hcells);
        void initializate(int _detector, int _matcher, int _w_size, int _h_size, int _num_wcells, int _num_hcells);
        void setCurrentFrame (Frame *_currentFrame); // Ingresa una nueva imagen al sistema
        void setPreviousFrame (Frame *_previousFrame); // Ingresa una nueva imagen al sistema
        void setCameraMatrix(Mat _K);

        void setDetector(int _detector); //
        int detectFeatures(); // Retorna el numero de features detectados
        int detectAndComputeFeatures(); // Retorna el numero de features detectados

        void setMatcher (int _matcher);
        void computeDescriptors();
        void computeGoodMatches(); // Retorna las correspondencias filtradas entre dos imagenes
        void extractMatches();

        void computeFastMatches(); // Calcular matches sin filtrado
        void computeEssentialMatches(); // Calcular matches con filtrado RANSAC de matriz essential
       
        
        vector<DMatch> goodMatches; // correspondencias finales entre dos frames
        Frame *currentFrame;
        Frame *previousFrame;
       

        int detectorType; // tipo de detector a utilizar
        int matcherType;  // tipo de matcher a utilizar

        Matcher matcher; // Matcher del sistema
        int nPointsDetect;  // numero de puntos detectados en la imagen actual
        int nBestMatches; // numero de matches finales
        int n_wcells;  // numero de celdas horizontales del grid
        int n_hcells;  // numero de celdas verticales del grid
         // el numero de celdas totales es n_wcells*n_hcells
        Ptr<Feature2D> detector;   //!< Pointer to OpenCV feature extractor

        // tamaño de la imagen 
        int w_size ;
        int h_size ;
        Mat K; // camera matrix


    
};

#endif