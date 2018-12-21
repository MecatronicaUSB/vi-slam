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
#include "Options.hpp"
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


/// Clase Frame: Contiene toda la informacion de la imagen actual
class Frame
{
    public:
        /**
         * @brief Constructor of Frame.
         */
        Frame();

        /**
         * @brief Destructor of Frame
         */
        ~Frame();

        // ---------- Attributes
        vector<Mat> grayImage =  vector<Mat>(5);  //!< imagen en escala de grises
        vector<Mat> gradientX= vector<Mat>(5) ;  //!< gradiente en direccion x de la imagen
        vector<Mat> gradientY= vector<Mat>(5);  //!< gradiente en direccion y de la imagen
        vector<Mat> gradient= vector<Mat>(5);   //!< gradiente en direccion x y y de la imagen
        
        vector<KeyPoint> keypoints;        //!< keypoints de la imagen
        vector<KeyPoint>  prevGoodMatches; //!< matches con la imagen anterior
        vector<KeyPoint>  nextGoodMatches; //!< matches con la imagen siguiente
        Mat descriptors;                   //!< keypoints de la imagen
        vector<Mat> prevPatches; //!< Parches de intensidad alrededor de los features
        vector<Mat> nextPatches; //!< Parches de intensidad alrededor de los features
        vector<Mat> candidatePoints  = vector<Mat>(5);
        vector<KeyPoint> debugKeypoints ;// debug Points
        vector<Mat> candidateDebugPoints  = vector<Mat>(5);

        int idFrame;      //!< identificador del frame
        double imageTime; //!< tiempo en el que fue tomado la imagen
        Sophus::SE3f rigid_transformation_;

        bool obtainedGradients;   //!< flag de obtencion del gradiente
        bool obtainedGoodMatches; //!< flag de obtencion de matches    
        bool isKeyFrame;          //!< flag de keyframe
};

class Camera
{
    public:
        Camera();
        Camera(int _detector, int _matcher, int _w_size, int _h_size, int _num_cells, int _length_path);
        void initializate(int _detector, int _matcher, int _w_size, int _h_size, int _num_cells, int _length_path);
        void Update (Mat _grayImage); // Ingresa una nueva imagen al sistema

        void setDetector(int _detector); //
        int detectFeatures(); // Retorna el numero de features detectados
        int detectAndComputeFeatures(); // Retorna el numero de features detectados

        void setMatcher (int _matcher);
        void computeDescriptors();
        void computeGoodMatches(); // Retorna las correspondencias filtradas entre dos imagenes

        void computeGradient(); // Calcula el gradiente de la imagen

        bool addKeyframe(); // Añade o no un keyframe 
        
        void saveFrame(); //(frame->framelist)
        void ObtainDebugPointsPreviousFrame();
        void ObtainPatchesPointsPreviousFrame();
        void computePatches(); // calcula los parches del frame actual;
        void computeResiduals(); // calculas los residuales fotometricos entre el frame actual y el anterior
        
        void printStatistics();
        
        vector <Frame *> frameList;  // lista de todos los frames
        vector <Mat>   Residuals;  // residuales de intensidad entre dos imagenes
        vector<DMatch> goodMatches; // correspondencias finales entre dos frames
        Frame * currentFrame;
       
        int w_residual;  // ancho del parche de intensidad y del residual
        int h_residual;  //  del parche de intensidad y del residual
        int detectorType; // tipo de detector a utilizar
        int matcherType;  // tipo de matcher a utilizar

        Matcher matcher; // Matcher del sistema
        int nPointsDetect;  // numero de puntos detectados en la imagen actual
        int nBestMatches; // numero de matches finales
        int n_cells;  // numero de celdas del grid
        Ptr<Feature2D> detector;   //!< Pointer to OpenCV feature extractor

        //Informacion de la imagen
        vector<int> w_size = vector<int>(5);
        vector<int> h_size = vector<int>(5);
        // Tamaño de parches (impar)
        int w_patch, h_patch;

        // Informacion de estadisticas
        double elapsed_detect;
        double elapsed_descriptors;
        double elapsed_computeGoodMatches;
        double elapsed_computeGradient;
        double elapsed_computePatches;

        // Promedios
        double elapsed_detect_mean;
        double elapsed_descriptors_mean;
        double elapsed_computeGoodMatches_mean;
        double elapsed_computeGradient_mean;
        double elapsed_computePatches_mean;
        double nPointsDetect_mean;
        double nBestMatches_mean;
        int num_images;

          // Suma total de tiempos
        double elapsed_detect_sum;
        double elapsed_descriptors_sum;
        double elapsed_computeGoodMatches_sum;
        double elapsed_computeGradient_sum;
        double elapsed_computePatches_sum;
        double nPointsDetect_sum;
        double nBestMatches_sum;
    
};

#endif