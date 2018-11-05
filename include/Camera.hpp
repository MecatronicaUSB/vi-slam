#include "Matcher.hpp"
#include <opencv2/imgproc.hpp>
using namespace cv;
using namespace cv::xfeatures2d;

// Tipos de detectores soportados
enum detectorType
{
    USE_KAZE,
    USE_AKAZE,
    USE_ORB,
    USE_SIFT,
    USE_SURF
};



class Frame
{
    public:
        /**
         * @brief Constructor of Frame.
         * 
         */
        Frame();

        /**
         * @brief Destructor of Frame
         * 
         */
        ~Frame();
        int idFrame;
        Mat grayImage;
        Mat gradientX;
        Mat gradientY;
        Mat gradient;
        
        vector<KeyPoint> keypoints;
        vector<KeyPoint>  prevGoodMatches; // matches con la imagen anterior
        vector<KeyPoint>  nextGoodMatches; // matches con la imagen siguiente
        Mat descriptors;
        vector<Mat> prevPatches; // Parches de intensidad alrededor de los features
        vector<Mat> nextPatches; // Parches de intensidad alrededor de los features
        
        int idFrame_;
        int nMatches;  
        double imageTime; // tiempo en el que fue tomado la imagen

        bool obtainedGradients;
        bool obtainedGoodMatches;     
        bool isKeyFrame;
};

class Camera
{
    public:
        Camera(int _detector, int _matcher, int _w_size, int _h_size);
        void initializate(int _detector, int _matcher, int _w_size, int _h_size);
        void Update (Mat _grayImage); // Ingresa una nueva imagen al sistema

        void setDetector(int _detector); //
        int detectFeatures(); // Retorna el numero de features detectados

        void setMatcher (int _matcher);
        void computeDescriptors();
        void computeGoodMatches(); // Retorna las correspondencias filtradas entre dos imagenes

        void computeGradient(); // Calcula el gradiente de la imagen

        bool addKeyframe(); // Añade o no un keyframe 
        
        void saveFrame(); //(frame->framelist)

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
        int n_cells;  // numero de celdas del grid
        Ptr<Feature2D> detector;   //!< Pointer to OpenCV feature extractor

        //Informacion de la imagen
        int w_size, h_size;
        // Tamaño de parches (impar)
        int w_patch, h_patch;

        // Informacion de estadisticas
        double elapsed_detect;
        double elapsed_descriptors;
        double elapsed_computeGoodMatches;
        double elapsed_computeGradient;
        double elapsed_computePatches;
    
};