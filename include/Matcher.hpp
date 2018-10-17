#ifndef MATCHER_H_
#define MATCHER_H_

// Opencv libraries
#include "opencv2/core.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/features2d.hpp"

using namespace cv;
using namespace cv::xfeatures2d;
using namespace std;

// Tipos de detectores soportados
enum detectorType
{
    USE_KAZE,
    USE_AKAZE,
    USE_ORB,
    USE_SIFT,
    USE_SURF
};


// Tipos de emparejadores soportados
enum matcherType
{
    USE_BRUTE_FORCE,
    USE_FLANN
};



class Matcher
{
    public:
        Matcher();
        Matcher(int _detector, int _matcher); // Anexar ROI pendiente
        void setFrames(Mat _frame1, Mat _frame2);
        void setDetector(int _detector);
        void setMatcher(int _matcher);
        void computeMatches();
        void computeSymMatches();
        int bestMatchesFilter ( int n_features);// Dsitrubuye los features uniformemente y selecciona  las mejores parejas
        int nnFilter(vector<vector<DMatch> > &matches, double nn_ratio); // filtro de vecinos mas cercanos
        void detectFeatures();
        void resetVectorMatches(vector<DMatch> &matches);
        void pushBackVectorMatches(vector<DMatch> &matches);
        void getMatches(vector<KeyPoint> &_matched1, vector<KeyPoint> &_matched2);
        void getGoodMatches(vector<KeyPoint> &_matched1, vector<KeyPoint> &_matched2);
        void sortMatches();
        double getMatchPercentage();
        void computeBestMatches(int n_features);
        void getGrid(int n_features, vector<KeyPoint> &grid_point);
        
        void 
        void printStatistics();


        vector< vector<DMatch> > aux_matches1; // Vector auxiliar
        vector< vector<DMatch> > aux_matches2; // Vector auxiliar
        vector<DMatch> matches; // correspondencias filtradas
        vector<DMatch> sortedMatches; // correspondencias ordenadas en funcion de la coordenada y del pixel
        vector<DMatch> goodMatches; // correspondencias finales
        Ptr<Feature2D> detector;                    //!< Pointer to OpenCV feature extractor
        Ptr<DescriptorMatcher> matcher;             //!< Pointer to OpenCV feature Matcher
        Mat frame1, frame2;
        vector<KeyPoint> matched1, matched2;
        vector<KeyPoint> keypoints_1, keypoints_2; // Vector para almacenar los puntos detectados con FAST
        Mat descriptors_1, descriptors_2;

        // ---------- Attributes
        int h_size, w_size;
        int nPointsDetect1, nPointsDetect2; // numero de puntos detectados en la imagen 1 y 2
        int nSymMatches; // Numero de correspondencias simetricas
        int nBestMatches; // numero de correspondencias finales

        // Estadisticas de tiempo
        double elapsed_detect1, elapsed_detect2, elapsed_knn1, elapsed_knn2;
        double elapsed_symMatches, elapsed_sortMatches, elapsed_bestMatches;
        double matchPercentage;


};

#endif