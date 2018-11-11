#ifndef MATCHER_H_
#define MATCHER_H_

// Opencv libraries
#include "opencv2/core.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/features2d.hpp"

using namespace cv;
using namespace cv::xfeatures2d;
using namespace std;


// Tipos de emparejadores soportados
enum matcherType
{
    USE_BRUTE_FORCE,
    USE_BRUTE_FORCE_HAMMING,
    USE_FLANN,
    USE_BRUTE_FORCE_GPU,
    USE_BRUTE_FORCE_GPU_HAMMING

};

class Matcher
{
    public:
        Matcher();
        Matcher(int _matcher); // Anexar ROI pendiente
        void setKeypoints(vector<KeyPoint> _keypoints_1, vector<KeyPoint> _keypoints_2);
        void setDescriptors(Mat _descriptors_1, Mat _descriptors_2);
        void setMatcher(int _matcher);
        void setImageDimensions(int w, int h); // dimensiones de la imagen
        void computeMatches();
        void computeSymMatches();
        int bestMatchesFilter ( int n_features);// Dsitrubuye los features uniformemente y selecciona  las mejores parejas
        int nnFilter(vector<vector<DMatch> > &matches, double nn_ratio); // filtro de vecinos mas cercanos
        void resetVectorMatches(vector<DMatch> &matches);
        void pushBackVectorMatches(vector<DMatch> &matches);
        void getMatches(vector<KeyPoint> &_matched1, vector<KeyPoint> &_matched2);
        void getGoodMatches(vector<KeyPoint> &_matched1, vector<KeyPoint> &_matched2);
        void sortMatches();
        double getMatchPercentage();
        void computeBestMatches(int n_features);
        void getGrid(int n_features, vector<KeyPoint> &grid_point);
        void printStatistics();
        void clear();


        vector< vector<DMatch> > aux_matches1; // Vector auxiliar
        vector< vector<DMatch> > aux_matches2; // Vector auxiliar
        vector<DMatch> matches; // correspondencias filtradas
        vector<DMatch> sortedMatches; // correspondencias ordenadas en funcion de la coordenada y del pixel
        vector<DMatch> goodMatches; // correspondencias finales
        Ptr<DescriptorMatcher> matcher;             //!< Pointer to OpenCV feature Matcher
        vector<KeyPoint> keypoints_1, keypoints_2; // Vector para almacenar los puntos detectados con FAST
        Mat descriptors_1, descriptors_2;

        // ---------- Attributes
        int h_size, w_size;
        int nSymMatches; // Numero de correspondencias simetricas
        int nBestMatches; // numero de correspondencias finales

        // Estadisticas de tiempo
        double elapsed_detect1, elapsed_detect2, elapsed_knn1, elapsed_knn2;
        double elapsed_symMatches, elapsed_sortMatches, elapsed_bestMatches;
        double matchPercentage;


};

#endif