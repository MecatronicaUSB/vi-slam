#ifndef MATCHER_H_
#define MATCHER_H_

// Opencv libraries
#include "opencv2/core.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/features2d.hpp"
#include <map>

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
        Mat grayImage; //!< imagen en escala de grises
        vector<KeyPoint> keypoints;        //!< keypoints dectados en la imagen
        vector<KeyPoint>  prevGoodMatches; //!< matches con la imagen anterior
        vector<KeyPoint>  nextGoodMatches; //!< matches con la imagen siguiente
        vector<KeyPoint>  prevMatches; //!< matches con la imagen anterior
        vector<KeyPoint>  nextMatches; //!< matches con la imagen siguiente
        Mat descriptors;                   //!< keypoints de la imagen
        

        int idFrame;      //!< identificador del frame
        double imageTime; //!< tiempo en el que fue tomado la imagen



        bool obtainedGoodMatches; //!< flag de obtencion de matches    
        bool isKeyFrame;          //!< flag de keyframe
        
        
        // alias to clarify map usage below
        using kp_idx_t = size_t;
        using landmark_idx_t = size_t;
        using img_idx_t = size_t;

        std::map<kp_idx_t, kp_idx_t> kp_next_match; // seypoint to 3d points
        std::map<kp_idx_t, kp_idx_t> kp_prev_match; // seypoint to 3d points
        std::map<kp_idx_t, landmark_idx_t> kp_landmark; // Mapa que relaciona el feature con su landmark

        bool kp_next_exist(size_t kp_idx) { return kp_next_match.count(kp_idx) > 0; }
        bool kp_prev_exist(size_t kp_idx) { return kp_prev_match.count(kp_idx) > 0; }
        bool kp_3d_exist(size_t kp_idx) { return kp_landmark.count(kp_idx) > 0; }

        landmark_idx_t& kp_3d_idx(size_t kp_idx) { return kp_landmark[kp_idx]; } // indice del vector de landmarks
        kp_idx_t& kp_next_idx(size_t kp_idx) { return kp_next_match[kp_idx]; } // indice del vector de landmarks
        kp_idx_t& kp_prev_idx(size_t kp_idx) { return kp_prev_match[kp_idx]; } // indice del vector de landmarks
        
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
        void setGridSize(int _n_wcells, int _n_h_cells); // dimensiones de la imagen
        void computeMatches();
        void computeFastMatches();
        void computeSymMatches();
        int bestMatchesFilter ();// Dsitrubuye los features uniformemente y selecciona  las mejores parejas
        int nnFilter(vector<vector<DMatch> > &matches, double nn_ratio); // filtro de vecinos mas cercanos
        void resetVectorMatches(vector<DMatch> &matches);
        void pushBackVectorMatches(vector<DMatch> &matches);
        void getMatches(vector<KeyPoint> &_matched1, vector<KeyPoint> &_matched2);
        void getGoodMatches(vector<KeyPoint> &_matched1, vector<KeyPoint> &_matched2);
        void getIndexesMatches(Frame* _previousFrame, Frame *_currentFrame);
        void sortMatches();
        
        void computeBestMatches();
        
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
        int n_wcells, n_hcells; // Numeros de celdas horizontales y verticales

        // Estadisticas de tiempo
        double elapsed_detect1, elapsed_detect2, elapsed_knn1, elapsed_knn2;
        double elapsed_symMatches, elapsed_sortMatches, elapsed_bestMatches;
        double matchPercentage;
        


};

#endif