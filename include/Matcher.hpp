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
    USE_SURF,
    USE_FAST // Sin matcher
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
        Matcher(int _detector, int _matcher); // Anexar ROI pendiente
        void setFrames(Mat _frame1, Mat _frame2);
        void setDetector(int _detector);
        void setMatcher(int _matcher);
        void computeMatches();
        void detectFeatures();
        void getGoodMatches(vector<KeyPoint> &_matched1, vector<KeyPoint> &_matched2);
        double getMatchPercentage();




    private:
        // ---------- Attributes
        vector< vector<DMatch> > matches; // Cambio
        Ptr<Feature2D> detector;                    //!< Pointer to OpenCV feature extractor
        Ptr<DescriptorMatcher> matcher;             //!< Pointer to OpenCV feature Matcher
        Mat frame1, frame2;
        vector<KeyPoint> matched1, matched2;
        vector<KeyPoint> keypoints_1, keypoints_2; // Vector para almacenar los puntos detectados con FAST
        Mat descriptors_1, descriptors_2;
        double matchPercentage;


};



