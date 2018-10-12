#ifndef MATCHERGPU_H_
#define MATCHERGPU_H_

// CPU matcher
#include "Matcher.hpp"

/// CUDA specific libraries
#include <opencv2/cudafilters.hpp>
#include "opencv2/cudafeatures2d.hpp"
#include "opencv2/xfeatures2d/cuda.hpp"
#include "opencv2/cudaimgproc.hpp"
#include "opencv2/cudaarithm.hpp"


class MatcherGPU : public Matcher
{
    public:
        MatcherGPU(int _detector, int _matcher); // Anexar ROI pendiente
        void setGPUFrames(Mat _frame1, Mat _frame2);
        void computeGPUMatches();
        void detectGPUFeatures();
        void setGPUDetector(int _detector);
        void setGPUMatcher(int _matcher);

        bool useGPU; // utilizar detector y matcher basado en GPU
    private:
        
        int detectorType;
        int matcherType;
        Ptr<cuda::Feature2DAsync> detectorGPU;                    //!< Pointer to OpenCV feature extractor
        Ptr<cuda::DescriptorMatcher> matcherGPU;             //!< Pointer to OpenCV feature Matcher
        cuda::GpuMat frame1GPU, frame2GPU;
        cuda::GpuMat keypointsGPU[2];
        cuda::GpuMat descriptorsGPU[2];
       


};



#endif