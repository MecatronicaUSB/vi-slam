#ifndef MATCHERGPU_H_
#define MATCHERGPU_H_

// CPU matcher
#include "Matcher.hpp"

/// CUDA specific libraries
#include <opencv2/cudafilters.hpp>
#include "opencv2/cudaimgproc.hpp"
#include "opencv2/cudaarithm.hpp"


class MatcherGPU : public Matcher
{
    public:
        MatcherGPU(int _matcher); // Anexar ROI pendiente
        void setGPUFrames(Mat _frame1, Mat _frame2);
        void computeGPUMatches();
        void setGPUMatcher(int _matcher);
        bool useGPU; // utilizar detector y matcher basado en GPU
        int matcherType;
        Ptr<cuda::DescriptorMatcher> matcherGPU;             //!< Pointer to OpenCV feature Matcher
        cuda::GpuMat descriptorsGPU[2] ;


};



#endif