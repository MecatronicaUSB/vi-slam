#include "../include/MatcherGPU.hpp"
#include <iostream>
#include <cmath>


MatcherGPU::MatcherGPU(int _detector, int _matcher)
{
    setGPUDetector(_detector);
    setGPUMatcher(_matcher);
}

void MatcherGPU::setGPUFrames(Mat _frame1, Mat _frame2)
{
    if (useGPU){
        frame1GPU.upload( _frame1); // Copiar apuntadores
        frame2GPU.upload(_frame2);
        
    }
    else{
        frame1 = _frame1; // Copiar apuntadores
        frame2 = _frame2;    
    }

    h_size  = frame1.rows;
    w_size = frame1.cols;
}
void MatcherGPU::setGPUDetector(int _detector)
{
    switch (_detector)
    {
        case USE_SURF:
        {   
            useGPU = true;
            detectorType = _detector; 
            break;
        }
        case USE_ORB:
        {
            useGPU= true;
            detectorType = _detector;
            break;
        }
        default:
        {
            useGPU = false;
            setDetector(_detector); // utilizar detector basado en CPU
            break;
        }
    }
}

void MatcherGPU::setGPUMatcher(int _matcher)
{

    switch (_matcher)
    {
        case USE_BRUTE_FORCE:
        {   
            if(useGPU){
                if (detectorType == USE_ORB) 
                    matcherGPU = cuda::DescriptorMatcher::createBFMatcher(NORM_HAMMING);
                else
                    matcherGPU = cuda::DescriptorMatcher::createBFMatcher();
            }
            else
                matcher = BFMatcher::create();
            break;
        }
        case USE_FLANN:
        {   
             if(useGPU)
             {
                std::cout<<"ERROR! FLANN no esta implementado para GPU"
                 <<"\n Se seleccionara Fuerza Bruta"<<endl;
                if (detectorType == USE_ORB) 
                    matcherGPU = cuda::DescriptorMatcher::createBFMatcher(NORM_HAMMING);
                else matcherGPU = cuda::DescriptorMatcher::createBFMatcher();
             }
             else matcher = FlannBasedMatcher::create();
            break;
        }
        default:
        {
            matcher = BFMatcher::create();
            break;
        }
    }
}

    


void MatcherGPU::detectGPUFeatures()
{    
    if (useGPU)
    {
        if (detectorType == USE_SURF)
        {
        cuda::SURF_CUDA surf;        
        // Loading previous keypoints found

        vector< vector<DMatch> > aux_matches1; // Vector auxiliar
        vector< vector<DMatch> > aux_matches2; // Vector auxiliar
        // Detecting keypoints and computing descriptors
        surf(frame1GPU, cuda::GpuMat(), keypointsGPU[0], descriptorsGPU[0]);
        surf(frame2GPU, cuda::GpuMat(), keypointsGPU[1], descriptorsGPU[1]);
        
        // Downloading results
        surf.downloadKeypoints(keypointsGPU[0], keypoints_1);
        surf.downloadKeypoints(keypointsGPU[1], keypoints_2);

        }
        else
        {
            Ptr<cuda::ORB> orb = cuda::ORB::create();
            orb->detectAndCompute(frame1GPU, cuda::GpuMat(), keypoints_1, descriptorsGPU[0]);
            orb->detectAndCompute(frame2GPU, cuda::GpuMat(), keypoints_2, descriptorsGPU[1]);

        }
    }                                    //ROI
   else{

   }
}

void MatcherGPU::computeGPUMatches()  // Calcula las parejas y realiza prueba de simetria
{
    if (useGPU){
        
    }
}