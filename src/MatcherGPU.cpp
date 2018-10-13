#include "../include/MatcherGPU.hpp"
#include <iostream>
#include <cmath>
#include <ctime>

MatcherGPU::MatcherGPU(int _detector, int _matcher)
{
    setGPUDetector(_detector);
    setGPUMatcher(_matcher);
}

void MatcherGPU::setGPUFrames(Mat _frame1, Mat _frame2)
{
    if (useGPU){
        frame1GPU.upload(_frame1); // Copiar apuntadores
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
        
        // Loading previous keypoints found
        // Detecting keypoints and computing descriptors
       
        cuda::SURF_CUDA surf;  

        clock_t begin = clock(); // Tiempo de inicio del codigo
        surf(frame1GPU, cuda::GpuMat(), keypointsGPU[0], descriptorsGPU[0]);
        clock_t detect1 = clock(); 
        surf(frame2GPU, cuda::GpuMat(), keypointsGPU[1], descriptorsGPU[1]);
        clock_t detect2 = clock();  
        elapsed_detect1 = double(detect1- begin) / CLOCKS_PER_SEC;
        elapsed_detect2 = double(detect2- detect1) / CLOCKS_PER_SEC;  
              
        // Downloading results
        surf.downloadKeypoints(keypointsGPU[0], keypoints_1);
        surf.downloadKeypoints(keypointsGPU[1], keypoints_2);

       

        }
        else
        {
            Ptr<cuda::ORB> orb = cuda::ORB::create();
            
            
            clock_t begin = clock(); // Tiempo de inicio del codigo
            orb->detectAndCompute(frame1GPU, cuda::GpuMat(), keypoints_1, descriptorsGPU[0]);
            clock_t detect1 = clock(); 
            orb->detectAndCompute(frame2GPU, cuda::GpuMat(), keypoints_2, descriptorsGPU[1]);
            clock_t detect2 = clock();  
            elapsed_detect1 = double(detect1- begin) / CLOCKS_PER_SEC;
            elapsed_detect2 = double(detect2- detect1) / CLOCKS_PER_SEC;  
            nPointsDetect1 = keypoints_1.size();
            nPointsDetect2 = keypoints_2.size();
        }
    }                                    //ROI
   else{
            detectFeatures();
   }

    
}

void MatcherGPU::computeGPUMatches()  // Calcula las parejas y realiza prueba de simetria
{
    if (useGPU){
        clock_t begin = clock(); // Tiempo de inicio del codigo
        matcherGPU->knnMatch(descriptorsGPU[0], descriptorsGPU[1], aux_matches1, 2);
        clock_t knn1 = clock(); 
        matcherGPU->knnMatch(descriptorsGPU[1], descriptorsGPU[0], aux_matches2, 2);   
        clock_t knn2 = clock();  
        elapsed_knn1 = double(knn1- begin) / CLOCKS_PER_SEC;
        elapsed_knn2 = double(knn2- knn1) / CLOCKS_PER_SEC;
        
        
    }
    else{
        computeMatches();
    }
}