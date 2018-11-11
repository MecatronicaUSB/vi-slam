#include "../include/MatcherGPU.hpp"
#include <iostream>
#include <cmath>
#include <ctime>

MatcherGPU::MatcherGPU()
{
    setGPUMatcher(0);
}

MatcherGPU::MatcherGPU(int _matcher)
{
    setGPUMatcher(_matcher);
}

void MatcherGPU::setGPUMatcher(int _matcher)
{

    switch (_matcher)
    {
        case USE_BRUTE_FORCE_GPU:
        {   
            matcherGPU = cuda::DescriptorMatcher::createBFMatcher();
            useGPU = true;  
        }
        case USE_BRUTE_FORCE_GPU_HAMMING:
        {   
            matcherGPU = cuda::DescriptorMatcher::createBFMatcher(NORM_HAMMING);
            useGPU = true;  
        }
        default:
        {
            useGPU = false;
            setMatcher(_matcher); // set cpu matcher
            break;
        }
    }
}

void MatcherGPU::computeGPUMatches()  // Calcula las parejas y realiza prueba de simetria
{
    descriptorsGPU[0].release();
    descriptorsGPU[1].release();
    if (useGPU){
        descriptorsGPU[0].upload(descriptors_1);
        descriptorsGPU[1].upload(descriptors_2);
  
        
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

