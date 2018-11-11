#include "../include/CameraGPU.hpp"


CameraGPU::CameraGPU(int _detector, int _matcher, int _w_size, int _h_size)
{
    initializateCameraGPU(_detector, _matcher, _w_size, _h_size); 
  
    n_cells = 144;
    w_patch = h_patch = 3;
}

void Camera::initializateCameraGPU(int _detector, int _matcher, int _w_size, int _h_size)
{
    w_size = _w_size;
    h_size = _h_size;
    setGPUDetector(_detector);
    setGPUMatcher(_matcher);
}

void CameraGPU::setGPUDetector(int _detector)
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

int CameraGPU::detectAndComputeGPUFeatures()
{

    // Liberarobjetos de la  memoria de GPU
    keypointsGPU.release();
    descriptorsGPU.release();
    frameGPU.release();

    if (useGPU)
    {
        frameGPU.upload(currentFrame->grayImage);
        if (detectorType == USE_SURF)
        {
            // Loading previous keypoints found
            // Detecting keypoints and computing descriptors
        
            cuda::SURF_CUDA surf;  

            surf(frameGPU, cuda::GpuMat(), keypointsGPU, descriptorsGPU);
                
            // Downloading results
            surf.downloadKeypoints(keypointsGPU, currentFrame->keypoints);
            descriptorsGPU.download(currentFrame->descriptors);
            surf.releaseMemory();

        }
        else
        {
            Ptr<cuda::ORB> orb = cuda::ORB::create();

            
            orb->detect(frameGPU, cuda::GpuMat(), currentFrame->keypoints, descriptorsGPU); 
            descriptorsGPU.download(currentFrame->descriptors);
            orb.release();
        }
                      
         nPointsDetect = currentFrame-> keypoints.size();
       
    }                                    //ROI
   else
   {
        nPointsDetect = detectFeatures();
   }

    return nPointsDetect;

}

void CameraGPU::setGPUMatcher (int _matcher)
{
    matcherGPU.setGPUMatcher(_matcher);
    matcherGPU.setImageDimensions(w_size, h_size);
}

void CameraGPU::computeGPUGoodMatches()
{
    matcherGPU.clear();
    matcherGPU.setKeypoints(frameList[frameList.size()-1]->keypoints, currentFrame->keypoints);
    matcherGPU.setDescriptors(frameList[frameList.size()-1]->descriptors, currentFrame->descriptors );
    matcherGPU.computeGPUMatches(); // Computa las primeras parejas
    matcherGPU.computeBestMatches(n_cells); // Aplicar nnFilter, prueba de simetría, filtrado de celdas
    matcherGPU.getGoodMatches(frameList[frameList.size()-1]->nextGoodMatches, currentFrame->prevGoodMatches) ;// matched 1, 2
    //matcher.printStatistics();
    currentFrame->obtainedGoodMatches = true;

}

bool CameraGPU::addGPUKeyFrame()
{
    clock_t cbegin, cdetect, cdescriptors, cgood, cgradient, cpatches;  
    cbegin = clock(); // Tiempo de inicio del codigo
    nPointsDetect =  detectAndComputeGPUFeatures();
    cdetect = clock(); 

    
    if ( (nPointsDetect > 350) && (frameList.size()!= 0))
    { // is a keyframe (maybe)
        cdescriptors = cdetect; 
        computeGPUGoodMatches();
        cgood = clock();
        computeGradient();
        cgradient = clock();
        computePatches();
        computeResiduals();
        cpatches = clock();
        saveFrame();
    }
    else if ( (nPointsDetect > 350) && (frameList.size() == 0)) // Primer frame
    {
        cdescriptors = cdetect; 
        cgood = clock();
        computeGradient();
        cgradient = clock();
        cpatches = clock();
        saveFrame();
        cout <<"First Image detected"<< "list = "<<frameList.size()<<endl;
    }

    if (currentFrame->isKeyFrame){
        elapsed_detect = double(cdetect-cbegin) / CLOCKS_PER_SEC; 
        elapsed_descriptors = double(cdescriptors-cdetect) / CLOCKS_PER_SEC; 
        elapsed_computeGoodMatches = double(cgood-cdescriptors) / CLOCKS_PER_SEC; 
        elapsed_computeGradient = double(cgradient-cgood) / CLOCKS_PER_SEC; 
        elapsed_computePatches = double(cpatches-cgradient) / CLOCKS_PER_SEC; 
        printStatistics();
    }
  
}