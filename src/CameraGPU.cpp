#include "../include/CameraGPU.hpp"


CameraGPU::CameraGPU(int _detector, int _matcher, int _w_size, int _h_size, int _num_cells, int _length_patch)
{
    initializateCameraGPU(_detector, _matcher, _w_size, _h_size, _num_cells, _length_patch); 
}


CameraGPU::CameraGPU()
{
    elapsed_detect_sum = 0.0; 
    elapsed_descriptors_sum = 0.0; 
    elapsed_computeGoodMatches_sum = 0.0;
    elapsed_computeGradient_sum = 0.0 ;
    elapsed_computePatches_sum = 0.0; 
    nPointsDetect_sum = 0.0;
    nBestMatches_sum = 0.0; 
}

void CameraGPU::initializateCameraGPU(int _detector, int _matcher, int _w_size, int _h_size, int _num_cells, int _length_path)
{
    w_size[0] = _w_size;
    h_size[0] = _h_size;
    for (int lvl = 1; lvl < 5; lvl++) {
        w_size[lvl] = _w_size >> lvl;
        h_size[lvl] = _h_size >> lvl;
    }
    setGPUDetector(_detector);
    setGPUMatcher(_matcher);

    n_cells = _num_cells;
    w_patch = h_patch = _length_path;
    elapsed_detect_sum = 0.0; 
    elapsed_descriptors_sum = 0.0; 
    elapsed_computeGoodMatches_sum = 0.0;
    elapsed_computeGradient_sum = 0.0 ;
    elapsed_computePatches_sum = 0.0; 
    nPointsDetect_sum = 0.0;
    nBestMatches_sum = 0.0; 
    num_images = 0;
}

void CameraGPU::setGPUDetector(int _detector)
{
    switch (_detector)
    {
        case USE_SURF:
        {   
            useGPU = true;
            detectorType = _detector; 
            cout << "Using SURF detector in GPU"<<endl;
            break;
        }
        case USE_ORB:
        {
            useGPU= true;
            detectorType = _detector;
            cout << "Using ORB detector in GPU"<<endl;
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
        frameGPU.upload(currentFrame->grayImage[0]);
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
            Ptr<cuda::ORB> orb = cuda::ORB::create(1000);

            
            orb->detectAndCompute(frameGPU, cuda::GpuMat(), currentFrame->keypoints, descriptorsGPU); 
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
    matcherGPU.setImageDimensions(w_size[0], h_size[0]);
}

void CameraGPU::computeGPUGoodMatches()
{
    matcherGPU.clear();
    matcherGPU.setKeypoints(frameList[frameList.size()-1]->keypoints, currentFrame->keypoints);
    matcherGPU.setDescriptors(frameList[frameList.size()-1]->descriptors, currentFrame->descriptors );
    matcherGPU.computeGPUMatches(); // Computa las primeras parejas
    matcherGPU.computeBestMatches(n_cells); // Aplicar nnFilter, prueba de simetría, filtrado de celdas
    matcherGPU.getGoodMatches(frameList[frameList.size()-1]->nextGoodMatches, currentFrame->prevGoodMatches) ;// matched 1, 2
    //matcherGPU.printStatistics();
    currentFrame->obtainedGoodMatches = true;

}

bool CameraGPU::addGPUKeyframe()
{
    clock_t cbegin, cdetect, cdescriptors, cgood, cgradient, cpatches;  
    cbegin = clock(); // Tiempo de inicio del codigo
  
    nPointsDetect =  detectAndComputeGPUFeatures();
    cdetect = clock(); 



    
    if ( (nPointsDetect > 1) && (frameList.size()!= 0))
    { // is a keyframe (maybe)
        cdescriptors = cdetect; 
        computeGPUGoodMatches();
        cgood = clock();
        computeGradient();
        cgradient = clock();
        ObtainPatchesPointsPreviousFrame();
        ObtainDebugPointsPreviousFrame(); // Debug
        //computePatches();
        //computeResiduals();
        cpatches = clock();
        saveFrame();
        nBestMatches =  matcherGPU.goodMatches.size();
    }
    else if ( (nPointsDetect > 1) && (frameList.size() == 0)) // Primer frame
    {
        cdescriptors = cdetect; 
        cgood = clock();
        computeGradient();
        cgradient = clock();
        cpatches = clock();
        saveFrame();
        cout <<"First Image detected"<< "list = "<<frameList.size()<<endl;
    }

    if (currentFrame->isKeyFrame && frameList.size()>1){
        elapsed_detect = double(cdetect-cbegin) / CLOCKS_PER_SEC; 
        elapsed_descriptors = double(cdescriptors-cdetect) / CLOCKS_PER_SEC; 
        elapsed_computeGoodMatches = double(cgood-cdescriptors) / CLOCKS_PER_SEC; 
        elapsed_computeGradient = double(cgradient-cgood) / CLOCKS_PER_SEC; 
        elapsed_computePatches = double(cpatches-cgradient) / CLOCKS_PER_SEC; 

        elapsed_detect_sum = elapsed_detect_sum+elapsed_detect; 
        elapsed_descriptors_sum = elapsed_descriptors_sum+elapsed_descriptors; 
        elapsed_computeGoodMatches_sum =elapsed_computeGoodMatches_sum+elapsed_computeGoodMatches;
        elapsed_computeGradient_sum = elapsed_computeGradient_sum +elapsed_computeGradient ;
        elapsed_computePatches_sum = elapsed_computePatches_sum+ elapsed_computePatches ; 
        nPointsDetect_sum = nPointsDetect_sum+nPointsDetect;
        nBestMatches_sum = nBestMatches_sum +nBestMatches; 

        elapsed_detect_mean = (elapsed_detect_sum)/(frameList.size()-1) ; 
        elapsed_descriptors_mean = (elapsed_descriptors_sum)/(frameList.size()-1) ; 
        elapsed_computeGoodMatches_mean = (elapsed_computeGoodMatches_sum)/(frameList.size()-1) ; 
        elapsed_computeGradient_mean = (elapsed_computeGradient_sum)/(frameList.size()-1) ; 
        elapsed_computePatches_mean =(elapsed_computePatches_sum)/(frameList.size()-1) ; 

        nPointsDetect_mean = nPointsDetect_sum/(frameList.size()-1);
        nBestMatches_mean = nBestMatches_sum/(frameList.size()-1); 
        
        //printStatistics();
    }
  
}