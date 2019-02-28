#include "../include/Camera.hpp"
#include <iostream>
#include <iomanip>
//Clase Frame

Frame::Frame(){

    obtainedGradients = false;
    obtainedGoodMatches = false;           
    isKeyFrame = false;
}


Frame::~Frame()
{
   
}
Camera::Camera()
{
    elapsed_detect_sum = 0.0; 
    elapsed_descriptors_sum = 0.0; 
    elapsed_computeGoodMatches_sum = 0.0;
    elapsed_computeGradient_sum = 0.0 ;
    elapsed_computePatches_sum = 0.0; 
    nPointsDetect_sum = 0.0;
    nBestMatches_sum = 0.0; 
}

Camera::Camera(int _detector, int _matcher, int _w_size, int _h_size, int _num_cells, int _length_patch)
{
    initializate(_detector, _matcher, _w_size, _h_size, _num_cells, _length_patch); 
  
}

void Camera::initializate(int _detector, int _matcher, int _w_size, int _h_size, int _num_cells, int _length_path)
{
    w_size = _w_size;
    h_size = _h_size;

    setDetector(_detector);
    setMatcher(_matcher);

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
// Deteccion
void Camera::Update (Mat _grayImage){
    currentFrame = new Frame();
    elapsed_computeGoodMatches = elapsed_computeGradient= elapsed_descriptors =elapsed_detect = 0.0;
    _grayImage.copyTo(currentFrame->grayImage); // Copiar imagen
    
}

int Camera::detectFeatures(){
            
    clock_t begin = clock(); // Tiempo de inicio del codigo
    detector -> detect( currentFrame->grayImage, currentFrame->keypoints);
    clock_t detect1 = clock(); 
    elapsed_detect = double(detect1- begin) / CLOCKS_PER_SEC;                     
    return currentFrame-> keypoints.size();

}

int Camera::detectAndComputeFeatures(){
            
    clock_t begin = clock(); // Tiempo de inicio del codigo
    detector -> detectAndCompute( currentFrame->grayImage,  Mat(), currentFrame->keypoints, currentFrame->descriptors);
    clock_t detect1 = clock(); 
    elapsed_detect = double(detect1- begin) / CLOCKS_PER_SEC;                     
    return currentFrame-> keypoints.size();

}

void Camera::setDetector(int _detector)
{
    switch (_detector) // Añadir brisk para futuras versiones
    {
        case USE_KAZE:
        {
            detector = KAZE::create( false, false, 0.005f); // Normaliza distancia
            //detector.setThreshold(200.0);
            cout << "Using Kaze detector"<<endl;
            break;
        }
        case USE_AKAZE:
        {
            detector = AKAZE::create(AKAZE::DESCRIPTOR_MLDB,
                                      0,  3,
                                      0.002f);
            cout << "Using Akaze detector"<<endl;
            break;
        }
        case USE_SIFT:
        {
            detector = SIFT::create(600); //Numero de feaures
            cout << "Using SIFT detector"<<endl;
            break;
        }
        case USE_SURF:
        {
            detector = SURF::create(1000); // setThreshold
            cout << "Using SURF detector"<<endl;
            break;
        }
        case USE_ORB:
        {
            detector = ORB::create(200);
            cout << "Using ORB detector"<<endl;
            break;
        }
        default:
        {
            detector = KAZE::create(false, false, 0.005);
            cout << "Using Kaze detector"<<endl;
            break;
        }
    }
}

// Matching
void Camera::computeDescriptors()
{
    detector->compute(currentFrame->grayImage, currentFrame->keypoints, currentFrame->descriptors);
}

void Camera::computeGoodMatches()
{
    matcher.clear();
    matcher.setKeypoints(frameList[frameList.size()-1]->keypoints, currentFrame->keypoints);
    matcher.setDescriptors(frameList[frameList.size()-1]->descriptors, currentFrame->descriptors );
    matcher.computeMatches(); // Computa las primeras parejas
    matcher.computeSymMatches();
    matcher.computeBestMatches(n_cells); // Aplicar nnFilter, prueba de simetría, filtrado de celdas
    matcher.getGoodMatches(frameList[frameList.size()-1]->nextGoodMatches, currentFrame->prevGoodMatches) ;// matched 1, 2
    //matcher.printStatistics();
    currentFrame->obtainedGoodMatches = true;

}

void Camera::computeFastMatches()
{
    matcher.clear();
    matcher.setKeypoints(frameList[frameList.size()-1]->keypoints, currentFrame->keypoints);
    matcher.setDescriptors(frameList[frameList.size()-1]->descriptors, currentFrame->descriptors );
    matcher.computeFastMatches(); // Computa las primeras parejas
    matcher.computeBestMatches(n_cells); // Aplicar nnFilter, prueba de simetría, filtrado de celdas
    matcher.getGoodMatches(frameList[frameList.size()-1]->nextGoodMatches, currentFrame->prevGoodMatches);
    currentFrame->obtainedGoodMatches = true;


}

void Camera::setMatcher(int _matcher)
{
    matcher.setMatcher(_matcher);
    matcher.setImageDimensions(w_size, h_size);
}


// Save current Frame

void Camera::saveFrame()
{
    currentFrame->isKeyFrame = true;
    frameList.push_back(currentFrame);

}

// Add keyFrame (or not)

bool Camera::addKeyframe()
{
               
    clock_t cbegin, cdetect, cdescriptors, cgood, cgradient, cpatches;  
    cbegin = clock(); // Tiempo de inicio del codigo
    nPointsDetect =  detectAndComputeFeatures();
    cdetect = clock(); 

    
    if ( (nPointsDetect > 10) && (frameList.size()!= 0)) // Frames disferentes al primero
    { // is a keyframe (maybe)
        //computeDescriptors();
        num_images = num_images+1;
        cdescriptors = clock(); 
        computeGoodMatches();
        cgood = clock();
        //computeGradient();
        cgradient = clock();
        /*
        ObtainPatchesPointsPreviousFrame();
        ObtainDebugPointsPreviousFrame(); // Debug
        */
        //computePatches();
        //computeResiduals();
        cpatches = clock();
        saveFrame();
        nBestMatches =  matcher.goodMatches.size();
    }
    else if ( (nPointsDetect > 1) && (frameList.size() == 0)) // Primer frame
    {
        //computeDescriptors();
        cdescriptors = clock(); 
        cgood = clock();
        //computeGradient();
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

        elapsed_detect_mean = (elapsed_detect_sum)/(num_images) ; 
        elapsed_descriptors_mean = (elapsed_descriptors_sum)/(num_images) ; 
        elapsed_computeGoodMatches_mean = (elapsed_computeGoodMatches_sum)/(num_images) ; 
        elapsed_computeGradient_mean = (elapsed_computeGradient_sum)/(num_images) ; 
        elapsed_computePatches_mean =(elapsed_computePatches_sum)/(num_images) ; 

                
        nPointsDetect_mean = nPointsDetect_sum/(num_images);
        nBestMatches_mean = nBestMatches_sum/(num_images); 
        

        //printStatistics();
    }
  
}


void Camera::printStatistics()
{
   
    /*cout<<"\nTdetect: " << fixed<< setprecision(1) << elapsed_detect*1000<<"ms"
    <<" Tdesc: " << fixed<< setprecision(1) << elapsed_descriptors*1000<<"ms"
    <<" Tmatch: " << fixed<< setprecision(1) << elapsed_computeGoodMatches*1000<<"ms"
    <<" Tgrad: " << fixed<< setprecision(3) << elapsed_computeGradient*1000<<"ms"
    <<" Tpatch: " << fixed<< setprecision(3) << elapsed_computePatches*1000<<"ms"
    <<" Ndetect: "<< nPointsDetect
    <<" Nmatch: " << nBestMatches
    <<" Nimg: " << frameList.size()
    <<std::endl;
    */
   cout<<"\nESTADISTICAS CAMARA"
    <<"\nTdetectM: " << fixed<< setprecision(1) << elapsed_detect_mean*1000<<"ms"
    <<" TdescM: " << fixed<< setprecision(1) << elapsed_descriptors_mean*1000<<"ms"
    <<" TmatchM: " << fixed<< setprecision(1) << elapsed_computeGoodMatches_mean*1000<<"ms"
    <<" TgradM: " << fixed<< setprecision(3) << elapsed_computeGradient_mean*1000<<"ms"
    <<" TpatchM: " << fixed<< setprecision(3) << elapsed_computePatches_mean*1000<<"ms"
    <<" NdetectM: "<< nPointsDetect_mean
    <<" NmatchM: " << nBestMatches_mean
    <<std::endl;
    /*
    <<"\nTiempo de knn I1: " << fixed<< setprecision(3) << elapsed_knn1*1000<<" ms"
    <<"\tTiempo de knn I2: " << fixed<< setprecision(3) << elapsed_knn2*1000<<" ms"
    <<"\nTiempo de symMatches: " << fixed<< setprecision(3) << elapsed_symMatches*1000<<" ms"
    <<"\tTiempo de sortMatches " << fixed<< setprecision(3) << elapsed_sortMatches*1000<<" ms"
    <<"\nTiempo de bestMatches " << fixed<< setprecision(3) << elapsed_bestMatches*1000<<" ms"
    */
    
    ;
}


 
 