#include "../include/Camera.hpp"
#include <iostream>
#include <iomanip>
//Clase Frame

Frame::Frame(){
        
    isKeyFrame = false;
}


Frame::~Frame()
{
   
}
Camera::Camera()
{

}

Camera::Camera(int _detector, int _matcher, int _w_size, int _h_size, int _num_wcells, int _num_hcells )
{
    initializate(_detector, _matcher, _w_size, _h_size, _num_wcells, _num_hcells); 
  
}

void Camera::initializate(int _detector, int _matcher, int _w_size, int _h_size, int _num_wcells, int _num_hcells)
{
    w_size = _w_size;
    h_size = _h_size;
     
    n_wcells = _num_wcells;
    n_hcells = _num_hcells;

    setDetector(_detector);
    setMatcher(_matcher);

   

}
// Deteccion
void Camera::Update (Mat _grayImage){
    currentFrame = new Frame();
    _grayImage.copyTo(currentFrame->grayImage); // Copiar imagen
    
}

int Camera::detectFeatures(){
            
    detector -> detect( currentFrame->grayImage, currentFrame->keypoints);
    
    return currentFrame-> keypoints.size();

}

int Camera::detectAndComputeFeatures(){
            
    detector -> detectAndCompute( currentFrame->grayImage,  Mat(), currentFrame->keypoints, currentFrame->descriptors);                  
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
    matcher.computeBestMatches(); // Aplicar nnFilter, prueba de simetría, filtrado de celdas
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
    matcher.computeBestMatches(); // Aplicar nnFilter, prueba de simetría, filtrado de celdas
    matcher.getGoodMatches(frameList[frameList.size()-1]->nextGoodMatches, currentFrame->prevGoodMatches);
    currentFrame->obtainedGoodMatches = true;


}

void Camera::setMatcher(int _matcher)
{
    matcher.setMatcher(_matcher);
    matcher.setImageDimensions(w_size, h_size);
    matcher.setGridSize(n_wcells, n_hcells);
}


// Save current Frame

void Camera::saveFrame()
{
    currentFrame->isKeyFrame = true;
    frameList.push_back(currentFrame);

}






 
 