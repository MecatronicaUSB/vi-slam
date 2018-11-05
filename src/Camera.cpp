#include "../include/Camera.hpp"
#include <iostream>
#include <iomanip>
//Clase Frame

Frame::Frame(){

    obtainedGradients = false;
    obtainedGoodMatches = false;           
    isKeyFrame = false;
}

Camera::Camera(int _detector, int _matcher, int _w_size, int _h_size)
{
    initializate(_detector, _matcher, _w_size, _h_size); 
  
    n_cells = 144;
    w_patch = h_patch = 3;
}

void Camera::initializate(int _detector, int _matcher, int _w_size, int _h_size)
{
    w_size = _w_size;
    h_size = _h_size;
    setDetector(_detector);
    setMatcher(_matcher);
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

void Camera::setDetector(int _detector)
{
    switch (_detector)
    {
        case USE_KAZE:
        {
            detector = KAZE::create(); // Normaliza distancia
            break;
        }
        case USE_AKAZE:
        {
            detector = AKAZE::create();
            break;
        }
        case USE_SIFT:
        {
            detector = SIFT::create();
            break;
        }
        case USE_SURF:
        {
            detector = SURF::create(400); // Normaliza distancia
            break;
        }
        case USE_ORB:
        {
            detector = ORB::create(1500);
            break;
        }
        default:
        {
            detector = KAZE::create();
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
    matcher.computeBestMatches(n_cells); // Aplicar nnFilter, prueba de simetría, filtrado de celdas
    matcher.getGoodMatches(frameList[frameList.size()-1]->nextGoodMatches, currentFrame->prevGoodMatches) ;// matched 1, 2
    //matcher.printStatistics();
    currentFrame->obtainedGoodMatches = true;

}

void Camera::setMatcher(int _matcher)
{
    matcher.setMatcher(_matcher);
    matcher.setImageDimensions(w_size, h_size);
}

// Gradiente

void Camera::computeGradient()
{

    Scharr(currentFrame->grayImage, currentFrame->gradientX, CV_16S, 1, 0, 3, 0, BORDER_DEFAULT);
    Scharr(currentFrame->grayImage, currentFrame->gradientY, CV_16S, 0, 1, 3, 0, BORDER_DEFAULT);
    
    Mat gradientX, gradientY;
    currentFrame->gradientX.copyTo(gradientX);
    currentFrame->gradientY.copyTo(gradientY);
    convertScaleAbs(gradientX, gradientX, 1.0, 0.0);
    convertScaleAbs(gradientY, gradientY, 1.0, 0.0);

    addWeighted(gradientX, 0.5, gradientY, 0.5, 0, currentFrame->gradient);

    currentFrame->obtainedGradients = true;
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
    nPointsDetect =  detectFeatures();
    cdetect = clock(); 

    
    if ( (nPointsDetect > 350) && (frameList.size()!= 0))
    { // is a keyframe (maybe)
        computeDescriptors();
        cdescriptors = clock(); 
        computeGoodMatches();
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
        computeDescriptors();
        cdescriptors = clock(); 
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

void Camera::computePatches()  // Crear parches de la imagen anterior y la actual
{ 
    Mat patch1(w_patch, h_patch, CV_16SC1);
    Mat patch2(w_patch, h_patch, CV_16SC1);
    int cx1, cy1; // ubicacion del feature dentro del parche (centro del parche)
    int start_x1, start_y1;
    int index_x1, index_y1;
    int cx2, cy2; // ubicacion del feature dentro del parche (centro del parche)
    int start_x2, start_y2;
    int index_x2, index_y2;
    int i, j;
    for (int index = 0 ; index < currentFrame->prevGoodMatches.size();index++ ) // imagen anterior
    {
        cx1 = frameList.back()->nextGoodMatches[index].pt.x;
        cy1 = frameList.back()->nextGoodMatches[index].pt.y;
        start_x1 = cx1-int((w_patch-1)/2);
        start_y1 = cy1-int((h_patch-1)/2);
        cx2 = currentFrame->prevGoodMatches[index].pt.x;
        cy2 = currentFrame->prevGoodMatches[index].pt.y;
        start_x2 = cx2-int((w_patch-1)/2);
        start_y2 = cy2-int((h_patch-1)/2);

        for ( i = 0; i< h_patch ; i++) // fila
        {
            index_x1 = start_x1+i;
            index_x2 = start_x2+i;
            for (j = 0; j< w_patch ; j++) // columna
            {
                index_y1 = start_y1+j;
                index_y2 = start_y2+j;
                patch1.at<char16_t>(i, j) = frameList.back()->grayImage.at<uchar>(index_y1, index_x1);
                patch2.at<char16_t>(i, j) = currentFrame->grayImage.at<uchar>(index_y2, index_x2);

            }
        }
        frameList.back()-> nextPatches.push_back(patch1);
        currentFrame-> prevPatches.push_back(patch2);

    }

}

void Camera::computeResiduals()
{
    Residuals.clear(); // limpiar contenedor
    for(int i = 0; i < currentFrame-> prevPatches.size(); i++)
    {
        Mat Residual =  frameList.back()-> nextPatches[i] -currentFrame-> prevPatches[i] ;
        Residuals.push_back(Residual);
    }
    cout<< "R = " <<Residuals.back()<<endl;
    cout<< "Rsize = " <<Residuals.size()<<endl;
}



void Camera::printStatistics()
{
   
    cout<<"\nTdetect: " << fixed<< setprecision(1) << elapsed_detect*1000<<"ms"
    <<" Tdesc: " << fixed<< setprecision(1) << elapsed_descriptors*1000<<"ms"
    <<" Tmatch: " << fixed<< setprecision(1) << elapsed_computeGoodMatches*1000<<"ms"
    <<" Tgrad: " << fixed<< setprecision(3) << elapsed_computeGradient*1000<<"ms"
    <<" Tpatch: " << fixed<< setprecision(3) << elapsed_computePatches*1000<<"ms"
    <<" Ndetect: "<< nPointsDetect
    <<" Nmatch: " << matcher.nBestMatches
    <<" Nimg: " << frameList.size()
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

