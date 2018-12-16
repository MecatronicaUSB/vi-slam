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
    grayImage.clear();
    gradientX.clear();
    gradientY.clear();
    gradient.clear();

    candidatePoints.clear();
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
    w_size[0] = _w_size;
    h_size[0] = _h_size;
    for (int lvl = 1; lvl < 5; lvl++) {
        w_size[lvl] = _w_size >> lvl;
        h_size[lvl] = _h_size >> lvl;
    }
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
    _grayImage.copyTo(currentFrame->grayImage[0]); // Copiar imagen
    // Escalamiento de la imagen para los niveles piramidales
    for (int i=1; i<5; i++) {
        resize(currentFrame->grayImage[i-1], currentFrame->grayImage[i], Size(), 0.5, 0.5);
    }
    
}

int Camera::detectFeatures(){
            
    clock_t begin = clock(); // Tiempo de inicio del codigo
    detector -> detect( currentFrame->grayImage[0], currentFrame->keypoints);
    clock_t detect1 = clock(); 
    elapsed_detect = double(detect1- begin) / CLOCKS_PER_SEC;                     
    return currentFrame-> keypoints.size();

}

int Camera::detectAndComputeFeatures(){
            
    clock_t begin = clock(); // Tiempo de inicio del codigo
    detector -> detectAndCompute( currentFrame->grayImage[0],  Mat(), currentFrame->keypoints, currentFrame->descriptors);
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
            cout << "Using Kaze detector"<<endl;
            break;
        }
        case USE_AKAZE:
        {
            detector = AKAZE::create();
            cout << "Using Akaze detector"<<endl;
            break;
        }
        case USE_SIFT:
        {
            detector = SIFT::create();
            cout << "Using SIFT detector"<<endl;
            break;
        }
        case USE_SURF:
        {
            detector = SURF::create(); // Normaliza distancia
            cout << "Using SURF detector"<<endl;
            break;
        }
        case USE_ORB:
        {
            detector = ORB::create(600);
            cout << "Using ORB detector"<<endl;
            break;
        }
        default:
        {
            detector = KAZE::create();
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
    matcher.computeBestMatches(n_cells); // Aplicar nnFilter, prueba de simetría, filtrado de celdas
    matcher.getGoodMatches(frameList[frameList.size()-1]->nextGoodMatches, currentFrame->prevGoodMatches) ;// matched 1, 2
    //matcher.printStatistics();
    currentFrame->obtainedGoodMatches = true;

}

void Camera::setMatcher(int _matcher)
{
    matcher.setMatcher(_matcher);
    matcher.setImageDimensions(w_size[0], h_size[0]);
}

// Gradiente

void Camera::computeGradient()
{
    for (int lvl = 0; lvl<5; lvl++) {
        Scharr(currentFrame->grayImage[lvl], currentFrame->gradientX[lvl], CV_16S, 1, 0, 3, 0, BORDER_DEFAULT);
        Scharr(currentFrame->grayImage[lvl], currentFrame->gradientY[lvl], CV_16S, 0, 1, 3, 0, BORDER_DEFAULT);
        
        Mat gradientX, gradientY;
        currentFrame->gradientX[lvl].copyTo(gradientX);
        currentFrame->gradientY[lvl].copyTo(gradientY);
        convertScaleAbs(gradientX, gradientX, 1.0, 0.0);
        convertScaleAbs(gradientY, gradientY, 1.0, 0.0);

        addWeighted(gradientX, 0.5, gradientY, 0.5, 0, currentFrame->gradient[lvl]);
    }

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
    nPointsDetect =  detectAndComputeFeatures();
    cdetect = clock(); 

    
    if ( (nPointsDetect > 1) && (frameList.size()!= 0))
    { // is a keyframe (maybe)
        //computeDescriptors();
        num_images = num_images+1;
        cdescriptors = clock(); 
        computeGoodMatches();
        cgood = clock();
        computeGradient();
        cgradient = clock();
        ObtainPatchesPointsPreviousFrame();
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
/*
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
*/
void Camera::computeResiduals()
{
    Residuals.clear(); // limpiar contenedor
    for(int i = 0; i < currentFrame-> prevPatches.size(); i++)
    {
        Mat Residual =  frameList.back()-> nextPatches[i] -currentFrame-> prevPatches[i] ;
        Residuals.push_back(Residual);
    }
    //cout<< "R = " <<Residuals.back()<<endl;
    //cout<< "Rsize = " <<Residuals.size()<<endl;
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

void Camera::ObtainPatchesPointsPreviousFrame() {
    vector<KeyPoint> goodKeypoints;
    
    goodKeypoints = frameList[frameList.size()-1]->nextGoodMatches;
    int num_max_keypoints = goodKeypoints.size();

    // Saves features found
    float factor_depth = 0.0002, factor_lvl;
    float depth_initialization = 1;    

    int lvl = 0;
    factor_lvl = 1 / pow(2, lvl);
    int patch_size_ = 5;
    int start_point = patch_size_ - 1 / 2;

    for (int i=0; i< min(num_max_keypoints, 20); i++) {

            float x = goodKeypoints[i].pt.x;
            float y = goodKeypoints[i].pt.y;



                float z = 1;

                for (int i=x-start_point; i<=x+start_point; i++) {
                    for (int j=y-start_point; j<=y+start_point; j++) {
                        if (i>0 && i<w_size[lvl] && j>0 && j<h_size[lvl]) {
                            Mat pointMat_patch = Mat::ones(1, 4, CV_32FC1);                
                            pointMat_patch.at<float>(0,0) = i;
                            pointMat_patch.at<float>(0,1) = j;
                            pointMat_patch.at<float>(0,2) = z;

                            frameList[frameList.size()-1]->candidatePoints[lvl].push_back(pointMat_patch);
                        
                    
                }
            }
        }
    }
}
 