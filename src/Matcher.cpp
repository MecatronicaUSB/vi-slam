#include "../include/Matcher.hpp"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <ctime>

Matcher::Matcher()
{
    setMatcher(0);

}

Matcher::Matcher(int _matcher)
{
    setMatcher(_matcher);
}

void Matcher::clear()
{
  keypoints_1.clear();
  keypoints_2.clear();
  descriptors_1.release();
  descriptors_2.release();
  aux_matches1.clear();
  aux_matches2.clear();
  matches.clear();
  goodMatches.clear();
  sortedMatches.clear();
}
void Matcher::setImageDimensions(int w, int h)
{
    w_size = w;
    h_size = h;
}

void Matcher::setGridSize(int _n_wcells, int _n_hcells)
{
    n_wcells = _n_wcells;
    n_hcells = _n_hcells;
}
void Matcher::setKeypoints(vector<KeyPoint> _keypoints_1, vector<KeyPoint> _keypoints_2)
{

  keypoints_1 = _keypoints_1;
  keypoints_2 = _keypoints_2;
}

void Matcher::setDescriptors(Mat _descriptors_1, Mat _descriptors_2)
{
    descriptors_1 = _descriptors_1;
    descriptors_2 = _descriptors_2;
}


void Matcher::setMatcher(int _matcher)
{
    switch (_matcher)
    {
        case USE_BRUTE_FORCE:
        {
            matcher = BFMatcher::create();
            cout << "Using Brute Force CPU Matcher"<<endl;
            break;
        }
        case USE_BRUTE_FORCE_HAMMING:
        {
            matcher = BFMatcher::create(NORM_HAMMING);
            cout << "Using Brute Force -Hamming CPU Matcher"<<endl;
            break;
        }
        case USE_FLANN:
        {   
            matcher = FlannBasedMatcher::create();
            cout << "Using FLANN CPU Matcher"<<endl;
            break;
        }
        default:
        {
            cout << "Using FLANN CPU Matcher"<<endl;
            matcher = FlannBasedMatcher::create();
            break;
        }
    }
}




void Matcher::computeMatches()
{ 
    matcher->knnMatch(descriptors_1, descriptors_2, aux_matches1, 2);
    matcher->knnMatch(descriptors_2, descriptors_1, aux_matches2, 2);  
}

void Matcher::computeFastMatches()
{ 

    matcher->knnMatch(descriptors_1, descriptors_2, aux_matches1, 2);


    // Descartar con distancia euclidiana (revisar los filtros de distancia)
    double nn_match_ratio = 0.8f; // Nearest-neighbour matching ratio
    
    int removed1;
    
   
    removed1 = nnFilter(aux_matches1, nn_match_ratio);

    std::vector<std::vector<cv::DMatch> >::iterator matchIterator1; // iterator for matches
    for (matchIterator1= aux_matches1.begin();matchIterator1!= aux_matches1.end(); ++matchIterator1) 
    {
        
        if (matchIterator1->size() >= 2) // dos  o mas vecinos
        {
                        
            matches.push_back(DMatch((*matchIterator1)[0].queryIdx,
                        (*matchIterator1)[0].trainIdx,
                        (*matchIterator1)[0].distance));

                    
            
            
        }
    }






    
}

void Matcher::computeSymMatches()  // Calcula las parejas y realiza prueba de simetria
{

   
	// save in global class variable
    
    // Descartar con distancia euclidiana (revisar los filtros de distancia)
    double nn_match_ratio = 0.8f; // Nearest-neighbour matching ratio
    
    int removed1;
    int removed2;
   
    removed1 = nnFilter(aux_matches1, nn_match_ratio);
    removed2 = nnFilter(aux_matches2, nn_match_ratio);
    
    std::vector<std::vector<cv::DMatch> >::iterator matchIterator1; // iterator for matches
    std::vector<std::vector<cv::DMatch> >::iterator matchIterator2; // iterator for matches
    for (matchIterator1= aux_matches1.begin();matchIterator1!= aux_matches1.end(); ++matchIterator1) 
    {
        
        if (matchIterator1->size() >= 2) // dos  o mas vecinos
        {
            
            for (matchIterator2= aux_matches2.begin();matchIterator2!= aux_matches2.end(); ++matchIterator2) 
            {
               
                if (matchIterator1->size() >= 2) // dos  o mas vecinos
                {
                    if ((*matchIterator1)[0].queryIdx == (*matchIterator2)[0].trainIdx && 
                        (*matchIterator2)[0].queryIdx == (*matchIterator1)[0].trainIdx) 
                    {
                        // add symmetrical match
                        
                        matches.push_back(DMatch((*matchIterator1)[0].queryIdx,
                                    (*matchIterator1)[0].trainIdx,
                                    (*matchIterator1)[0].distance));
                        //matched1.push_back(keypoints_1[(*matchIterator1)[0].queryIdx]);
                        //matched2.push_back(keypoints_2[(*matchIterator1)[0].trainIdx]);
                                
                        break; // next match in image 1 -> image 2
                    }
                }
            }
        }
    }

    
    nSymMatches = matches.size();
}



int Matcher::nnFilter(vector<vector<DMatch> > &matches, double nn_ratio)
{
    std::vector<std::vector<cv::DMatch> >::iterator matchIterator; // iterator for matches
    int removed;
    for (matchIterator=matches.begin();matchIterator!= matches.end(); ++matchIterator) {
        // if 2 NN has been identified
        if (matchIterator->size() > 1) 
        {   // check distance ratio
            if ((*matchIterator)[0].distance > nn_ratio*((*matchIterator)[1].distance)) 
            {    
                //std::cout<<(*matchIterator)[0].distance<<"\t"; 
                matchIterator->clear(); // remove match
                removed++;
            }
        } else { // does not have 2 neighbours
            matchIterator->clear(); // remove match
            removed++;
        }
    }
    //std::cout<<endl; 
    return removed;
}

int Matcher::bestMatchesFilter(){
    float winWSize;
    float winHSize;

    

    winWSize = w_size/n_wcells; // Tamaño horizontal de ventana
    winHSize = h_size/n_hcells; // Tamaño vertical de ventana

    //cout <<"Win w size = "<<fixed<<winWSize<<endl;
    //cout <<"Win h size = "<< fixed<<winHSize<<endl;
    

     // iterator for matches
    std::vector<cv::DMatch> ::iterator matchIterator; // iterator for matches
    float h_final;
    float w_final;

    
    vector<DMatch> VectorMatches; // Cambio
    DMatch point;
    // inicializar los resultados en la celdas horizontales como distancias "infinitas"
    point.distance = 100000.0f; // un numero grande
    for (int i = 0; i<n_wcells; i++) {
        VectorMatches.push_back(point);
    }
    

    matchIterator = sortedMatches.begin();

    h_final = winHSize;
    int i;
    for (int j = 0; j<n_hcells; j++) {
        // if 2 NN has been identified
            while (keypoints_1[(*matchIterator).queryIdx].pt.y <= h_final )
            {
                    w_final = winWSize;
                    i = 0;
                    while(keypoints_1[(*matchIterator).queryIdx].pt.x>w_final)
                    {
                         w_final = w_final+winWSize;
                         i++;
                         
                    }
                    if((*matchIterator).distance < (VectorMatches[i]).distance)
                    {
                        //cout << "Aprueba "<<i<<endl;
                        
                        VectorMatches[i].distance = (*matchIterator).distance; // Puede haber un problema de memoria
                        VectorMatches[i].queryIdx = (*matchIterator).queryIdx;
                        VectorMatches[i].imgIdx = (*matchIterator).imgIdx;
                        VectorMatches[i].trainIdx = (*matchIterator).trainIdx;
                    }
                            
                    //cout<<"w final "<<w_final<<endl;
                    ++matchIterator;
                    if (matchIterator == sortedMatches.end() ) break;

            }
            pushBackVectorMatches(VectorMatches);
            resetVectorMatches(VectorMatches);
            h_final = h_final+winHSize;
            if (matchIterator ==sortedMatches.end() ) break;

    }

    //cout<<"Tamaño final = "<<goodMatches.size()<<endl;
    nBestMatches = goodMatches.size();
    return goodMatches.size();

    
}


void Matcher::getMatches(vector<KeyPoint> &_matched1, vector<KeyPoint> &_matched2)
{
    _matched1.clear(); // Borrar datos antiguos
    _matched2.clear(); // Borrar datos antiguos
    for(unsigned i = 0; i < matches.size(); i++) {
        _matched1.push_back(keypoints_1[matches[i].queryIdx]);
        _matched2.push_back(keypoints_2[matches[i].trainIdx]);
    }
}

void Matcher::getIndexesMatches(Frame* _previousFrame, Frame *_currentFrame)
{
    for(unsigned i = 0; i < matches.size(); i++) {
        _previousFrame->kp_next_idx(matches[i].queryIdx) = matches[i].trainIdx;
        _currentFrame->kp_prev_idx(matches[i].trainIdx) = matches[i].queryIdx;
    }
}


void Matcher::getGoodMatches(vector<KeyPoint> &_matched1, vector<KeyPoint> &_matched2)
{
    _matched1.clear(); // Borrar datos antiguos
    _matched2.clear(); // Borrar datos antiguos
    for(unsigned i = 0; i < goodMatches.size(); i++) {
        _matched1.push_back(keypoints_1[goodMatches[i].queryIdx]);
        _matched2.push_back(keypoints_2[goodMatches[i].trainIdx]);
    }
}


void Matcher::resetVectorMatches(vector<DMatch> &Vector)
{
    for (int i = 0 ; i< Vector.size(); i++)
    {
        Vector[i].distance = 100000.0f;
    }
}

void Matcher::pushBackVectorMatches(vector<DMatch> &Vector)
{
      for (int i = 0 ; i< Vector.size(); i++)
    {
        if (Vector[i].distance!= 100000.0f)
        {
            goodMatches.push_back(Vector[i]);
        }
    }
}


void Matcher::sortMatches()
{
    
    Mat yCoord = Mat::zeros(1, matches.size(), CV_32F) ; // vector para almacenar las coordenadas y de los puntos
    Mat ySorted; // vector para almacenar los indices de yCoord con las valores ordenados;

    // almacenar coordenadas en el vector
    for (int i = 0; i<matches.size();i++) 
    {
        yCoord.at<float>(0, i) = keypoints_1[matches[i].queryIdx].pt.y;
    }

    // Ordenar arreglos de coordenadas de forma ascendente, y guardar los indices en ySorted;
    
    cv::sortIdx(yCoord, ySorted, CV_SORT_EVERY_ROW + CV_SORT_ASCENDING); 

    int index; // indice del arreglo de matches
    for (int i = 0; i<matches.size(); i++)
    {
        index = ySorted.at<int>(0, i); // indice
        sortedMatches.push_back(matches[index]);
    }
    
}
void Matcher::computeBestMatches()
{
   

        sortMatches();
   
        int matches_found = bestMatchesFilter();

}

void Matcher::printStatistics()
{
    cout<<"\nESTADISTICAS"
    <<"\nNumero de matches simetricos: " << nSymMatches
    <<"\tNumero de matches finales: " << nBestMatches
    <<"\nTiempo de knn I1: " << fixed<< setprecision(3) << elapsed_knn1*1000<<" ms"
    <<"\tTiempo de knn I2: " << fixed<< setprecision(3) << elapsed_knn2*1000<<" ms"
    <<"\nTiempo de symMatches: " << fixed<< setprecision(3) << elapsed_symMatches*1000<<" ms"
    <<"\tTiempo de sortMatches " << fixed<< setprecision(3) << elapsed_sortMatches*1000<<" ms"
    <<"\nTiempo de bestMatches " << fixed<< setprecision(3) << elapsed_bestMatches*1000<<" ms"
    <<endl;
    ;

}

