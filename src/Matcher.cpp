#include "../include/Matcher.hpp"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <ctime>

Matcher::Matcher()
{
    setDetector(0);
    setMatcher(0);
}

Matcher::Matcher(int _detector, int _matcher)
{
    setDetector(_detector);
    setMatcher(_matcher);
}

void Matcher::setFrames(Mat _frame1, Mat _frame2)
{
    frame1 = _frame1; // Copiar apuntadores
    frame2 = _frame2;
    h_size  = frame1.rows;
    w_size = frame1.cols;
}
void Matcher::setDetector(int _detector)
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

void Matcher::setMatcher(int _matcher)
{
    switch (_matcher)
    {
        case USE_BRUTE_FORCE:
        {
            matcher = BFMatcher::create();
            break;
        }
        case USE_FLANN:
        {   
            matcher = FlannBasedMatcher::create();
            break;
        }
        default:
        {
            matcher = FlannBasedMatcher::create();
            break;
        }
    }
}

void Matcher::detectFeatures()
{             
    clock_t begin = clock(); // Tiempo de inicio del codigo
    detector -> detectAndCompute( frame1, Mat(), keypoints_1, descriptors_1 );
    clock_t detect1 = clock(); 
    detector -> detectAndCompute( frame2, Mat(), keypoints_2, descriptors_2 );
    clock_t detect2 = clock();  
    elapsed_detect1 = double(detect1- begin) / CLOCKS_PER_SEC;
    elapsed_detect2 = double(detect2- detect1) / CLOCKS_PER_SEC;                          
    nPointsDetect1 = keypoints_1.size();
    nPointsDetect2 = keypoints_2.size();
}


void Matcher::computeMatches()
{ 
    clock_t begin = clock(); // Tiempo de inicio del codigo
    matcher->knnMatch(descriptors_1, descriptors_2, aux_matches1, 2);
    clock_t knn1 = clock(); 
    matcher->knnMatch(descriptors_2, descriptors_1, aux_matches2, 2);
    clock_t knn2 = clock();  
    elapsed_knn1 = double(knn1- begin) / CLOCKS_PER_SEC;
    elapsed_knn2 = double(knn2- knn1) / CLOCKS_PER_SEC;
 
    
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

int Matcher::bestMatchesFilter(int n_features){
    float winWSize;
    float winHSize;

    

    winWSize = w_size/floor(sqrt(n_features));
    winHSize = h_size/floor(sqrt(n_features));

    //cout <<"Win w size = "<<fixed<<winWSize<<endl;
    //cout <<"Win h size = "<< fixed<<winHSize<<endl;

     // iterator for matches
    std::vector<cv::DMatch> ::iterator matchIterator; // iterator for matches
    float h_final;
    float w_final;

    int root_n;

    root_n = static_cast<int>(floor(sqrt(n_features)));
    //cout <<"root_n = "<< root_n<<endl;
    vector<DMatch> VectorMatches; // Cambio
    DMatch point;
    point.distance = 100000.0f;
    for (int j = 0; j<root_n; j++) {
        VectorMatches.push_back(point);
    }
    

    matchIterator = sortedMatches.begin();

    h_final = winHSize;
    int i;
    for (int j = 0; j<root_n; j++) {
        // if 2 NN has been identified
            
            while (keypoints_1[(*matchIterator).queryIdx].pt.y <= h_final )
            {
                    w_final = winWSize;
                    i = 0;
                    while(keypoints_1[(*matchIterator).queryIdx].pt.x>w_final)
                    {
                         w_final = w_final+winWSize;
                         i++;
                         cout<<"Distance = "<<VectorMatches[i].distance<<endl;
                        cout<<"pos x = "<<VectorMatches[i].distance<<endl;
                     
                    }
                    if((*matchIterator).distance < (VectorMatches[i]).distance)
                    {
                        cout << "Aprueba "<<i<<endl;
                        
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


    nBestMatches = goodMatches.size();
    return goodMatches.size();

    
}

void Matcher::getGrid(int n_features, vector<KeyPoint> &grid_points)
{
    float h_final;
    float w_final;
    float winHSize;
    float winWSize;

    winWSize = w_size/floor(sqrt(n_features));
    winHSize = h_size/floor(sqrt(n_features));

    h_final = winHSize;
    
    int root_n;

    KeyPoint point;
    root_n = static_cast<int>(floor(sqrt(n_features)));
    for (int j = 0; j<root_n; j++)
    {
        // if 2 NN has been identified
        w_final = winWSize;
        for (int i = 0; i < root_n; i++) //Mejorar deslizando desde el medio
        {
            point.pt.x = w_final-winWSize/2;
            point.pt.y = h_final-winHSize/2;
            grid_points.push_back(point);
            w_final = w_final+winWSize;
            
        } 
        //cout<<"w final "<<w_final<<endl;

        h_final = h_final+winHSize;
                
    }

    cout<< "\nSize grid= " <<grid_points.size()<<endl;
    
    

}

void Matcher::getMatches(vector<KeyPoint> &_matched1, vector<KeyPoint> &_matched2)
{
    for(unsigned i = 0; i < matches.size(); i++) {
        _matched1.push_back(keypoints_1[matches[i].queryIdx]);
        _matched2.push_back(keypoints_2[matches[i].trainIdx]);
    }
}


void Matcher::getGoodMatches(vector<KeyPoint> &_matched1, vector<KeyPoint> &_matched2)
{
    for(unsigned i = 0; i < goodMatches.size(); i++) {
        _matched1.push_back(keypoints_1[goodMatches[i].queryIdx]);
        _matched2.push_back(keypoints_2[goodMatches[i].trainIdx]);
    }
}

double Matcher::getMatchPercentage(){
    return 0.0;
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

void Matcher::computeBestmatches(int n_features)
{
        clock_t begin = clock(); // Tiempo de inicio del codigo
        computeSymMatches();
        clock_t sym = clock(); 
        sortMatches();
        clock_t sort = clock(); 
        int matches_found = bestMatchesFilter(n_features);
        clock_t best = clock(); 
        elapsed_symMatches = double(sym- begin) / CLOCKS_PER_SEC;
        elapsed_sortMatches= double(sort- sym) / CLOCKS_PER_SEC;
        elapsed_bestMatches= double(best- sort) / CLOCKS_PER_SEC;
}

void Matcher::printStatistics()
{
    cout<<"\nESTADISTICAS"

    <<"\nPuntos detectados I1: " << nPointsDetect1
    <<"\tPuntos detectados I2: " << nPointsDetect2
    <<"\nNumero de matches simetricos: " << nSymMatches
    <<"\tNumero de matches finales: " << nBestMatches

    <<"\nTiempo de deteccion  I1: " << fixed<< setprecision(3) << elapsed_detect1*1000<<" ms"
    <<"\tTiempo de deteccion  I2: " << fixed<< setprecision(3) << elapsed_detect2*1000<<" ms"
    <<"\nTiempo de knn I1: " << fixed<< setprecision(3) << elapsed_knn1*1000<<" ms"
    <<"\tTiempo de knn I2: " << fixed<< setprecision(3) << elapsed_knn2*1000<<" ms"
    <<"\nTiempo de symMatches: " << fixed<< setprecision(3) << elapsed_symMatches*1000<<" ms"
    <<"\tTiempo de sortMatches " << fixed<< setprecision(3) << elapsed_sortMatches*1000<<" ms"
    <<"\nTiempo de bestMatches " << fixed<< setprecision(3) << elapsed_bestMatches*1000<<" ms"
    <<endl;
    ;

}