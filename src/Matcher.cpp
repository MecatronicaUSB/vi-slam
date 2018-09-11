#include "../include/Matcher.hpp"
#include <iostream>


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
    cout << h_size<<endl;
    cout << w_size<<endl;
}
void Matcher::setDetector(int _detector)
{
    switch (_detector)
    {
        case USE_KAZE:
        {
            detector = KAZE::create();
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
            detector = SURF::create(400);
            break;
        }
        case USE_ORB:
        {
            detector = ORB::create();
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
{                                        //ROI
    detector -> detectAndCompute( frame1, Mat(), keypoints_1, descriptors_1 );
    detector -> detectAndCompute( frame2, Mat(), keypoints_2, descriptors_2 );

}

void Matcher::computeSymMatches()  // Calcula las parejas y realiza prueba de simetria
{
    detectFeatures(); //Detectar caracteristicas

    // to store matches temporarily
	// match using desired matcher
    vector< vector<DMatch> > aux_matches1; // Vector auxiliar
    vector< vector<DMatch> > aux_matches2; // Vector auxiliar
	matcher->knnMatch(descriptors_1, descriptors_2, aux_matches1, 2);
    matcher->knnMatch(descriptors_2, descriptors_1, aux_matches2, 2);
	// save in global class variable
    std::cout<< "Numero de puntos detectados en 2 = "<<keypoints_2.size()<<endl;
    
    // Descartar con distancia euclidiana (revisar los filtros de distancia)
    double nn_match_ratio = 0.8f; // Nearest-neighbour matching ratio
    int removed1;
    int removed2;
    removed1 = nn_filter(aux_matches1, nn_match_ratio);
    removed2 = nn_filter(aux_matches2, nn_match_ratio);

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
                        matched1.push_back(keypoints_1[(*matchIterator1)[0].queryIdx]);
                        matched2.push_back(keypoints_2[(*matchIterator1)[0].trainIdx]);
                                
                        break; // next match in image 1 -> image 2
                    }
                }
            }
        }
    }

}

int Matcher::nn_filter(vector<vector<DMatch> > &matches, double nn_ratio)
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

int Matcher::bestPairsFilter(int n_features){
    
}
void Matcher::getGoodMatches(vector<KeyPoint> &_matched1, vector<KeyPoint> &_matched2)
{
    for(unsigned i = 0; i < matched1.size(); i++) {
        _matched1.push_back(matched1[i]);
        _matched2.push_back(matched2[i]);
    }
}

double Matcher::getMatchPercentage(){
    return 0.0;
}
