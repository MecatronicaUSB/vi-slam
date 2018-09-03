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

void Matcher::computeMatches()
{
    detectFeatures(); //Detectar caracteristicas

    // to store matches temporarily
	// match using desired matcher
	matcher->knnMatch(descriptors_1, descriptors_2, matches, 2);
	// save in global class variable

    // Descartar con distancia euclidiana (revisar los filtros de distancia)
    double nn_match_ratio = 0.8f; // Nearest-neighbour matching ratio
    for(unsigned i = 0; i < matches.size(); i++) {
        if(matches[i][0].distance < nn_match_ratio * matches[i][1].distance) {
            matched1.push_back(keypoints_1[matches[i][0].queryIdx]);
            matched2.push_back(keypoints_2[matches[i][0].trainIdx]);
        }
    }

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
