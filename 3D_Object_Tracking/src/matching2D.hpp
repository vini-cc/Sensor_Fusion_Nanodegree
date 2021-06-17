#ifndef matching2D_hpp
#define matching2D_hpp

#include <stdio.h>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"


void detKeypointsHarris(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, double &t_det, bool bVis=false);
void detKeypointsShiTomasi(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, double &t_det, bool bVis=false);
void detKeypointsModern(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, std::string detectorType, double &t_det, bool bVis=false);
void detKeypointsFAST(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis=false); // ~NEW
void detKeypointsSIFT(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis=false); // ~NEW
void detKeypointsBRISK(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis=false); // ~NEW
void detKeypointsAKAZE(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis=false);// ~NEW
void detKeypointsORB(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis=false); // ~NEW
void descKeypoints(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, std::string descriptorType, double &t_desc);
void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource, cv::Mat &descRef,
                      std::vector<cv::DMatch> &matches, std::string descriptorType, std::string matcherType, std::string selectorType, int &totalMatch);

#endif /* matching2D_hpp */
