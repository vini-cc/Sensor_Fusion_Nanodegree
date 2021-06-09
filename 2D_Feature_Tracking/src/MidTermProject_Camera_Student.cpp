/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>
#include <string>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"

using namespace std;

/* MAIN PROGRAM */
int main(int argc, const char *argv[])
{

    // Variables to create results for tasks 7, 8 and 9.

    double t_desc = 0;
    double t_det = 0;
    // double t_total = t_desc + t_det;
    int carKeyP = 0;
    int totalMatch = 0;



    // data location
    string dataPath = "../";

    // camera
    string imgBasePath = dataPath + "images/";
    string imgPrefix = "KITTI/2011_09_26/image_00/data/000000"; // left camera, color
    string imgFileType = ".png";

    int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 9;   // last file index to load
    int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

    // misc
    int dataBufferSize = 2;       // no. of images which are held in memory (ring buffer) at the same time
    vector<DataFrame> dataBuffer; // list of data frames which are held in memory at the same time
    bool bVis = false;            // visualize results

    int num_detectorType = 0;
    string detectorType;



    cout << "Choose your Detector type (write just the number): \n" // Alphabetically
        << "Press 1 -> AKAZE\n"
        << "Press 2 -> BRISK\n"
        << "Press 3 -> FAST\n"
        << "Press 4 -> HARRIS\n"
        << "Press 5 -> ORB\n"
        << "Press 6 -> SHITOMASI\n"
        << "Press 7 -> SIFT\n"
        << endl;
    cin >> num_detectorType;


    switch(num_detectorType) {
        case 1:
            detectorType = "AKAZE";
            cout << "You selected Detector " << num_detectorType << "->" << detectorType << endl;
            break;
        case 2:
            detectorType = "BRISK";
            cout << "You selected Detector " << num_detectorType << "->" << detectorType << endl;
            break;
        case 3:
            detectorType = "FAST";
            cout << "You selected Detector " << num_detectorType << "->" << detectorType << endl;
            break;
        case 4:
            detectorType = "HARRIS";
            cout << "You selected Detector " << num_detectorType << "->" << detectorType << endl;
            break;
        case 5:
            detectorType = "ORB";
            cout << "You selected Detector " << num_detectorType << "->" << detectorType << endl;
            break;
        case 6:
            detectorType = "SHITOMASI";
            cout << "You selected Detector " << num_detectorType << "->" << detectorType << endl;
            break;
        case 7:
            detectorType = "SIFT";
            cout << "You selected Detector " << num_detectorType << "->" << detectorType << endl;
            break;
    }

    int num_descriptorType = 0;
    string descriptorType;



    cout << "Choose your Descriptor type (write just the number): \n" // Alphabetically
        << "Press 1 -> AKAZE\n"
        << "Press 2 -> BRIEF\n"
        << "Press 3 -> BRISK\n"
        << "Press 4 -> FREAK\n"
        << "Press 5 -> ORB\n"
        << "Press 6 -> SIFT\n"
        << endl;
    cin >> num_descriptorType;


    switch(num_descriptorType) {
        case 1:
            descriptorType = "AKAZE";
            cout << "You selected Descriptor " << num_descriptorType << "->" << descriptorType << endl;
            break;
        case 2:
            descriptorType = "BRIEF";
            cout << "You selected Descriptor " << num_descriptorType << "->" << descriptorType << endl;
            break;
        case 3:
            descriptorType = "BRISK";
            cout << "You selected Descriptor " << num_descriptorType << "->" << descriptorType << endl;
            break;
        case 4:
            descriptorType = "FREAK";
            cout << "You selected Descriptor " << num_descriptorType << "->" << descriptorType << endl;
            break;
        case 5:
            descriptorType = "ORB";
            cout << "You selected Descriptor " << num_descriptorType << "->" << descriptorType << endl;
            break;
        case 6:
            descriptorType = "SIFT";
            cout << "You selected Descriptor " << num_descriptorType << "->" << descriptorType << endl;
            break;
    }




    /* MAIN LOOP OVER ALL IMAGES */

    for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++)
    {
        /* LOAD IMAGE INTO BUFFER */

        // assemble filenames for current index
        ostringstream imgNumber;
        imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
        string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

        // load image from file and convert to grayscale
        cv::Mat img, imgGray;
        img = cv::imread(imgFullFilename);
        cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

        //// STUDENT ASSIGNMENT
        //// TASK MP.1 -> replace the following code with ring buffer of size dataBufferSize

        // push image into data frame buffer
        DataFrame frame;
        frame.cameraImg = imgGray;
        dataBuffer.push_back(frame);

        //// EOF STUDENT ASSIGNMENT
        cout << "#1 : LOAD IMAGE INTO BUFFER done" << endl;

        /* DETECT IMAGE KEYPOINTS */

        // extract 2D keypoints from current image
        vector<cv::KeyPoint> keypoints; // create empty feature list for current image

        //// STUDENT ASSIGNMENT
        //// TASK MP.2 -> add the following keypoint detectors in file matching2D.cpp and enable string-based selection based on detectorType
        //// -> HARRIS, FAST, BRISK, ORB, AKAZE, SIFT

        // Despite the usage of if/else if, the correct probably is a switch/break statement. But I did that way first, so let it be :)
        // I'm adding t_det here just as a try..

        if (detectorType.compare("SHITOMASI") == 0){
            detKeypointsShiTomasi(keypoints, imgGray, t_det, false);
        }else if (detectorType.compare("HARRIS") == 0) {
            detKeypointsHarris(keypoints, imgGray, t_det, false);
        }else if (detectorType.compare("FAST") == 0 ||
                detectorType.compare("BRISK") == 0 ||
                detectorType.compare("ORB") == 0 ||
                detectorType.compare("AKAZE") == 0 ||
                detectorType.compare("SIFT") == 0) {
            detKeypointsModern(keypoints, imgGray, detectorType, t_det, false);
        } else {
            cout << detectorType << "is an invalid detectorType." << endl;
        }
        //// EOF STUDENT ASSIGNMENT

        //// STUDENT ASSIGNMENT
        //// TASK MP.3 -> only keep keypoints on the preceding vehicle

        // only keep keypoints on the preceding vehicle
        bool bFocusOnVehicle = true;
        cv::Rect vehicleRect(535, 180, 180, 150);
        if (bFocusOnVehicle)
        {
            // 1st attempt:
            auto keyP = keypoints.begin();
            while (keyP != keypoints.end()) {

                if (vehicleRect.contains(keyP->pt)) { // Just to confirm that the points are inside the rectangle.
                    keyP++;
                    continue;
                }
                // After the op, keyP are erased.
                keyP = keypoints.erase(keyP);
            }      
        }

        //// EOF STUDENT ASSIGNMENT

        // optional : limit number of keypoints (helpful for debugging and learning)
        bool bLimitKpts = false;
        if (bLimitKpts)
        { //Seems to me that this is trully necessary to keep things working properly and to compare it later.
            // Is it really optional?!
            int maxKeypoints = 50;

            if (detectorType.compare("SHITOMASI") == 0)
            { // there is no response info, so keep the first 50 as they are sorted in descending quality order
                keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
            }
            cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
            cout << " NOTE: Keypoints have been limited!" << endl;
        }

        // push keypoints and descriptor for current frame to end of data buffer
        (dataBuffer.end() - 1)->keypoints = keypoints;
        cout << "#2 : DETECT KEYPOINTS done" << endl;
        carKeyP = carKeyP + keypoints.size();

        /* EXTRACT KEYPOINT DESCRIPTORS */

        //// STUDENT ASSIGNMENT
        //// TASK MP.4 -> add the following descriptors in file matching2D.cpp and enable string-based selection based on descriptorType
        //// -> BRIEF, ORB, FREAK, AKAZE, SIFT

        cv::Mat descriptors;
        string descriptorType = "BRISK"; // BRIEF, ORB, FREAK, AKAZE, SIFT
        descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, descriptorType, t_desc);
        //// EOF STUDENT ASSIGNMENT

        // push descriptors for current frame to end of data buffer
        (dataBuffer.end() - 1)->descriptors = descriptors;

        cout << "#3 : EXTRACT DESCRIPTORS done" << endl;

        if (dataBuffer.size() > 1) // wait until at least two images have been processed
        {

            /* MATCH KEYPOINT DESCRIPTORS */

            vector<cv::DMatch> matches;
            string matcherType = "MAT_BF";        // MAT_BF, MAT_FLANN
            string descriptorType = "DES_BINARY"; // DES_BINARY, DES_HOG
            string selectorType = "SEL_KNN";       // SEL_NN, SEL_KNN

            //// STUDENT ASSIGNMENT
            //// TASK MP.5 -> add FLANN matching in file matching2D.cpp
            //// TASK MP.6 -> add KNN match selection and perform descriptor distance ratio filtering with t=0.8 in file matching2D.cpp

            matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                             (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
                             matches, descriptorType, matcherType, selectorType, totalMatch);

            //// EOF STUDENT ASSIGNMENT

            // store matches in current data frame
            (dataBuffer.end() - 1)->kptMatches = matches;

            cout << "#4 : MATCH KEYPOINT DESCRIPTORS done" << endl;

            // visualize matches between current and previous image
            bVis = true;
            if (bVis)
            {
                cv::Mat matchImg = ((dataBuffer.end() - 1)->cameraImg).clone();
                cv::drawMatches((dataBuffer.end() - 2)->cameraImg, (dataBuffer.end() - 2)->keypoints,
                                (dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->keypoints,
                                matches, matchImg,
                                cv::Scalar::all(-1), cv::Scalar::all(-1),
                                vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

                string windowName = "Matching keypoints between two camera images";
                cv::namedWindow(windowName, 7);
                cv::imshow(windowName, matchImg);
                cout << "Press key to continue to next image\n\n" << endl;
                cv::waitKey(0); // wait for key to be pressed
            }
            bVis = false;
        }

    } // eof loop over all images

    cout << "###### End of the program ######\n" << endl;
    cout << "The feature tracking, using " << detectorType << " as detector, and " << descriptorType << " as descriptor generates the following results:" << endl;
    cout << "*** The detection in all images took " << t_det << " ms." << endl;
    cout << "*** The description in all images took " << t_desc << " ms." << endl;
    cout << "*** The Detection and description in all images took " << t_det + t_desc << " ms." << endl;
    cout << "*** The total keypoints used in extraction, just in the car, in all images, was " << carKeyP << " keypoints." << endl;
    cout << "*** The total keypoints that were matched between two images was " << totalMatch << " keypoints." << endl;
    cout << "\nPlease, visit my github to see the statistics of the project: https://github.com/vini-cc/sensor_fusion--Modified/tree/master/2D_Feature_Tracking\n" << endl;
    cout << "\nThank you and see ya!\nVinicius Costa." << endl;

    return 0;
}