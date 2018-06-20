//
//  main.cpp
//  BallcastCamReconstruct
//
//  Created by Taras Vozniuk on 04/06/2018.
//  Copyright © 2018 Ballcast. All rights reserved.
//

#include <iostream>
#include <chrono>
#include "ScreenCaptureSourceWrapper.h"

#include "Semaphore.h"
#include "opecvUtils.hpp"
#include "ImageFilters.hpp"
#include "EllipseDetectorYaed.h"

#ifndef ENABLE_LSD_TRANSFORM
#define ENABLE_LSD_TRANSFORM true
#endif

#ifndef USING_SCREEN_CAPTURE
#define USING_SCREEN_CAPTURE false
#endif

#ifndef CUSTOM_EXPORT_SESSION
#define CUSTOM_EXPORT_SESSION false
#endif

using namespace cv;
using namespace std::chrono;

bool lineEquivalence(const Vec4i& _l1, const Vec4i& _l2)
{
    Vec4i l1(_l1), l2(_l2);
    
    float length1 = sqrtf((l1[2] - l1[0])*(l1[2] - l1[0]) + (l1[3] - l1[1])*(l1[3] - l1[1]));
    float length2 = sqrtf((l2[2] - l2[0])*(l2[2] - l2[0]) + (l2[3] - l2[1])*(l2[3] - l2[1]));
    
    float product = (l1[2] - l1[0])*(l2[2] - l2[0]) + (l1[3] - l1[1])*(l2[3] - l2[1]);
    
    if (fabs(product / (length1 * length2)) < cos(CV_PI / 30))
        return false;
    
    float mx1 = (l1[0] + l1[2]) * 0.5f;
    float mx2 = (l2[0] + l2[2]) * 0.5f;
    
    float my1 = (l1[1] + l1[3]) * 0.5f;
    float my2 = (l2[1] + l2[3]) * 0.5f;
    float dist = sqrtf((mx1 - mx2)*(mx1 - mx2) + (my1 - my2)*(my1 - my2));
    
    if (dist > std::max(length1, length2) * 0.5f)
        return false;
    
    return true;
}

void lineSegmentDetectorTransform(Mat image, Mat& mask, Mat& output, Scalar lowerBound, Scalar upperBound){
    Mat target = image.clone();
    Mat bgrImage; cvtColor(image, bgrImage, COLOR_BGRA2BGR);
    Mat hsvImage; cvtColor(bgrImage, hsvImage, COLOR_BGR2HSV);
    Mat grassMask; inRange(hsvImage, lowerBound, upperBound, grassMask);
    Mat grassOnlyFrameImage; bitwise_and(image, image, grassOnlyFrameImage, grassMask);
    
    Mat grayscale; cvtColor(grassOnlyFrameImage, grayscale, CV_BGRA2GRAY);

    Ptr<LineSegmentDetector> detector = createLineSegmentDetector(LSD_REFINE_STD);
    std::vector<Vec4f> lines; detector->detect(grayscale, lines);
    std::vector<Vec4f> linesWithoutSmall;
    std::copy_if (lines.begin(), lines.end(), std::back_inserter(linesWithoutSmall), [](Vec4f line){
        float length = sqrtf((line[2] - line[0]) * (line[2] - line[0])
                           + (line[3] - line[1]) * (line[3] - line[1]));
        return length > 30;
    });
    
    std::cout << "Detected " << lines.size() << " lines" << std::endl;
    std::cout << "Without small " << linesWithoutSmall.size() << " lines" << std::endl;
    
    std::vector<int> labels;
    int equilavenceClassesCount = cv::partition(linesWithoutSmall, labels, lineEquivalence);
    std::cout << "Equivalence classes: " << equilavenceClassesCount << std::endl;
    
    RNG rng(215526);
    std::vector<Scalar> colors(equilavenceClassesCount);
    for (int i = 0; i < equilavenceClassesCount; i++){
        colors[i] = Scalar(rng.uniform(30,255), rng.uniform(30, 255), rng.uniform(30, 255));;
    }
    
    Mat clearTarget = Mat::zeros(image.rows, image.cols, CV_8UC3);
    for (int i = 0; i < linesWithoutSmall.size(); i++){
        Vec4f& detectedLine = linesWithoutSmall[i];
        line(clearTarget,
             cv::Point(detectedLine[0], detectedLine[1]),
             cv::Point(detectedLine[2], detectedLine[3]), colors[labels[i]], 2);
    }
    
    mask = grassOnlyFrameImage;
    output = clearTarget;
}

void lineFilterHoughPFornaciariEllipse(Mat image, Mat& mask, Mat& output, Scalar lowerBound, Scalar upperBound, bool sharpen = false, bool dilate = false){
    Mat target = image.clone();
    
    // add sharpening to enhance lines
    if(sharpen){
        Mat kern = (Mat_<char>(3, 3) <<
                -1, -1, -1,
                -1, 9, -1,
                -1, -1, -1);
        filter2D(target, target, target.depth(), kern);
    }
    
    // perform core line detection that involves filtering out non-grass and simple custom min(half-line diff)
    Mat lineMask; filteredSlowLineMask(target, lineMask, lowerBound, upperBound,
                                       14,  // half line width
                                       50); // line filter threshold
    
    // grab an original grayscale image for ellipse detection
    Mat grayscale; cvtColor(image, grayscale, CV_BGRA2GRAY);
    Size sz = grayscale.size();
    
    // Parameters Settings (Sect. 4.2)
    int    iThLength = 16;
    float  fThObb = 3.0f;
    float  fThPos = 1.0f;
    float  fTaoCenters = 0.05f;
    int    iNs = 16;
    float  fMaxCenterDistance = sqrt(float(sz.width*sz.width + sz.height*sz.height)) * fTaoCenters;
    float  fThScoreScore = 0.7f;
    // Gaussian filter parameters, in pre-processing
    Size   szPreProcessingGaussKernelSize = Size(5, 5);
    double dPreProcessingGaussSigma = 1.0;
    
    float  fDistanceToEllipseContour = 0.1f;    // (Sect. 3.3.1 - Validation)
    float  fMinReliability = 0.5;    // Const parameters to discard bad ellipses
    
    CEllipseDetectorYaed yaed;
    yaed.SetParameters(szPreProcessingGaussKernelSize,
                       dPreProcessingGaussSigma,
                       fThPos,
                       fMaxCenterDistance,
                       iThLength,
                       fThObb,
                       fDistanceToEllipseContour,
                       fThScoreScore,
                       fMinReliability,
                       iNs);
    
    vector<Ellipse> detectedEllipses;
    yaed.Detect(grayscale, detectedEllipses);
    yaed.DrawDetectedEllipses((Mat3b&)target, detectedEllipses, 1);
    
    // dilate to help connecting weak lines together
    if(dilate){
        Mat kernel = getStructuringElement(MORPH_RECT, cv::Size(3, 3));
        cv::dilate(lineMask, lineMask, kernel);
    }
    
    // probabilistic hough lines transform: fast but wobbling
    std::vector<Vec4f> houghLines; HoughLinesP(lineMask, houghLines, 1, M_PI/180,
                                               100, 50, 10);
    for (const Vec4f& detectedLine: houghLines){
        line(target,
             cv::Point(detectedLine[0], detectedLine[1]),
             cv::Point(detectedLine[2], detectedLine[3]), Scalar(255, 0, 0), 2);
    }
    
    mask = lineMask;
    output = target;
}

void performTransformFromScreenCapture(){
    ScreenCaptureSourceWrapper source = ScreenCaptureSourceWrapper();
    Semaphore* semaphor = new Semaphore();
    source.init(semaphor);
    
    while(source.isEnabled()){
        semaphor->wait();
        
        milliseconds start_time = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
        
        void* imageBuffer = source.lastFrameBuffer();
        source.lockBaseAddress(imageBuffer);
        
        unsigned char* buffer = source.getBaseAddress(imageBuffer);
        Mat image = Mat((int)source.bufferHeight(imageBuffer), (int)source.bufferWidth(imageBuffer), CV_8UC4, buffer);
        Mat smallerImage; resize(image, smallerImage, cv::Size(), 0.5, 0.5, INTER_CUBIC);
        
        #if ENABLE_LSD_TRANSFORM
            Scalar lowerBound = Scalar((75 * 180/360) - 1, (0 * 256) - 1, (0 * 256) - 1, 0);
            Scalar upperBound = Scalar((150 * 180/360) - 1, (1 * 256) - 1, (1 * 256) - 1, 1);
            Mat lineMask; lineSegmentDetectorTransform(smallerImage, lineMask, smallerImage, lowerBound, upperBound);
        #else
            Scalar lowerBound = Scalar((50 * 180/360) - 1, (0.45 * 256) - 1, (0.15 * 256) - 1, 0);
            Scalar upperBound = Scalar((150 * 180/360) - 1, (1 * 256) - 1, (1 * 256) - 1, 1);
            Mat lineMask; lineFilterHoughPFornaciariEllipse(smallerImage, lineMask, smallerImage, lowerBound, upperBound);
        #endif
        
        milliseconds end_time = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
        std::cout << end_time.count() - start_time.count() << std::endl;
        
        //imshow("Line Mask", lineMask);
        imshow("Detected Lines", smallerImage);
        char c = (char)waitKey(1);
        if(c == 27)
            break;
        
        source.unlockAndRelease(imageBuffer);
    }
    
    delete semaphor;
}

void performTransformFromLoadedImage(const String& filename){
    Mat image = imread(filename);
    Mat smallerImage; resize(image, smallerImage, cv::Size(), 0.5, 0.5, INTER_CUBIC);
    
    #if ENABLE_LSD_TRANSFORM
        Scalar lowerBound = Scalar((75 * 180/360) - 1, (0 * 256) - 1, (0 * 256) - 1, 0);
        Scalar upperBound = Scalar((150 * 180/360) - 1, (1 * 256) - 1, (1 * 256) - 1, 1);
        Mat lineMask; lineSegmentDetectorTransform(smallerImage, lineMask, smallerImage, lowerBound, upperBound);
    #else
        Scalar lowerBound = Scalar((50 * 180/360) - 1, (0.45 * 256) - 1, (0.15 * 256) - 1, 0);
        Scalar upperBound = Scalar((150 * 180/360) - 1, (1 * 256) - 1, (1 * 256) - 1, 1);
        Mat lineMask; lineFilterHoughPFornaciariEllipse(smallerImage, lineMask, smallerImage, lowerBound, upperBound);
    #endif
    
    //imshow("Line Mask", lineMask);
    imshow("Detected Lines", smallerImage);
    waitKey();
}

void performCustomExportSession(const String& folderPath){
    vector<String> matchedFilenames; glob(folderPath + "/*.png", matchedFilenames);
}

int main(int argc, const char * argv[]) {
    namedWindow("Line Mask", WINDOW_NORMAL);
    namedWindow("Detected Lines", WINDOW_NORMAL);
    
    if(argc < 3){
        std::cout << "Expected file name and frames folder passed" << std::endl;
    }

    #if CUSTOM_EXPORT_SESSION
        performCustomExportSession(argv[2]);
    #elif USING_SCREEN_CAPTURE
        performTransformFromScreenCapture();
    #else
        performTransformFromLoadedImage(argv[1]);
    #endif
}





