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

using namespace cv;
using namespace std::chrono;

typedef void* CGAccessSessionRef;

static void
createTrigTable( int numangle, double min_theta, double theta_step,
                float irho, float *tabSin, float *tabCos )
{
    float ang = static_cast<float>(min_theta);
    for(int n = 0; n < numangle; ang += (float)theta_step, n++ )
    {
        tabSin[n] = (float)(sin((double)ang) * irho);
        tabCos[n] = (float)(cos((double)ang) * irho);
    }
}

static Mat
HoughLinesStandard(InputArray src,
                   float rho, float theta,
                   int threshold,
                   double min_theta, double max_theta)
{
    Mat img = src.getMat();
    int i, j;
    float irho = 1 / rho;

    CV_Assert( img.type() == CV_8UC1 );
    
    const uchar* image = img.ptr();
    int step = (int)img.step;
    int width = img.cols;
    int height = img.rows;
    
    int max_rho = width + height;
    int min_rho = -max_rho;
    
    int numangle = cvRound((max_theta - min_theta) / theta);
    int numrho = cvRound(((max_rho - min_rho) + 1) / rho);
    
    Mat _accum = Mat::zeros( (numangle+2), (numrho+2), CV_32SC1 );
    std::vector<int> _sort_buf;
    AutoBuffer<float> _tabSin(numangle);
    AutoBuffer<float> _tabCos(numangle);
    int *accum = _accum.ptr<int>();
    float *tabSin = _tabSin, *tabCos = _tabCos;
    
    // create sin and cos table
    createTrigTable( numangle, min_theta, theta,
                    irho, tabSin, tabCos );
    
    // stage 1. fill accumulator
    for( i = 0; i < height; i++ ){
        for( j = 0; j < width; j++ ){
            if( image[i * step + j] != 0 ){
                for(int n = 0; n < numangle; n++ ){
                    int r = cvRound( j * tabCos[n] + i * tabSin[n] );
                    r += (numrho - 1) / 2;
                    accum[(n+1) * (numrho+2) + r+1]++;
                }
            }
        }
    }
    
    return _accum;
}

void houghTest(Mat image){
    std::cout << image.size() << ", channels: " << image.channels() << std::endl;
    
    Mat edges;
    Canny(image, edges, 100.0, 200.0);
    
    // Measure the time naive hough transform performs
    milliseconds start_time = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
    Mat accum = HoughLinesStandard(edges, 1.0, CV_PI/180, 30, 0, CV_PI);
    milliseconds end_time = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
    std::cout << end_time.count() - start_time.count() << std::endl;
    //std::cout << accum;
    
    // Create a window for display
    // Show our image inside it.
    // And wait for keystoke in the window
    // namedWindow("Display window", WINDOW_AUTOSIZE);
    // imshow("Display window", edges);
    // waitKey(0);
}

/*
void screenshotAndDisplayInOpenCV() {
    milliseconds start_time = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
    CGImageRef imageRef = CGWindowListCreateImage(CGRectInfinite, kCGWindowListOptionAll, kCGNullWindowID, kCGWindowImageDefault);
    CFDataRef rawData = CGDataProviderCopyData(CGImageGetDataProvider(imageRef));
    uint8_t* buffer = (uint8_t*)CFDataGetBytePtr(rawData);
    
    // CGAccessSessionRef imageDataSession = CGAccessSessionCreate(CGImageGetDataProvider(imageRef));
    // uint8_t* buffer = (uint8_t*)CGAccessSessionGetBytePointer(imageDataSession);
    
    std::cout << (int)CGImageGetHeight(imageRef) << std::endl;
    std::cout << (int)CGImageGetWidth(imageRef) << std::endl;
    
    Mat image = Mat((int)CGImageGetHeight(imageRef), (int)CGImageGetWidth(imageRef), CV_8UC4, buffer);
    CGImageRelease(imageRef);
    
    milliseconds end_time = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
    std::cout << end_time.count() - start_time.count() << std::endl;
    
    // Create a window for display
    // Show our image inside it.
    // And wait for keystoke in the window
    namedWindow("Display window", WINDOW_AUTOSIZE);
    imshow("Display window", image);
    waitKey(0);
}
*/

void coreTransform(Mat image, Mat& mask, Mat& output, Scalar lowerBound, Scalar upperBound){
    Mat smallerImage; resize(image, smallerImage, cv::Size(), 0.5, 0.5, INTER_CUBIC);
    
    
    //GaussianBlur(smallerImage, smallerImage, Size(3, 3), 0);
    Mat kern = (Mat_<char>(3, 3) <<
                -1, -1, -1,
                -1, 9, -1,
                -1, -1, -1);
    filter2D(smallerImage, smallerImage, smallerImage.depth(), kern);
    
    
    Mat lineMask; filteredSlowLineMask(smallerImage, lineMask, lowerBound, upperBound, 14);
    
    
    Mat ellipseDetectInput; cvtColor(smallerImage, ellipseDetectInput, CV_BGRA2GRAY);
    Size sz = smallerImage.size();
    
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
    yaed.Detect(ellipseDetectInput, detectedEllipses);
    yaed.DrawDetectedEllipses((Mat3b&)smallerImage, detectedEllipses, 1);
    
    //Mat kernel = getStructuringElement(MORPH_RECT, cv::Size(2, 2));
    //dilate(lineMask, lineMask, kernel);
    /*
    std::vector<std::vector<cv::Point>> contours;
    std::vector<Vec4i> hierarchy;
    findContours(lineMask, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
    std::cout << "Contours: " << contours.size() << std::endl;
    
    for (int i = 0; i< contours.size(); i++){
        //if(contourArea(contours[i], false) > 0){
            drawContours(smallerImage, contours,
                             i, Scalar(0, 255, 0),
                             1, LINE_8,
                            hierarchy, 0, cv::Point());
        //}
        //std::cout << "Contour(" << i << "): " << contourArea(contours[i], false) << std::endl;
    }
    */
    
    std::vector<Vec4f> houghLines; HoughLinesP(lineMask, houghLines, 1, M_PI/180,
                                               100, 50, 10);
    for (const Vec4f& detectedLine: houghLines){
        line(smallerImage,
             cv::Point(detectedLine[0], detectedLine[1]),
             cv::Point(detectedLine[2], detectedLine[3]), Scalar(255, 0, 0), 2);
    }
    
    mask = lineMask;
    output = smallerImage;
}

void performTransformFromScreenCapture(Scalar lowerBound, Scalar upperBound){
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
        Mat lineMask; Mat smallerImage; coreTransform(image, lineMask, smallerImage, lowerBound, upperBound);
        
        milliseconds end_time = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
        std::cout << end_time.count() - start_time.count() << std::endl;
        
        imshow("Line Mask", lineMask);
        imshow("Detected Lines", smallerImage);
        char c = (char)waitKey(1);
        if(c == 27)
            break;
        
        source.unlockAndRelease(imageBuffer);
    }
    
    delete semaphor;
}

void performTransformFromLoadedImage(const String& filename, Scalar lowerBound, Scalar upperBound){
    Mat image = imread(filename);
    Mat lineMask; Mat smallerImage; coreTransform(image, lineMask, smallerImage, lowerBound, upperBound);
    imshow("Line Mask", lineMask);
    imshow("Detected Lines", smallerImage);
    waitKey();
}

int main(int argc, const char * argv[]) {
    namedWindow("Line Mask", WINDOW_NORMAL);
    namedWindow("Detected Lines", WINDOW_NORMAL);
    
    if(argc < 2){
        std::cout << "Expected file name passed" << std::endl;
    }
    
    Scalar lowerBound = Scalar((50 * 180/360) - 1, (0.45 * 256) - 1, (0.15 * 256) - 1, 0);
    Scalar upperBound = Scalar((150 * 180/360) - 1, (1 * 256) - 1, (1 * 256) - 1, 1);
    if(true){
        performTransformFromScreenCapture(lowerBound, upperBound);
    } else {
        performTransformFromLoadedImage(argv[1], lowerBound, upperBound);
    }
}





