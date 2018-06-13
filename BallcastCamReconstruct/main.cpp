//
//  main.cpp
//  BallcastCamReconstruct
//
//  Created by Taras Vozniuk on 04/06/2018.
//  Copyright © 2018 Ballcast. All rights reserved.
//

#include <iostream>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <CoreGraphics/CoreGraphics.h>

#include "ScreenCaptureSourceWrapper.h"
#include "Semaphore.h"
#include "opecvUtils.hpp"
#include "ImageFilters.hpp"

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

int main(int argc, const char * argv[]) {
    //houghTest(argc, argv);
    
    namedWindow("Screen capture", WINDOW_NORMAL);

    ScreenCaptureSourceWrapper source = ScreenCaptureSourceWrapper();
    Semaphore* semaphor = new Semaphore();
    source.init(semaphor);
    while(source.isEnabled()){
        semaphor->wait();
        
        milliseconds start_time = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
        
        CVImageBufferRef imageBuffer = source.lastFrameBuffer();
        CVPixelBufferLockBaseAddress(imageBuffer, kCVPixelBufferLock_ReadOnly);
        
        unsigned char* buffer = static_cast<unsigned char*>(CVPixelBufferGetBaseAddress(imageBuffer));
        CGSize bufferSize = CVImageBufferGetEncodedSize(imageBuffer);
        Mat image = Mat((int)bufferSize.height, (int)bufferSize.width, CV_8UC4, buffer);

        Scalar lowerBound = Scalar(35, 30, 30, 0);
        Scalar upperBound = Scalar(60, 210, 170, 1);
        //Scalar lowerBound = Scalar(uint8(60 * 255/360), uint8(0.20 * 255), uint8(0.2 * 255));
        //Scalar upperBound = Scalar(uint8(120 * 255/360), uint8(0.80 * 255), uint8(0.75 * 255));
        
        Mat smallerImage; resize(image, smallerImage, cv::Size(), 0.5, 0.5, INTER_CUBIC);
        Mat lineMask; filteredSlowLineMask(smallerImage, lineMask, lowerBound, upperBound, 20);

        milliseconds end_time = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
        std::cout << end_time.count() - start_time.count() << std::endl;
        
        imshow("Screen capture", lineMask);
        char c = (char)waitKey(25);
        if(c == 27)
            break;
        
        CVPixelBufferUnlockBaseAddress(imageBuffer, kCVPixelBufferLock_ReadOnly);
        CVPixelBufferRelease(imageBuffer);
    }
    
    delete semaphor;
}





