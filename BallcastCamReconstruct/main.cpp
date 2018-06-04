//
//  main.cpp
//  BallcastCamReconstruct
//
//  Created by Taras Vozniuk on 04/06/2018.
//  Copyright © 2018 Ballcast. All rights reserved.
//

#include <iostream>
#include <opencv2/opencv.hpp>
using namespace cv;

int main(int argc, const char * argv[]) {
    
    Mat a(100, 100, CV_32F);
    randu(a, Scalar::all(1), Scalar::all(std::rand()));
    cv::log(a, a);
    a /= std::log(2.);
    
    return 0;
}
