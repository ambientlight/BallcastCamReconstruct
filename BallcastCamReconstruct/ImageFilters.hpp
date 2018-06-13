//
//  ImageFilters.hpp
//  BallcastCamReconstruct
//
//  Created by Taras Vozniuk on 12/06/2018.
//  Copyright © 2018 Ballcast. All rights reserved.
//

#ifndef ImageFilters_hpp
#define ImageFilters_hpp

#include <opencv2/opencv.hpp>

void filteredLineMask(cv::Mat image, cv::Mat& output, cv::Scalar lowerGreen, cv::Scalar higherGreen, uint64_t lineWidth);
void filteredSlowLineMask(cv::Mat image, cv::Mat& output, cv::Scalar lowerGreen, cv::Scalar higherGreen, uint64_t lineWidth);

#endif /* ImageFilters_hpp */
