//
//  opecvUtils.hpp
//  BallcastCamReconstruct
//
//  Created by Taras Vozniuk on 12/06/2018.
//  Copyright © 2018 Ballcast. All rights reserved.
//

#ifndef opecvUtils_hpp
#define opecvUtils_hpp

#include <opencv2/opencv.hpp>

void shiftColumns(cv::Mat in, cv::Mat& out, int numRight);
void shiftRows(cv::Mat in, cv::Mat& out, int numBottom);

#endif /* opecvUtils_hpp */
