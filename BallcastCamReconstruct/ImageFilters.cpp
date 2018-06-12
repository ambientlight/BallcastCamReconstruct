//
//  ImageFilters.cpp
//  BallcastCamReconstruct
//
//  Created by Taras Vozniuk on 12/06/2018.
//  Copyright © 2018 Ballcast. All rights reserved.
//

#include "ImageFilters.hpp"
#include "opecvUtils.hpp"
using namespace cv;

void filteredLineMask(Mat image, Mat& output, Scalar lowerGreen, Scalar higherGreen, uint64_t lineWidth){
    Mat hsvImage; cvtColor(image, hsvImage, COLOR_BGR2HSV);
    
    Mat grassMask; inRange(hsvImage, lowerGreen, higherGreen, grassMask);
    Mat grassOnlyFrameImage; bitwise_and(image, image, grassOnlyFrameImage, grassMask);
    
    //TODO: Line search space should bitwise_and with line_seach_mask
    Mat lineSearchSpace = image;
    
    Mat blueComponent; extractChannel(lineSearchSpace, blueComponent, 2);
    
    Mat halfLineWidthShiftedDown; shiftRows(blueComponent, halfLineWidthShiftedDown, round(lineWidth / 2));
    Mat halfLineWidthShiftedUp; shiftRows(blueComponent, halfLineWidthShiftedUp, -round(lineWidth / 2));
    
    Mat filterDiffDown; subtract(blueComponent, halfLineWidthShiftedDown, filterDiffDown);
    Mat filterDiffUp; subtract(blueComponent, halfLineWidthShiftedUp, filterDiffUp);
    
    Mat nonGrassArea; bitwise_not(grassMask, nonGrassArea);
    Mat diff = min(filterDiffDown, filterDiffUp);
    Mat target; bitwise_and(diff, diff, target, nonGrassArea);
    // turn into a mask
    Mat targetMask; inRange(target, Scalar(1), Scalar(255), targetMask);
    output = targetMask;
}
