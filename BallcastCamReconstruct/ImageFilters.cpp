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
    Mat halfLineWidthShiftedRight; shiftColumns(blueComponent, halfLineWidthShiftedRight, round(lineWidth / 2));
    Mat halfLineWidthShiftedLeft; shiftColumns(blueComponent, halfLineWidthShiftedLeft, -round(lineWidth / 2));
    
    Mat filterDiffDown; subtract(blueComponent, halfLineWidthShiftedDown, filterDiffDown);
    Mat filterDiffUp; subtract(blueComponent, halfLineWidthShiftedUp, filterDiffUp);
    Mat filterDiffRight; subtract(blueComponent, halfLineWidthShiftedRight, filterDiffRight);
    Mat filterDiffLeft; subtract(blueComponent, halfLineWidthShiftedLeft, filterDiffLeft);
    
    Mat nonGrassArea; bitwise_not(grassMask, nonGrassArea);
    Mat diffVert = min(filterDiffDown, filterDiffUp);
    Mat diffHor = min(filterDiffRight, filterDiffLeft);
    Mat diff = min(diffHor, diffVert);
    Mat target; bitwise_and(diff, diff, target, nonGrassArea);
    // turn into a mask
    Mat targetMask; inRange(target, Scalar(1), Scalar(255), targetMask);
    output = targetMask;
}

void filteredSlowLineMask(Mat image, Mat& output, Scalar lowerGreen, Scalar higherGreen, uint64_t lineWidth){
    Mat hsvImage; cvtColor(image, hsvImage, COLOR_BGR2HSV);
    
    Mat grassMask; inRange(hsvImage, lowerGreen, higherGreen, grassMask);
    Mat nonGrassMask; bitwise_not(grassMask, nonGrassMask);

    Mat grassOnlyFrameImage; bitwise_and(image, image, grassOnlyFrameImage, grassMask);

    //TODO: Line search space should bitwise_and with line_seach_mask
    Mat lineSearchSpace = image;

    Mat blueComponent; extractChannel(lineSearchSpace, blueComponent, 2);
    int halfLineWidth = round(lineWidth / 2);
    blueComponent.forEach<uint8_t>([&](uint8_t& b, const int* position) -> void {
        int rowIndex = position[0];
        int columnIndex = position[1];
        bool skipRows = false;
        bool skipCols = false;

        if(rowIndex <= halfLineWidth || rowIndex >= blueComponent.rows - halfLineWidth){ skipRows = true; }
        if(!skipRows &&(  grassMask.at<uint8_t>(rowIndex - halfLineWidth, columnIndex) == 0
           || grassMask.at<uint8_t>(rowIndex + halfLineWidth, columnIndex) == 0)){
            skipRows = true;
        }

        if(columnIndex <= halfLineWidth || columnIndex >= blueComponent.cols - halfLineWidth){ skipCols = true; }
        if(!skipCols && (grassMask.at<uint8_t>(rowIndex, columnIndex - halfLineWidth) == 0
           || grassMask.at<uint8_t>(rowIndex, columnIndex + halfLineWidth) == 0)){
            skipCols = true;
        }

        if(!skipRows && !skipCols){
            uint8_t a = blueComponent.at<uint8_t>(rowIndex - halfLineWidth, columnIndex);
            uint8_t c = blueComponent.at<uint8_t>(rowIndex + halfLineWidth, columnIndex);
            uint8_t d = blueComponent.at<uint8_t>(rowIndex, columnIndex - halfLineWidth);
            uint8_t e = blueComponent.at<uint8_t>(rowIndex, columnIndex + halfLineWidth);
            b = min(min(b - a, b - c), min(b - d, b - e));
        } else if (!skipRows){
            uint8_t a = blueComponent.at<uint8_t>(rowIndex - halfLineWidth, columnIndex);
            uint8_t c = blueComponent.at<uint8_t>(rowIndex + halfLineWidth, columnIndex);
            b = min(b - a, b - c);
        } else if (!skipCols){
            uint8_t d = blueComponent.at<uint8_t>(rowIndex, columnIndex - halfLineWidth);
            uint8_t e = blueComponent.at<uint8_t>(rowIndex, columnIndex + halfLineWidth);
            b = min(b - d, b - e);
        } else {
            b = 0;
        }
    });

    // turn into a mask
    Mat target; bitwise_and(blueComponent, blueComponent, target, nonGrassMask);
    Mat targetMask; inRange(target, Scalar(1), Scalar(255), targetMask);
    output = targetMask;
}


