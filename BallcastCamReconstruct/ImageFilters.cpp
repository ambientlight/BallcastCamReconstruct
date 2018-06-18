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
    //Mat nonGrassMask; bitwise_not(grassMask, nonGrassMask);

    Mat grassOnlyFrameImage; bitwise_and(image, image, grassOnlyFrameImage, grassMask);
    output = grassOnlyFrameImage;
    
    //TODO: Line search space should bitwise_and with line_seach_mask
    Mat lineSearchSpace = image;

    Mat blueComponent; extractChannel(lineSearchSpace, blueComponent, 0);
    int halfLineWidth = (int)lineWidth / 2;
    
    blueComponent.forEach<uint8_t>([&](uint8_t& b, const int* position) -> void {
        int rowIndex = position[0];
        int columnIndex = position[1];
        bool skipRows = false;
        bool skipCols = false;

        if(rowIndex <= halfLineWidth || rowIndex >= blueComponent.rows - halfLineWidth){ skipRows = true; }
        if(!skipRows && (grassMask.at<uint8_t>(rowIndex - halfLineWidth, columnIndex) == 0 || grassMask.at<uint8_t>(rowIndex + halfLineWidth, columnIndex) == 0)){
            skipRows = true;
        }

        if(columnIndex <= halfLineWidth || columnIndex >= blueComponent.cols - halfLineWidth){ skipCols = true; }
        if(!skipCols && (grassMask.at<uint8_t>(rowIndex, columnIndex - halfLineWidth) == 0 || grassMask.at<uint8_t>(rowIndex, columnIndex + halfLineWidth) == 0)){
            skipCols = true;
        }

        if(!skipRows && !skipCols){
            int a = (int)blueComponent.at<uint8_t>(rowIndex - halfLineWidth, columnIndex);
            int c = (int)blueComponent.at<uint8_t>(rowIndex + halfLineWidth, columnIndex);
            int d = (int)blueComponent.at<uint8_t>(rowIndex, columnIndex - halfLineWidth);
            int e = (int)blueComponent.at<uint8_t>(rowIndex, columnIndex + halfLineWidth);
            int o = (int)b;
            b = (uint8_t)abs(min(min(o - a, o - c), min(o - d, o - e)));
        } else if (!skipRows){
            int a = (int)blueComponent.at<uint8_t>(rowIndex - halfLineWidth, columnIndex);
            int c = (int)blueComponent.at<uint8_t>(rowIndex + halfLineWidth, columnIndex);
            int o = (int)b;
            b = (uint8_t)abs(min(o - a, o - c));
        } else if (!skipCols){
            int d = (int)blueComponent.at<uint8_t>(rowIndex, columnIndex - halfLineWidth);
            int e = (int)blueComponent.at<uint8_t>(rowIndex, columnIndex + halfLineWidth);
            int o = (int)b;
            b = (uint8_t)abs(min(o - d, o - e));
        } else {
            b = 0;
        }
        
        b = b > 50 ? b : 0;
    });
    
    
    //Mat target = Mat::zeros(lineSearchSpace.rows, lineSearchSpace.cols, CV_8UC4);
    //insertChannel(blueComponent, target, 0);
    
    // turn into a mask
    //Mat target; bitwise_and(blueComponent, blueComponent, target, nonGrassMask);
    Mat targetMask; inRange(blueComponent, Scalar(1), Scalar(255), targetMask);
    output = blueComponent;
}


