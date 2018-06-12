//
//  opecvUtils.cpp
//  BallcastCamReconstruct
//
//  Created by Taras Vozniuk on 12/06/2018.
//  Copyright © 2018 Ballcast. All rights reserved.
//

#include "opecvUtils.hpp"
using namespace cv;

void shiftColumns(Mat in, Mat& out, int numRight){
    if(numRight == 0){
        in.copyTo(out);
        return;
    }
    
    int ncols = in.cols;
    int nrows = in.rows;
    out = Mat::zeros(in.size(), in.type());
    
    numRight = numRight%ncols;
    if(numRight < 0)
        numRight = ncols+numRight;
    
    in(cv::Rect(ncols-numRight,0, numRight,nrows)).copyTo(out(cv::Rect(0,0,numRight,nrows)));
    in(cv::Rect(0,0, ncols-numRight,nrows)).copyTo(out(cv::Rect(numRight,0,ncols-numRight,nrows)));
}

void shiftRows(Mat in, Mat& out, int numBottom){
    if(numBottom == 0){
        in.copyTo(out);
        return;
    }
    
    int ncols = in.cols;
    int nrows = in.rows;
    out = Mat::zeros(in.size(), in.type());
    
    numBottom = numBottom%nrows;
    if(numBottom < 0)
        numBottom = nrows+numBottom;
    
    in(cv::Rect(0,nrows-numBottom, ncols,numBottom)).copyTo(out(cv::Rect(0,0, ncols,numBottom)));
    in(cv::Rect(0,0, ncols,nrows-numBottom)).copyTo(out(cv::Rect(0, numBottom, ncols,nrows-numBottom)));
}
