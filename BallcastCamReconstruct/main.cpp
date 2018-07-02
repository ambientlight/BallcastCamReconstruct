//
//  main.cpp
//  BallcastCamReconstruct
//
//  Created by Taras Vozniuk on 04/06/2018.
//  Copyright © 2018 Ballcast. All rights reserved.
//

#include <iostream>
#include <chrono>
#include <algorithm>
#include <sys/types.h>
#include <sys/stat.h>

#include <nlohmann/json.hpp>

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
#define CUSTOM_EXPORT_SESSION true
#endif

using namespace cv;
using namespace std::chrono;
using json = nlohmann::json;

Vec2d linearParameters(Vec4i line){
    Mat a = (Mat_<double>(2, 2) <<
                line[0], 1,
                line[2], 1);
    Mat y = (Mat_<double>(2, 1) <<
                line[1],
                line[3]);
    Vec2d mc; solve(a, y, mc);
    return mc;
}

Vec4i extendedLine(Vec4i line, double d){
    // oriented left-t-right
    Vec4d _line = line[2] - line[0] < 0 ? Vec4d(line[2], line[3], line[0], line[1]) : Vec4d(line[0], line[1], line[2], line[3]);
    double m = linearParameters(_line)[0];
    // solution of pythagorean theorem and m = yd/xd
    double xd = sqrt(d * d / (m * m + 1));
    double yd = xd * m;
    return Vec4d(_line[0] - xd, _line[1] - yd , _line[2] + xd, _line[3] + yd);
}

std::vector<Point2i> boundingRectangleContour(Vec4i line, float d){
    // finds coordinates of perpendicular lines with length d in both line points
    // https://math.stackexchange.com/a/2043065/183923
    
    Vec2f mc = linearParameters(line);
    float m = mc[0];
    float factor = sqrtf(
        (d * d) / (1 + (1 / (m * m)))
    );
    
    float x3, y3, x4, y4, x5, y5, x6, y6;
    // special case(vertical perpendicular line) when -1/m -> -infinity
    if(m == 0){
        x3 = line[0]; y3 = line[1] + d;
        x4 = line[0]; y4 = line[1] - d;
        x5 = line[2]; y5 = line[3] + d;
        x6 = line[2]; y6 = line[3] - d;
    } else {
        // slope of perpendicular lines
        float m_per = - 1/m;
        
        // y1 = m_per * x1 + b
        float c_per1 = line[1] - m_per * line[0];
        float c_per2 = line[3] - m_per * line[2];
        
        // coordinates of perpendicular lines
        x3 = line[0] + factor; y3 = m_per * x3 + c_per1;
        x4 = line[0] - factor; y4 = m_per * x4 + c_per1;
        x5 = line[2] + factor; y5 = m_per * x5 + c_per2;
        x6 = line[2] - factor; y6 = m_per * x6 + c_per2;
    }

    return std::vector<Point2i> {
        Point2i(x3, y3),
        Point2i(x4, y4),
        Point2i(x6, y6),
        Point2i(x5, y5)
    };
}


bool extendedBoundingRectangleLineEquivalence(const Vec4i& _l1, const Vec4i& _l2, float extensionLengthFraction, float maxAngleDiff, float boundingRectangleThickness){
    
    Vec4i l1(_l1), l2(_l2);
    // extend lines by percentage of line width
    float len1 = sqrtf((l1[2] - l1[0])*(l1[2] - l1[0]) + (l1[3] - l1[1])*(l1[3] - l1[1]));
    float len2 = sqrtf((l2[2] - l2[0])*(l2[2] - l2[0]) + (l2[3] - l2[1])*(l2[3] - l2[1]));
    Vec4i el1 = extendedLine(l1, len1 * extensionLengthFraction);
    Vec4i el2 = extendedLine(l2, len2 * extensionLengthFraction);
    
    // reject the lines that have wide difference in angles
    float a1 = atan(linearParameters(el1)[0]);
    float a2 = atan(linearParameters(el2)[0]);
    if(fabs(a1 - a2) > maxAngleDiff * M_PI / 180.0){
        return false;
    }
    
    // calculate window around extended line
    // at least one point needs to inside extended bounding rectangle of other line,
    std::vector<Point2i> lineBoundingContour = boundingRectangleContour(el1, boundingRectangleThickness/2);
    return
        pointPolygonTest(lineBoundingContour, cv::Point(el2[0], el2[1]), false) == 1 ||
        pointPolygonTest(lineBoundingContour, cv::Point(el2[2], el2[3]), false) == 1;
}

std::vector<Vec4i> lineSegmentDetectorTransform(Mat image, Mat& mask, Mat& output, Scalar lowerBound, Scalar upperBound){
    Mat target = image.clone();
    Mat bgrImage; cvtColor(image, bgrImage, COLOR_BGRA2BGR);
    Mat hsvImage; cvtColor(bgrImage, hsvImage, COLOR_BGR2HSV);
    Mat grassMask; inRange(hsvImage, lowerBound, upperBound, grassMask);
    Mat grassOnlyFrameImage; bitwise_and(image, image, grassOnlyFrameImage, grassMask);
    
    Mat grayscale; cvtColor(grassOnlyFrameImage, grayscale, CV_BGRA2GRAY);

    Ptr<LineSegmentDetector> detector = createLineSegmentDetector(LSD_REFINE_STD);
    std::vector<Vec4i> lines; detector->detect(grayscale, lines);
    std::vector<Vec4i> linesWithoutSmall;
    std::copy_if (lines.begin(), lines.end(), std::back_inserter(linesWithoutSmall), [](Vec4f line){
        float length = sqrtf((line[2] - line[0]) * (line[2] - line[0])
                           + (line[3] - line[1]) * (line[3] - line[1]));
        return length > 30;
    });
    
    std::vector<int> labels;
    int equilavenceClassesCount = cv::partition(linesWithoutSmall, labels, [](const Vec4i l1, const Vec4i l2){
        return extendedBoundingRectangleLineEquivalence(l1, l2, 0.5, 4.0, 10);
    });
    
    Mat clearTarget = Mat::zeros(image.rows, image.cols, CV_8UC3);
    // point cloud out of each equivalence classes
    std::vector<std::vector<Point2i>> pointClouds(equilavenceClassesCount);
    for (int i = 0; i < linesWithoutSmall.size(); i++){
        Vec4i& detectedLine = linesWithoutSmall[i];
        pointClouds[labels[i]].push_back(Point2i(detectedLine[0], detectedLine[1]));
        pointClouds[labels[i]].push_back(Point2i(detectedLine[2], detectedLine[3]));
    }
    
    // fit line to each equivalence class point cloud
    std::vector<Vec4i> reducedLines = std::accumulate(pointClouds.begin(), pointClouds.end(), std::vector<Vec4i>{}, [&](std::vector<Vec4i> target, const std::vector<Point2i>& _pointCloud){
        std::vector<Point2i> pointCloud = _pointCloud;
        
        //lineParams: [vx,vy, x0,y0]: (normalized vector, point on our contour)
        // (x,y) = (x0,y0) + t*(vx,vy), t -> (-inf; inf)
        Vec4f lineParams; fitLine(pointCloud, lineParams, CV_DIST_L2, 0, 0.01, 0.01);
        
        // derive the bounding xs of point cloud
        decltype(pointCloud)::iterator minXP, maxXP;
        std::tie(minXP, maxXP) = std::minmax_element(pointCloud.begin(), pointCloud.end(), [](const Point2i& p1, const Point2i& p2){ return p1.x < p2.x; });
        
        // derive y coords of fitted line
        float m = lineParams[1] / lineParams[0];
        int y1 = ((minXP->x - lineParams[2]) * m) + lineParams[3];
        int y2 = ((maxXP->x - lineParams[2]) * m) + lineParams[3];
        
        target.push_back(Vec4i(minXP->x, y1, maxXP->x, y2));
        return target;
    });
    
    for(Vec4i reduced: reducedLines){
        line(clearTarget, Point(reduced[0], reduced[1]), Point(reduced[2], reduced[3]), Scalar(255, 255, 255), 1);
    }
    
    mask = grassOnlyFrameImage;
    output = clearTarget;
    return reducedLines;
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

void performCustomExportSession(const std::string& outputFolderPath){
    //vector<String> matchedFilenames; glob(folderPath + "/*.png", matchedFilenames);
    //std::copy(matchedFilenames.begin(), matchedFilenames.end(), std::ostream_iterator<String>(cout, "\n"));
    
    std::ifstream jsonInput("frame_data.json");
    json frameData; jsonInput >> frameData;

    milliseconds start_time = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
    std::cout << "Frames(all): " << frameData.size() << std::endl;
    std::vector<json> regularFrames; std::copy_if(frameData.begin(), frameData.end(), std::back_inserter(regularFrames), [](const json& element){
        return element["type"] == "regular";
    });
    std::cout << "Frames(regular): " << regularFrames.size() << std::endl;
    std::vector<json> extendedFrames; std::transform(regularFrames.begin(), regularFrames.end(), std::back_inserter(extendedFrames), [&](const json& regularFrame){
        json frame = regularFrame;
        
        std::string path = regularFrame["imagePath"];
        Mat image = imread(path);

        Scalar lowerBound = Scalar((75 * 180/360) - 1, (0 * 256) - 1, (0 * 256) - 1, 0);
        Scalar upperBound = Scalar((150 * 180/360) - 1, (1 * 256) - 1, (1 * 256) - 1, 1);
        Mat lineMask; Mat linesImage; std::vector<Vec4i> lines = lineSegmentDetectorTransform(image, lineMask, linesImage, lowerBound, upperBound);
        std::vector<std::vector<int>> linesCompat; std::transform(lines.begin(), lines.end(), std::back_inserter(linesCompat), [](const Vec4i& line){
            return std::vector<int>{ line[0], line[1], line[2], line[3] };
        });

        frame["detectedLines"] = linesCompat;
        
        std::stringstream sOutputImagePath; sOutputImagePath << outputFolderPath << "/" << frame["progress"] << ".png";
        frame["detectedLinesImagePath"] = sOutputImagePath.str();
        
        struct stat pathInfo;
        bool lineOutputDirExists = stat(outputFolderPath.c_str(), &pathInfo) == 0;
        if(!lineOutputDirExists){
            mkdir(outputFolderPath.c_str(), 0666);
        }
        
        imwrite(sOutputImagePath.str(), linesImage);
        return frame;
    });
    
    milliseconds end_time = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
    std::cout << end_time.count() - start_time.count() << std::endl;
    
    std::ofstream jsonOutput("frame_data_proc.json");
    jsonOutput << std::setw(4) << (json)extendedFrames << std::endl;
    //std::copy(extendedFrames.begin(), extendedFrames.end(), std::ostream_iterator<json>(cout, "\n"));
}

int main(int argc, const char * argv[]) {
    namedWindow("Line Mask", WINDOW_NORMAL);
    namedWindow("Detected Lines", WINDOW_NORMAL);
    
    if(argc < 3){
        std::cout << "Expected file name and frames folder passed" << std::endl;
    }

    Vec4i line = Vec4i(1956, 452, 1235, 435);
    linearParameters(line);
    
    #if CUSTOM_EXPORT_SESSION
        performCustomExportSession(argv[2]);
    #elif USING_SCREEN_CAPTURE
        performTransformFromScreenCapture();
    #else
        performTransformFromLoadedImage(argv[1]);
    #endif
    
    return 0;
}





