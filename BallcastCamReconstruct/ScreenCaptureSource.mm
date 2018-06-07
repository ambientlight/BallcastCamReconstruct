//
//  ScreenCaptureTest.m
//  BallcastCamReconstruct
//
//  Created by Taras Vozniuk on 07/06/2018.
//  Copyright © 2018 Ballcast. All rights reserved.
//

#include <thread>
#include <chrono>
#include <opencv2/opencv.hpp>

#import "ScreenCaptureSource.h"
#import <CoreVideo/CoreVideo.h>
#import <CoreMedia/CoreMedia.h>

using namespace cv;

@interface ScreenCaptureSource()

@property(nonatomic, retain) AVCaptureSession* captureSession;
@property(nonatomic, retain) AVCaptureVideoDataOutput* output;
@property(nonatomic, retain) AVCaptureScreenInput* input;

@property(nonatomic) BOOL once;

@end

@implementation ScreenCaptureSource

ScreenCaptureSourceImpl::ScreenCaptureSourceImpl(void): self(NULL){
}

ScreenCaptureSourceImpl::~ScreenCaptureSourceImpl(void) {
    [(__bridge id)self dealloc];
}

void ScreenCaptureSourceImpl::init(void) {
    self = [[ScreenCaptureSource alloc] init];
}


- (instancetype)init {
    self = [super init];
    if(self){
        self.once = false;
        self.captureSession = [AVCaptureSession new];
        self.input = [AVCaptureScreenInput new];
        [self.captureSession addInput:self.input];
        
        self.output = [AVCaptureVideoDataOutput new];
        [self.output setVideoSettings:@{
            (id)kCVPixelBufferPixelFormatTypeKey: [NSNumber numberWithUnsignedInt:kCVPixelFormatType_32BGRA]
        }];
        
        
        [self.output setAlwaysDiscardsLateVideoFrames:true];
        [self.input setMinFrameDuration:CMTimeMake(1, 50)];
        
        self.input.capturesCursor = false;
        self.input.capturesMouseClicks = false;
        
        [self.captureSession addOutput:self.output];
        [self.output setSampleBufferDelegate:self queue:dispatch_get_global_queue(DISPATCH_QUEUE_PRIORITY_HIGH, 0)];
        [self.captureSession startRunning];
    }
    
    return self;
}

- (void)dealloc {
    [self.captureSession stopRunning];
    while(self.captureSession.isRunning){
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    
    [self.output release];
    [self.input release];
    [self.captureSession release];
    [super dealloc];
}

-(void)setFrameRate:(CMTime)interval {
    [self.input setMinFrameDuration:interval];
}

-(void)stop{
    [self.captureSession stopRunning];
}

//-(void)pause {
//    if(self.Paused) return;
//    self.Paused = true;
//    if(self.output){
//        self.output.connections[0].enabled = NO;
//    }
//}

//-(void)resume {
//    if(!self.Paused) return;
//    self.Paused = false;
//    if(self.output){
//        self.output.connections[0].enabled = YES;
//    }
//}

@end

@implementation ScreenCaptureSource(AVCaptureVideoDataOutputSampleBufferDelegate)

- (void)captureOutput:(AVCaptureOutput *)captureOutput didOutputSampleBuffer:(CMSampleBufferRef)sampleBuffer fromConnection:(AVCaptureConnection *)connection {
    
    CVImageBufferRef imageBuffer = CMSampleBufferGetImageBuffer(sampleBuffer);
    CVPixelBufferLockBaseAddress(imageBuffer, kCVPixelBufferLock_ReadOnly);
    
    //size_t bytesPerRow = CVPixelBufferGetBytesPerRow(imageBuffer);
    unsigned char* buffer = static_cast<unsigned char*>(CVPixelBufferGetBaseAddress(imageBuffer));
    CGSize bufferSize = CVImageBufferGetEncodedSize(imageBuffer);
    
    Mat image = Mat((int)bufferSize.height, (int)bufferSize.width, CV_8UC4, buffer);
    //std::cout << image << std::endl;
    std::cout << bufferSize.width << ", " << bufferSize.height << std::endl;
    
    CVPixelBufferUnlockBaseAddress(imageBuffer, kCVPixelBufferLock_ReadOnly);
}

@end
