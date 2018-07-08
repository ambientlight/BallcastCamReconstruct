//
//  ScreenCaptureTest.m
//  BallcastCamReconstruct
//
//  Created by Taras Vozniuk on 07/06/2018.
//  Copyright © 2018 Ballcast. All rights reserved.
//

#include <thread>
#include <chrono>
#import "ScreenCaptureSource.h"
#import <CoreMedia/CoreMedia.h>
#import <AppKit/AppKit.h>

@interface ScreenCaptureSource()

@property(nonatomic, retain) AVCaptureSession* captureSession;
@property(nonatomic, retain) AVCaptureVideoDataOutput* output;
@property(nonatomic, retain) AVCaptureScreenInput* input;

@property(nonatomic, unsafe_unretained) Semaphore* semaphore;
@property(nonatomic, unsafe_unretained) CVImageBufferRef lastFrameBuffer;

@property(nonatomic) BOOL complete;
@property(nonatomic) BOOL shouldGetNextFrame;

@end

@implementation ScreenCaptureSource

ScreenCaptureSourceImpl::ScreenCaptureSourceImpl(): self(NULL){
}

ScreenCaptureSourceImpl::~ScreenCaptureSourceImpl(void) {
    [(__bridge id)self dealloc];
}

void ScreenCaptureSourceImpl::init(Semaphore* semaphore) {
    self = [[ScreenCaptureSource alloc] initWithSemaphore:semaphore];
}

void ScreenCaptureSourceImpl::setShouldGetNextFrame(bool shouldGetNextFrame) {
    if(self){
        [(__bridge id)self setShouldGetNextFrame:shouldGetNextFrame];
    }
}

bool ScreenCaptureSourceImpl::isEnabled() const {
    return self ? [(__bridge id)self complete] : true;
}

CVImageBufferRef ScreenCaptureSourceImpl::lastFrameBuffer() const {
    return self ? [(__bridge id)self lastFrameBuffer] : nullptr;
}

- (instancetype)initWithSemaphore:(Semaphore*)semaphore {
    self = [super init];
    if(self){
        self.semaphore = semaphore;
        self.complete = false;
        self.captureSession = [AVCaptureSession new];
        
        NSArray<NSScreen *>* screens = [NSScreen screens];
        NSScreen* targetScreen = [NSScreen mainScreen];
        if(screens.count > 1){
            //NSLog(@"Multiple screens available, selecting first non-main one");
            for (NSScreen* screen in screens) {
                if([screen isEqual:[NSScreen mainScreen]]){
                    continue;
                }
                
                targetScreen = screen;
                break;
            }
        }
        
        CGDirectDisplayID displayId = [[targetScreen.deviceDescription valueForKey:@"NSScreenNumber"] intValue];
        self.input = [[AVCaptureScreenInput alloc] initWithDisplayID:displayId];
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
    
    // if main thread is processing the frame drop it
    if(!self.shouldGetNextFrame){
        //NSLog(@"Drop frame");
        return;
    }
    
    CVPixelBufferRef imageBuffer = CMSampleBufferGetImageBuffer(sampleBuffer);
    CVPixelBufferRetain(imageBuffer);
    self.lastFrameBuffer = imageBuffer;
    
    self.shouldGetNextFrame = false;
    self.semaphore->notify();
}

@end
