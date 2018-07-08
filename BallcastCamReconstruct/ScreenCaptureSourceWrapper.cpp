//
//  ScreenCaptureSourceWrapper.cpp
//  BallcastCamReconstruct
//
//  Created by Taras Vozniuk on 07/06/2018.
//  Copyright © 2018 Ballcast. All rights reserved.
//

#include "ScreenCaptureSourceWrapper.h"
#include "ScreenCaptureSource-C-Interface.h"
#include <iostream>

#include <CoreVideo/CoreVideo.h>

ScreenCaptureSourceWrapper::ScreenCaptureSourceWrapper(void): _impl(nullptr){}

void ScreenCaptureSourceWrapper::init(Semaphore* semaphore){
    _impl = new ScreenCaptureSourceImpl();
    _impl->init(semaphore);
}

ScreenCaptureSourceWrapper::~ScreenCaptureSourceWrapper(){
    if(_impl){
        delete _impl; _impl = nullptr;
    }
}

bool ScreenCaptureSourceWrapper::isEnabled() const {
    return _impl != nullptr ? !_impl->isEnabled() : true;
}

void* ScreenCaptureSourceWrapper::lastFrameBuffer() const {
    return _impl != nullptr ? _impl->lastFrameBuffer() : nullptr;
}

void ScreenCaptureSourceWrapper::setShouldGetNextFrame(bool shouldGetNextFrame) {
    if(_impl){
        _impl->setShouldGetNextFrame(shouldGetNextFrame);
    }
}

void ScreenCaptureSourceWrapper::lockBaseAddress(void* imageBuffer) const {
    CVPixelBufferLockBaseAddress((CVPixelBufferRef)imageBuffer, kCVPixelBufferLock_ReadOnly);
}

void ScreenCaptureSourceWrapper::unlockAndRelease(void* imageBuffer) const {
    CVPixelBufferUnlockBaseAddress((CVPixelBufferRef)imageBuffer, kCVPixelBufferLock_ReadOnly);
    CVPixelBufferRelease((CVPixelBufferRef)imageBuffer);
}

unsigned char* ScreenCaptureSourceWrapper::getBaseAddress(void* imageBuffer) const {
    return static_cast<unsigned char*>(CVPixelBufferGetBaseAddress((CVPixelBufferRef)imageBuffer));
}

float ScreenCaptureSourceWrapper::bufferHeight(void* imageBuffer) const {
    return CVImageBufferGetEncodedSize((CVPixelBufferRef)imageBuffer).height;
}

float ScreenCaptureSourceWrapper::bufferWidth(void* imageBuffer) const {
    return CVImageBufferGetEncodedSize((CVPixelBufferRef)imageBuffer).width;
}
