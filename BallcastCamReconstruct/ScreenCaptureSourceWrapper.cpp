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

CVImageBufferRef ScreenCaptureSourceWrapper::lastFrameBuffer() const {
    return _impl != nullptr ? _impl->lastFrameBuffer() : nullptr;
}

