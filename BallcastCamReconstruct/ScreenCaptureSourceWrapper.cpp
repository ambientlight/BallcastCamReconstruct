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

void ScreenCaptureSourceWrapper::init(){
    _impl = new ScreenCaptureSourceImpl();
    _impl->init();
}

ScreenCaptureSourceWrapper::~ScreenCaptureSourceWrapper(){
    if(_impl){
        delete _impl; _impl = nullptr;
    }
}

