//
//  ScreenCaptureSourceWrapper.h
//  BallcastCamReconstruct
//
//  Created by Taras Vozniuk on 07/06/2018.
//  Copyright © 2018 Ballcast. All rights reserved.
//

#ifndef ScreenCaptureSourceWrapper_h
#define ScreenCaptureSourceWrapper_h

class ScreenCaptureSourceImpl;

class ScreenCaptureSourceWrapper {
public:
    ScreenCaptureSourceWrapper(void);
    ~ScreenCaptureSourceWrapper(void);
    
    void init(void);
    void doSomethingWithMyClass(void);

private:
    ScreenCaptureSourceImpl* _impl;
};

#endif /* ScreenCaptureSourceWrapper_h */
