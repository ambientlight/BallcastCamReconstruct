//
//  ScreenCaptureSourceWrapper.h
//  BallcastCamReconstruct
//
//  Created by Taras Vozniuk on 07/06/2018.
//  Copyright © 2018 Ballcast. All rights reserved.
//

#import "Semaphore.h"

#ifndef ScreenCaptureSourceWrapper_h
#define ScreenCaptureSourceWrapper_h

class ScreenCaptureSourceImpl;

class ScreenCaptureSourceWrapper {
public:
    ScreenCaptureSourceWrapper(void);
    ~ScreenCaptureSourceWrapper(void);
    
    void init(Semaphore*);
    bool isEnabled() const;
    void* lastFrameBuffer() const;

    void lockBaseAddress(void* imageBuffer) const;
    unsigned char* getBaseAddress(void* imageBuffer) const;
    float bufferHeight(void* imageBuffer) const;
    float bufferWidth(void* imageBuffer) const;
    void unlockAndRelease(void* imageBuffer) const;
    
private:
    ScreenCaptureSourceImpl* _impl;
};

#endif /* ScreenCaptureSourceWrapper_h */
