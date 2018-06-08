//
//  ScreenCaptureSource-C-Interface.h
//  BallcastCamReconstruct
//
//  Created by Taras Vozniuk on 07/06/2018.
//  Copyright © 2018 Ballcast. All rights reserved.
//

#ifndef ScreenCaptureSource_C_Interface_h
#define ScreenCaptureSource_C_Interface_h

#import "Semaphore.h"
#import <CoreVideo/CoreVideo.h>

class ScreenCaptureSourceImpl {
public:
    ScreenCaptureSourceImpl(void);
    ~ScreenCaptureSourceImpl(void);
    
    void init(Semaphore*);
    bool isEnabled() const;
    CVImageBufferRef lastFrameBuffer() const;

private:
    void* self;
};

#endif /* ScreenCaptureSource_C_Interface_h */
