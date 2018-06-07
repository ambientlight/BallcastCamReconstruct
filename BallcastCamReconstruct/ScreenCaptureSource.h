//
//  ScreenCaptureTest.h
//  BallcastCamReconstruct
//
//  Created by Taras Vozniuk on 07/06/2018.
//  Copyright © 2018 Ballcast. All rights reserved.
//

#ifndef ScreenCaptureSource_h
#define ScreenCaptureSource_h

#import <Foundation/Foundation.h>
#import <AVFoundation/AVFoundation.h>
#import "ScreenCaptureSource-C-Interface.h"

@interface ScreenCaptureSource:  NSObject<AVCaptureVideoDataOutputSampleBufferDelegate> {
    
}

@end

#endif /* ScreenCaptureSource_h */
