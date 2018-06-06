//
//  ScreenCaptureTest.m
//  BallcastCamReconstruct
//
//  Created by Taras Vozniuk on 07/06/2018.
//  Copyright © 2018 Ballcast. All rights reserved.
//

#import "ScreenCaptureSource.h"

@implementation ScreenCaptureSource

ScreenCaptureSourceImpl::ScreenCaptureSourceImpl(void): self(NULL){
}

ScreenCaptureSourceImpl::~ScreenCaptureSourceImpl(void)
{
    [(__bridge id)self dealloc];
}

void ScreenCaptureSourceImpl::init(void)
{
    self = [[ScreenCaptureSource alloc] init];
}

int ScreenCaptureSourceImpl::doSomethingWith(void* aParameter)
{
    return [(id)self doSomethingWith:aParameter];
}

void ScreenCaptureSourceImpl::logMyMessage(char* aCStr)
{
    [(id)self logMyMessage:aCStr];
}

- (int) doSomethingWith:(void*)aParameter
{
    int result = 0;
    
    // ... some code to calculate the result
    
    return result;
}

- (void)logMyMessage:(char*)aCStr
{
    NSLog(@"%@", [NSString stringWithUTF8String:aCStr]);
}

@end
