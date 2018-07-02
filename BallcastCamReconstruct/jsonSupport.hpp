//
//  jsonSupport.hpp
//  BallcastCamReconstruct
//
//  Created by Taras Vozniuk on 02/07/2018.
//  Copyright © 2018 Ballcast. All rights reserved.
//

#ifndef jsonSupport_hpp
#define jsonSupport_hpp

#include <iostream>
#include <algorithm>

namespace bf {
    struct Vector3f {
        float x;
        float y;
        float z;
    };
    
    struct Size {
        float widht;
        float height;
    };
    
    struct CameraParameters {
        Vector3f position;
        Vector3f rotation;
        float fov;
    };
    
    struct FrameSample {
        std::string type;
        std::string completeness;
        bool wasCompleted;
        
        CameraParameters camera;
        Vector3f ballPosition;
        std::string imagePath;
        double progress;
        Size canvasSize;
        
        //std::vector<std::string> relationIds;
    };
}

#endif /* jsonSupport_hpp */
