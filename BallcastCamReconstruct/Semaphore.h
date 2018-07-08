//
//  Semaphore.hpp
//  BallcastCamReconstruct
//
//  Created by Taras Vozniuk on 08/06/2018.
//  Copyright © 2018 Ballcast. All rights reserved.
//

#ifndef Semaphore_hpp
#define Semaphore_hpp

#include <mutex>
#include <condition_variable>

class Semaphore {
public:
    Semaphore (int count_ = 0): count(count_) {}
    
    inline void notify(){
        std::unique_lock<std::mutex> lock(mtx);
        count++;
        cv.notify_one();
    }
    
    inline void wait(){
        std::unique_lock<std::mutex> lock(mtx);
        cv.wait(lock, [this]() { return count > 0; });
        count--;
    }
    
    inline bool wouldWait(){
        return count == 0;
    }
    
private:
    std::mutex mtx;
    std::condition_variable cv;
    int count;
};

#endif /* Semaphore_hpp */
