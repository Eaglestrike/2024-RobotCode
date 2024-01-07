#pragma once

namespace FFHelpers{
    inline double sign(double x){
        if(x > 0){
            return 1.0;
        }
        else if(x < 0){
            return -1.0;
        }
        return 0.0;
    }
}