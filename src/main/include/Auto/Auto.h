#pragma once

#include "ShuffleboardSender/ShuffleboardSender.h"

#include "Constants/AutoConstants.h"

#include "AutoPathSegment.h"

class Auto{
    public:
        Auto(bool shuffleboard);

        void Periodic();

    private:
        AutoPathSegment segments_; //Drive segments

        AutoConstants::AutoPath path_; //Path instructions

        ShuffleboardSender shuff_;
};