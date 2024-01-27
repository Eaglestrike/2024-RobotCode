#pragma once

#include "Poses.h"

/**
 * Profile to get from one velocity to another
 * 
 * A straight line
*/
class VelocityProfile{
    public:
        VelocityProfile(double maxA);

        void SetTarget(double finalVel, Poses::Pose1D startPose);
        Poses::Pose1D GetPose();

        bool isFinished();
        double GetDuration();

        double GetMaxA();
        void SetMaxA(double maxA);

    private:
        double maxA_;

        Poses::Pose1D startPose_;
        double startTime_;
        Poses::Pose1D finalPose_;
        double finalTime_;
};