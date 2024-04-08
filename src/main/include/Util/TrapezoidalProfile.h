#pragma once

#include <frc/Timer.h>

#include "Util/Poses.h"

class TrapezoidalProfile{
    public:
        TrapezoidalProfile(double maxVel, double maxAcc);
        Poses::Pose1D currentPose();
        bool setTarget(Poses::Pose1D currPose, Poses::Pose1D finalPose);
        bool isFinished();

        bool regenerate(Poses::Pose1D currPose);

        double getTime();
        double getDuration();
        double getDisplacement();
        Poses::Pose1D getTargetPose();

        double getMaxVel();
        double getMaxAcc();
        void setMaxVel(double maxVel);
        void setMaxAcc(double maxAcc);
        
        void Zero(Poses::Pose1D pose);

    private:

        frc::Timer timer_;

        double maxVel_;
        double maxAcc_;
        Poses::Pose1D startPose_;
        Poses::Pose1D finalPose_;

        struct CalcTimes{
            double accTime;
            Poses::Pose1D accEnd;
            double coastTime;
            Poses::Pose1D coastEnd;
            double deaccTime;
        }calcTimes_;

        
};