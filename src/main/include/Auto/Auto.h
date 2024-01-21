#pragma once

#include <vector>

#include "ShuffleboardSender/ShuffleboardSender.h"

#include "Drive/SwerveControl.h"
#include "Util/Odometry.h"
#include "AutoPathSegment.h"

#include "Constants/AutoConstants.h"

class Auto{
    public:
        Auto(SwerveControl &swerve, Odometry &odom, bool shuffleboard);
        void SetPath(AutoConstants::AutoPath path, int index);

        void AutoInit();
        void AutoPeriodic();

    private:
        void ShooterPeriodic(double t);
        void IntakePeriodic(double t);

        void LoadPath(const AutoConstants::AutoPath& path);
        AutoPathSegment segments_; //Drive segments

        //Shooter &shooter_;
        //Intake &intake_;

        std::vector<AutoConstants::AutoPath> paths_; //Path instructions

        int pathNum_;
        int index_;
        double blockStart_;
        double blockEnd_;
        void NextBlock();
        bool EvaluateElement(AutoConstants::AutoElement element);
        void EvaluateShootElement(AutoConstants::AutoElement element);
        void EvaluateIntakeElement(AutoConstants::AutoElement element);

        struct SubsystemTiming{
            bool finished;
            double start;
            bool hasStarted;
            double end;
        };

        SubsystemTiming shooterTiming_;
        bool intaking_; //If the intake should be intaking or stowed on this block
        SubsystemTiming intakeTiming_;

        ShuffleboardSender shuff_;
};