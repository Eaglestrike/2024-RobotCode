#pragma once

#include <vector>

#include "ShuffleboardSender/ShuffleboardSender.h"

#include "Intake/Intake.h"
#include "Drive/SwerveControl.h"
#include "Util/Odometry.h"
#include "AutoPathSegment.h"

#include "Constants/AutoConstants.h"

class Auto{
    public:
        Auto(bool shuffleboard, SwerveControl &swerve, Odometry &odom, Intake &intake);
        void SetPath(AutoConstants::AutoPath path, uint index);

        void AutoInit();
        void AutoPeriodic();
        
        void ShuffleboardInit();
        void ShuffleboardPeriodic();

    private:
        void DrivePeriodic(double t);
        void ShooterPeriodic(double t);
        void IntakePeriodic(double t);

        void LoadPath(const AutoConstants::AutoPath& path);
        AutoPathSegment segments_; //Drive segments
        Intake &intake_;

        //Shooter &shooter_;
        //Intake &intake_;

        std::vector<AutoConstants::AutoPath> paths_; //Path instructions

        int pathNum_;
        int index_;
        double blockStart_;
        double blockEnd_;
        void NextBlock();
        void EvaluateElement(AutoConstants::AutoElement element);
        void EvaluateDriveElement(AutoConstants::AutoElement element);
        void EvaluateShootElement(AutoConstants::AutoElement element);
        void EvaluateIntakeElement(AutoConstants::AutoElement element);

        struct SubsystemTiming{
            bool finished;
            double start;
            bool hasStarted;
            double end;
        };        
        void ResetTiming(SubsystemTiming& timing);

        SubsystemTiming driveTiming_;
        SubsystemTiming shooterTiming_;
        bool intaking_; //If the intake should be intaking or stowed on this block
        SubsystemTiming intakeTiming_;

        ShuffleboardSender shuff_;
};