#pragma once

#include <vector>

#include <frc/geometry/Pose2d.h>

#include "ShuffleboardSender/ShuffleboardSender.h"

#include "Intake/Intake.h"
#include "Drive/SwerveControl.h"
#include "Shooter/Shooter.h"
#include "Util/Odometry.h"
#include "Drive/AutoAngLineup.h"
#include "AutoPathSegment.h"

#include "Constants/AutoConstants.h"
#include "Util/Logger.h"

class Auto{
    public:
        Auto(bool shuffleboard, SwerveControl &swerve, Odometry &odom, AutoAngLineup &autoLineup, Intake &intake, Shooter &shooter, FRCLogger &logger);
        void SetPath(uint index, AutoConstants::AutoPath path);
        void SetSegment(uint index, std::string to, std::string back); //Drive -> Intake -> Drive -> Shoot
        void SetSegment(uint index, std::string path); //Drive -> Intake -> Shoot in place
        void SetDrive(uint index, std::string path);
        void Clear();

        std::vector<frc::Pose2d> GetAllPoses();

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
        Odometry &odometry_;
        AutoAngLineup &autoLineup_;
        Intake &intake_;
        Shooter &shooter_;
        SwerveControl &swerve_;
        FRCLogger &logger_;

        std::vector<AutoConstants::AutoPath> paths_; //Path instructions

        double autoStart_;

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

        bool inChannel_; //Boolean to store data about the blinds spot
        SubsystemTiming driveTiming_;
        bool startedLineup_;
        vec::Vector2D shootPos_;
        SubsystemTiming channelTiming_; //Used to time channel loading time 
        SubsystemTiming shooterTiming_;
        bool hadPiece_;
        SubsystemTiming intakeTiming_;

        double CHANNEL_TIME = AutoConstants::CHANNEL_TIME;
        double SHOOT_TIME = AutoConstants::SHOOT_TIME;
        double INTAKE_TIME = AutoConstants::INTAKE_TIME;

        double DRIVE_PADDING = AutoConstants::DRIVE_PADDING;
        double INTAKE_PADDING = AutoConstants::INTAKE_PADDING;
        double SHOOT_PADDING = AutoConstants::SHOOT_PADDING;

        double SHOOT_POS_TOL = AutoConstants::SHOOT_POS_TOL;

        std::string ElementToString(const AutoConstants::AutoElement element);
        ShuffleboardSender shuff_;
};