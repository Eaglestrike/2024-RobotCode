#pragma once

#define SHOOTER_AUTO_TUNE false

#include <map>

#include "Util/Mechanism.h"
#include "Util/simplevectors.hpp"

#if SHOOTER_AUTO_TUNE
#include "FFAutotuner/FFAutotuner.h"
#endif

#include "ShuffleboardSender/ShuffleboardSender.h"

#include "Submechanisms/Flywheel.h"
#include "Submechanisms/Pivot.h"

#include "Constants/ShooterConstants.h"

namespace vec = svector;

class Shooter : public Mechanism{
    public:
        enum State{
            STOP,
            PREPARING,
            STROLL //Set to low speed
        };
  
        Shooter(std::string name, bool enabled, bool shuffleboard);

        void Stop();
        void Stroll();

        void SetUp(double vel, double spin, double ang);
        void Prepare(vec::Vector2D robotPos, vec::Vector2D robotVel, bool blueSpeaker);

        bool CanShoot(vec::Vector2D robotPos, vec::Vector2D robotVel, double robotYaw);

        double GetTargetRobotYaw();

        void SetOdometry(vec::Vector2D robotPos, vec::Vector2D robotVel, double robotYaw);//Debug info passing in

    private:
        void CoreInit() override;
        void CorePeriodic() override;
        void CoreTeleopPeriodic() override;
        
        void CoreShuffleboardInit() override;
        void CoreShuffleboardPeriodic() override;

        State state_;

        Flywheel lflywheel_;
        Flywheel rflywheel_;
        Pivot pivot_;

        //Shooter config
        double strollSpeed_ = ShooterConstants::STROLL_SPEED;

        std::map<double, ShooterConstants::ShootConfig> shootData_ = ShooterConstants::SHOOT_DATA;
        double kSpin_ = ShooterConstants::K_SPIN;

        ShooterConstants::ShootConfig shot_;
        double spin_;

        //Odometry Targets
        vec::Vector2D targetPos_;
        vec::Vector2D targetVel_;
        double targetYaw_;

        double posTol_ = ShooterConstants::SHOOT_POS_TOL;
        double velTol_ = ShooterConstants::SHOOT_VEL_TOL;
        double yawTol_ = ShooterConstants::SHOOT_YAW_TOL;

        //Debug odom vals
        vec::Vector2D robotPos_;
        vec::Vector2D robotVel_;
        double robotYaw_;

        //Kinematic calculations
        struct FKRes{
            bool score; //If the shot will score
            bool aimed; //if the robot is actually aiming at the target
            vec::Vector2D error; //width, height
        };
        FKRes CalculateForwardKinematics(vec::Vector2D robotPos, vec::Vector2D robotVel, double robotYaw, vec::Vector2D target, ShooterConstants::ShootConfig shot);

        struct IKRes{
            double targRobotYaw;
            ShooterConstants::ShootConfig shot;
        };
        IKRes CalculateInverseKinematics(vec::Vector2D target);

        //ShuffleboardSender
        std::string StateToString(State state);
        ShuffleboardSender shuff_;

        #if SHOOTER_AUTO_TUNE
        //Tuning will override any state
        bool lflyTuning_, rflyTuning_, pivotTuning_;
        FFAutotuner lflyTuner_, rflyTuner_, pivotTuner_;
        #endif
};