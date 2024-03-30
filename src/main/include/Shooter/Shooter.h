#pragma once


#define PIVOT_AUTO_TUNE false

#include <map>

#include "Util/Mechanism.h"
#include "Util/simplevectors.hpp"
#include "Util/Logger.h"

#if PIVOT_AUTO_TUNE
#include "FFAutotuner/FFAutotuner.h"
#endif

#include "ShuffleboardSender/ShuffleboardSender.h"

#include "Submechanisms/Flywheel.h"
#include "Submechanisms/Pivot.h"

#include "Constants/ShooterConstants.h"

#include <rev/CANSparkMax.h>

namespace vec = svector;

class Shooter : public Mechanism{
    public:
        enum State{
            STOP,
            SHOOT,
            FERRY,
            AMP,
            STROLL, //Set to low speed
            MANUAL_TARGET, //Manual input angles
            EJECT,
            THROW //Ferry low
        };
  
        Shooter(std::string name, bool enabled, bool shuffleboard);

        void Stop();
        void Stroll();
        void Amp();
        void ManualTarget(double target);
        void Eject(); //Only spins flywheels
        void Ferry(vec::Vector2D robotPos, vec::Vector2D robotVel);
        void Throw();

        void SetUp(double vel, double spin, double ang);
        void Prepare(vec::Vector2D robotPos, vec::Vector2D robotVel, bool needGamePiece);
        void SetGamepiece(bool hasPiece);

        void Trim(vec::Vector2D trim); //Up/down left/right trim for target
        void ZeroRelative();

        bool CanShoot(int posVal = 0);
        bool UseAutoLineup();
        vec::Vector2D GetTrim();
        bool IsManual();
        bool PivotAtTarget();

        double GetTargetRobotYaw();

        void SetOdometry(vec::Vector2D robotPos, vec::Vector2D robotVel, double robotYaw);//Debug info passing in
        void SetHooked(bool hooked);

        void Log(FRCLogger &logger);

    private:
        void CoreInit() override;
        void CorePeriodic() override;
        void CoreTeleopPeriodic() override;
        
        void CoreShuffleboardInit() override;
        void CoreShuffleboardPeriodic() override;

        State state_;
        bool hasPiece_;
        bool autoStroll_;
        double timerStart_;

        Flywheel bflywheel_;
        Flywheel tflywheel_;
        Pivot pivot_;
        
        // rev::CANSparkMax m_kickerMotor;
        // double kickerVolts_ = -8.0;

        //Shooter config
        double strollSpeed_ = ShooterConstants::STROLL_SPEED;
        double ejectSpeed_ = ShooterConstants::EJECT_SPEED;
        double shootAmpSpeed_ = ShooterConstants::SHOOT_AMP_SPEED;
        double shootTime_ = ShooterConstants::SHOOT_TIME;

        std::map<double, ShooterConstants::ShootConfig> shootData_ = ShooterConstants::SHOOT_DATA;
        std::map<double, ShooterConstants::ShootConfig> ferryData_ = ShooterConstants::FERRY_DATA;
        double kD_ = ShooterConstants::kD;
        double cT_ = ShooterConstants::cT;
        double ferryR_ = ShooterConstants::FERRY_R;

        bool hasShot_;
        ShooterConstants::ShootConfig shot_;
        double spin_;

        //Odometry Targets
        vec::Vector2D targetPos_;
        vec::Vector2D targetVel_;
        double targetYaw_;
        double posYawTol_, negYawTol_; //distances to min and max yaw from target

        vec::Vector2D trim_;

        //Odometry data
        vec::Vector2D robotPos_;
        vec::Vector2D robotVel_;
        double robotYaw_;

        //Tolerances
        double posTol_ = ShooterConstants::SHOOT_POS_TOL; //Driving position tolerance
        double velTol_ = ShooterConstants::SHOOT_VEL_TOL; //Driving velocity tolerance
        //double yawTol_ = ShooterConstants::SHOOT_YAW_TOL; //Driving yaw tolerance
        double shootYawPercent_ = ShooterConstants::SHOOT_YAW_PERCENT; //Percent of shootable area
        double lineupYawPercent_ = ShooterConstants::LINEUP_YAW_PERCENT; //Percent of lineup needing area
        double pivotAngPercent_ = ShooterConstants::PIVOT_ANG_PERCENT;
        


        //Kinematic calculations (unused)
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

        #if PIVOT_AUTO_TUNE
        //Tuning will override any state
        bool pivotTuning_;
        FFAutotuner pivotTuner_;
        #endif
};