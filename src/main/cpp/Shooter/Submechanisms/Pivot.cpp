#include "Shooter/Submechanisms/Pivot.h"

#include <cmath>

#include <ctre/phoenix6/controls/Follower.hpp>

#include "Util/Utils.h"

using ctre::phoenix6::controls::Follower;

/**
 * Constructor
 * 
 * @param name name
 * @param enabled enabled
 * @param shuffleboard shuffleboard debug
*/
Pivot::Pivot(std::string name, bool enabled, bool shuffleboard):
    Mechanism(name, enabled, shuffleboard),
    state_{STOP},

    motor_{ShooterConstants::PIVOT_ID},
    motorChild_{ShooterConstants::PIVOT_CHILD_ID},
    //follower_{ShooterConstants::PIVOT_ID, true},
    gearing_{ShooterConstants::PIVOT_GEARING},
    volts_{0.0},
    maxVolts_{ShooterConstants::PIVOT_MAX_VOLTS},

    encoder_{ShooterConstants::PIVOT_ENCODER_ID, ShooterConstants::SHOOTER_CANBUS},
    offset_{ShooterConstants::PIVOT_OFFSET},

    bounds_{
        .min = ShooterConstants::PIVOT_MIN,
        .max = ShooterConstants::PIVOT_MAX
    },
    pid_{ShooterConstants::PIVOT_PID}, accum_{0.0},
    ff_{ShooterConstants::PIVOT_FF},
    posTol_{ShooterConstants::PIVOT_POS_TOL},
    velTol_{ShooterConstants::PIVOT_VEL_TOL},
    profile_{ShooterConstants::PIVOT_MAX_V, ShooterConstants::PIVOT_MAX_A},
    currPose_{0.0, 0.0, 0.0},
    shuff_{name, shuffleboard}
{
    // motor_.RestoreFactoryDefaults();
    // motorChild_.RestoreFactoryDefaults();

    // motor_.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
    // motorChild_.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);

    motor_.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
    motorChild_.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);

    motor_.SetInverted(true);
    motorChild_.SetInverted(false);

    //motorChild_.SetControl(follower_);

    //motorChild_.Follow(motor_, true);
}

/**
 * Core functions
*/
void Pivot::CorePeriodic(){
    double pos = -2*M_PI * encoder_.GetAbsolutePosition().GetValueAsDouble() + offset_;
    double vel = -2*M_PI * encoder_.GetVelocity().GetValueAsDouble(); //Rotations -> Radians
    double acc = (vel - currPose_.vel)/0.02; //Sorry imma assume
    currPose_ = {pos, vel, acc};
}

void Pivot::CoreTeleopPeriodic(){
    double t = Utils::GetCurTimeS();
    double dt = t - prevT_;
    if(dt > 0.3){
        dt = 0.0;
    }
    switch(state_){
        case STOP:
            volts_ = 0.0;
            break;
        case AIMING:
            // if(profile_.isFinished()){ //Enable if profile is good but tolerances are bad
            //     state_ = AT_TARGET;
            // }
            [[fallthrough]];
        case AT_TARGET:
        {
            Poses::Pose1D target = profile_.currentPose();
            double ff = ff_.ks*Utils::Sign(target.vel) + ff_.kv*target.vel + ff_.ka*target.acc + ff_.kg*cos(target.pos);

            Poses::Pose1D error = target - currPose_;
            accum_ += error.pos * dt;
            double pid = pid_.kp*error.pos + pid_.ki*accum_ + pid_.kd*error.vel;

            volts_ = ff + pid;

            bool atTarget = (std::abs(error.pos) < posTol_) && (std::abs(error.vel) < velTol_);
            if(state_ == AIMING && profile_.isFinished()){ //if case deal with fallthrough
                if(atTarget){
                    state_ = AT_TARGET; //At target due to tolerances
                }
                else{
                    profile_.regenerate(currPose_);
                }
            }
            if (state_ == AT_TARGET){
                if(!atTarget){ //Regenerate profile if it shifts out of bounds (TODO test)
                    profile_.regenerate(currPose_);
                    state_ = AIMING;
                }
            }

            if(shuff_.isEnabled()){ //Shuff prints
                shuff_.PutNumber("pos error", error.pos, {1,1,5,3});
                shuff_.PutNumber("vel error", error.vel, {1,1,6,3});
            }
            break;
        }
        case JUST_VOLTAGE:
            break; //Voltage already set through volts_
    }
    volts_ = std::clamp(volts_, -maxVolts_, maxVolts_);
    if((currPose_.pos > bounds_.max) && (volts_ > ff_.kg*cos(bounds_.max))){
        volts_ = 0.0;
    }
    else if((currPose_.pos < bounds_.min) && (volts_ < ff_.kg*cos(bounds_.min))){
        volts_ = 0.0;
    }
    motor_.SetVoltage(units::volt_t{volts_ + 0.2*Utils::Sign(volts_)}); //This motor has more weight
    motorChild_.SetVoltage(units::volt_t{volts_});

    prevT_ = t;
}

/**
 * Sets to idle (no voltage)
*/
void Pivot::Stop(){
    state_ = STOP;
}

/**
 * Pivot starts aiming towards the angle as long as it's in bounds
*/
void Pivot::SetAngle(double angle){
    if(angle > bounds_.max || angle < bounds_.min){
        return;
    }
    Poses::Pose1D target = {.pos = angle, .vel = 0.0, .acc = 0.0};
    profile_.setTarget(currPose_, target);
    accum_ = 0.0;

    Poses::Pose1D error = target - currPose_;
    bool atTarget = (std::abs(error.pos) < posTol_) && (std::abs(error.vel) < velTol_);
    if(atTarget){
        state_ = AT_TARGET;
    }
    else{
        state_ = AIMING;
    }
}

/**
 * Pivot just sets the voltage
*/
void Pivot::SetVoltage(double volts){
    volts_ = volts;
    state_ = JUST_VOLTAGE;
}

/**
 * Zeros the encoder (should be level)
*/
void Pivot::Zero(){
    offset_ = 2*M_PI * encoder_.GetAbsolutePosition().GetValueAsDouble() + bounds_.min;
}

/**
 * Is at target
*/
bool Pivot::AtTarget(){
    //std::cout<<"Pivot: "<<StateToString(state_)<<std::endl;
    return (state_ == AT_TARGET);
}

/**
 * Get Pose
*/
Poses::Pose1D Pivot::GetPose(){
    return currPose_;
}

/**
 * State to string
*/
std::string Pivot::StateToString(Pivot::State state){
    switch(state){
        case STOP : return "Stop";
        case AIMING : return "AIMING";
        case AT_TARGET : return "AT_TARGET";
        case JUST_VOLTAGE : return "Voltage";
        default: return "Unknown";
    }
}

/**
 * Core Shuffleboard prints
*/
void Pivot::CoreShuffleboardInit(){
    //Voltage Control (row 0)
    shuff_.add("volts", &volts_, {1,1,0,0}, true);
    shuff_.add("max volts", &maxVolts_, {1,1,1,0}, true);
    shuff_.addButton("Set Voltage",
                    [&](){
                            SetVoltage(volts_);
                            std::cout<<"Set Voltage to " << volts_ << std::endl;
                        },
                    {1,1,2,0}
                    );
    shuff_.addButton("Stop", [&](){Stop();}, {1,1,3,0});

    //Info (middle-right)
    shuff_.PutString("State", StateToString(state_), {2,1,4,0});
    shuff_.add("pos", &currPose_.pos, {1,1,4,1}, false);
    shuff_.add("vel", &currPose_.vel, {1,1,5,1}, false);
    shuff_.add("acc", &currPose_.acc, {1,1,6,1}, false);
    shuff_.add("volts", &volts_, {1,1,4,2}, false);
    shuff_.addButton("zero", [&](){Zero(); std::cout<<"Zeroed"<<std::endl;}, {1,1,6,2});

    //Bounds (middle-bottom)
    shuff_.add("min", &bounds_.min, {1,1,4,4}, true);
    shuff_.add("max", &bounds_.max, {1,1,5,4}, true);
    shuff_.add("offset", &offset_, {1,1,6,4}, true);

    //Velocity Control (row 2 and 3)
    shuff_.PutNumber("Angle Targ", 0.0, {1,1,0,2});
    shuff_.PutNumber("Max Vel", profile_.getMaxVel(), {1,1,1,2});
    shuff_.PutNumber("Max Acc", profile_.getMaxAcc(), {1,1,2,2});
    shuff_.addButton("Set Target",
                    [&](){
                            double maxV = shuff_.GetNumber("Max Vel", ShooterConstants::FLYWHEEL_MAX_A);
                            double maxA = shuff_.GetNumber("Max Acc", ShooterConstants::FLYWHEEL_MAX_A);
                            profile_.setMaxVel(maxV);
                            profile_.setMaxAcc(maxA);
                            std::cout<<"Set maxV to " << maxV << " : " <<"Set maxA to " << maxA <<std::endl;
                            double targ = shuff_.GetNumber("Angle Targ", 0.0);
                            SetAngle(targ);
                            std::cout<<"Set Target to " << targ << std::endl;
                        },
                    {1,1,3,2}
                    );
    shuff_.add("pos tol", &posTol_, {1,1,4,2});
    shuff_.add("vel tol", &velTol_, {1,1,5,2});

    shuff_.add("kS", &ff_.ks, {1,1,0,3}, true);
    shuff_.add("kV", &ff_.kv, {1,1,1,3}, true);
    shuff_.add("kA", &ff_.ka, {1,1,2,3}, true);
    shuff_.add("kG", &ff_.kg, {1,1,3,3}, true);

    shuff_.add("kP", &pid_.kp, {1,1,0,4}, true);
    shuff_.add("kI", &pid_.ki, {1,1,1,4}, true);
    shuff_.add("kD", &pid_.kd, {1,1,2,4}, true);
}

void Pivot::CoreShuffleboardPeriodic(){
    shuff_.PutString("State", StateToString(state_));

    shuff_.update(true);

    if(maxVolts_ < 0.0){
        maxVolts_ = 0.0;
    }
}