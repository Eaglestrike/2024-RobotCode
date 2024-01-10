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
    state_{IDLE},
    motor_{ShooterConstants::PIVOT_ID, ShooterConstants::SHOOTER_CANBUS},
    motorChild_{ShooterConstants::PIVOT_CHILD_ID, ShooterConstants::SHOOTER_CANBUS},
    volts_{0.0},
    maxVolts_{ShooterConstants::PIVOT_MAX_VOLTS},
    encoder_{ShooterConstants::PIVOT_ID},
    offset_{ShooterConstants::PIVOT_OFFSET},
    bounds_{
        .min = ShooterConstants::PIVOT_MIN,
        .max = ShooterConstants::PIVOT_MAX
    },
    pid_{ShooterConstants::PIVOT_PID}, accum_{0.0},
    ff_{ShooterConstants::PIVOT_FF},
    profile_{ShooterConstants::PIVOT_MAX_V, ShooterConstants::PIVOT_MAX_A},
    currPose_{0.0, 0.0, 0.0},
    shuff_{name, shuffleboard}
{
    motorChild_.SetControl(Follower(ShooterConstants::PIVOT_ID, true)); //Follow parent
}

/**
 * Core functions
*/
void Pivot::CorePeriodic(){
    double pos = (encoder_.GetAbsolutePosition() + offset_);
    double vel = 2*M_PI * motor_.GetVelocity().GetValueAsDouble(); //Rotations -> Radians
    double acc = (vel - currPose_.vel)/0.02; //Sorry imma assume
    currPose_ = {pos, vel, acc};
}

void Pivot::CoreTeleopPeriodic(){
    switch(state_){
        case IDLE:
            volts_ = 0.0;
            break;
        case AIMING:
            if(profile_.isFinished()){
                state_ = AT_TARGET;
            }
            [[fallthrough]];
        case AT_TARGET:
        {
            Poses::Pose1D target = profile_.currentPose();
            double ff = ff_.ks*Utils::Sign(target.vel) + ff_.kv*target.vel + ff_.ka*target.acc + ff_.kg*cos(target.pos);

            double error = target.pos - currPose_.pos;
            double velError = target.vel - currPose_.vel;
            double pid = pid_.kp*error + pid_.ki*accum_ + pid_.kd*velError;

            volts_ = ff + pid;
            break;
        }
        case JUST_VOLTAGE:
            break; //Voltage already set
    }
    volts_ = std::clamp(volts_, -maxVolts_, maxVolts_);
    motor_.SetVoltage(units::volt_t{volts_});
}

/**
 * Sets to idle (no voltage)
*/
void Pivot::Idle(){
    state_ = IDLE;
}

/**
 * Pivot starts aiming towards the angle as long as it's in bounds
*/
void Pivot::SetAngle(double angle){
    if(angle > bounds_.max || angle < bounds_.min){
        return;
    }

    profile_.setTarget(currPose_, {.pos = angle, .vel = 0.0, .acc = 0.0});
    accum_ = 0.0;
    state_ = AIMING;
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
    //motor_.SetPosition(units::turn_t{0.0}); //Reset relative encoder
    offset_ = -encoder_.GetAbsolutePosition(); 
}

/**
 * Is at target
*/
bool Pivot::AtTarget(){
    return state_ == AT_TARGET;
}

/**
 * Get Pose
*/
Poses::Pose1D Pivot::getPose(){
    return currPose_;
}

/**
 * State to string
*/
std::string Pivot::StateToString(Pivot::State state){
    switch(state){
        case IDLE : return "Idle";
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

    //Info (middle-right)
    shuff_.PutString("State", StateToString(state_), {2,1,4,0});
    shuff_.add("pos", &currPose_.pos, {1,1,4,1}, false);
    shuff_.add("vel", &currPose_.vel, {1,1,5,1}, false);
    shuff_.add("acc", &currPose_.acc, {1,1,6,1}, false);
    shuff_.add("volts", &volts_, {1,1,4,2}, false);
    shuff_.addButton("zero", [&](){Zero(); std::cout<<"Zeroed"<<std::endl;}, {1,1,5,2});

    //Bounds (middle-bottom)
    shuff_.add("min", &bounds_.min, {1,1,4,4}, true);
    shuff_.add("max", &bounds_.max, {1,1,5,4}, true);

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