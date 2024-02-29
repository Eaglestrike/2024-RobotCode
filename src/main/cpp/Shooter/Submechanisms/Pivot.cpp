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
    volts_{0.0},
    maxVolts_{ShooterConstants::PIVOT_MAX_VOLTS},

    encoder_{ShooterConstants::PIVOT_ENCODER_ID, ShooterConstants::SHOOTER_CANBUS},
    offset_{ShooterConstants::PIVOT_OFFSET},
    gearing_{ShooterConstants::PIVOT_GEARING},

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
    motor_.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
    motorChild_.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);

    motor_.SetInverted(true);
    motorChild_.SetInverted(false);
}

void Pivot::CoreInit(){
    ZeroRelative();

    currPose_ = GetAbsPose();
    profile_.setTarget(currPose_, currPose_);

    hooked_ = true;
}

/**
 * Core functions
*/
void Pivot::CorePeriodic(){
    Poses::Pose1D absPose = GetAbsPose();
    Poses::Pose1D newPose = GetRelPose(); //Use relative encoder
    if(std::abs(newPose.pos - absPose.pos) > 0.02){
        ZeroRelative();
        newPose = absPose;
    }
    newPose.acc = (newPose.vel - currPose_.vel)/0.02; //Sorry imma assume
    currPose_ = newPose;
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
            profile_.setTarget(currPose_, currPose_);
            break;
        case UNHOOK:
            posTol_ = 0.02;
            velTol_ = 10.0;
            [[fallthrough]];
        case AIMING:
            [[fallthrough]];
        case AT_TARGET:
        {
            Poses::Pose1D target = profile_.currentPose();
            double ff = ff_.ks*Utils::Sign(target.vel) + ff_.kv*target.vel + ff_.ka*target.acc;
            double grav = ff_.kg*cos(currPose_.pos); //Have gravity be feedback
            
            volts_ = ff + grav;

            Poses::Pose1D error = target - currPose_;
            accum_ += error.pos * dt;
            double pid = pid_.kp*error.pos + pid_.ki*accum_ + pid_.kd*error.vel;
            volts_ += pid;

            if(std::abs(error.pos) < inchTol_){
                cycle_++;
                cycle_ %= inch_.numCycles;
                double inch = (cycle_ < inch_.onCycles) ? inch_.volts : 0.0;
                inch *= Utils::Sign(error.pos);
                volts_ += inch;
            }

            bool finished = profile_.isFinished();
            bool atTarget = (std::abs(error.pos) < posTol_) && (std::abs(error.vel) < velTol_);

            if(state_ == UNHOOK && finished && atTarget){ //Go to next target after unhooking
                hooked_ = false;
                SetAngle(tempTarg_);
                profile_.setMaxAcc(ShooterConstants::PIVOT_MAX_A);
                profile_.setMaxVel(ShooterConstants::PIVOT_MAX_V);
            }
            else if(state_ == AIMING && finished){ //if case to deal with fallthrough
                if(atTarget){
                    state_ = AT_TARGET; //At target due to tolerances
                }
                else{
                    profile_.regenerate(currPose_);
                }
            }
            else if (state_ == AT_TARGET){
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

    if(hooked_){
        tempTarg_ = angle;
        if(state_ == UNHOOK){ //No need to regenerate
            return;
        }
        angle = ShooterConstants::PIVOT_UNHOOK;
        profile_.setMaxAcc(7.0); //Go faster when unhooking
        profile_.setMaxVel(6.0);
    }
    Poses::Pose1D currTarg = profile_.getTargetPose();
    Poses::Pose1D target = {.pos = angle, .vel = 0.0, .acc = 0.0};

    Poses::Pose1D error = target - currPose_;
    bool atTarget = (std::abs(error.pos) < posTol_) && (std::abs(error.vel) < velTol_);
    if(hooked_){
        state_ = UNHOOK;
    }
    else if(atTarget){
        state_ = AT_TARGET;
    }
    else{
        state_ = AIMING;
    }

    if(std::abs(currTarg.pos - angle) > 0.001){ //Basically the same target
        Poses::Pose1D startPose;
        if(profile_.isFinished()){
            startPose = currPose_;
            accum_ = 0.0;
        }
        else{
            startPose = profile_.currentPose();
        }
        profile_.setTarget(startPose, target);
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
 * Zeros the encoder (should be at bottom)
*/
void Pivot::Zero(){
    offset_ = 2*M_PI * encoder_.GetAbsolutePosition().GetValueAsDouble() + bounds_.min;
    ZeroRelative();
}

/**
 * Zeros the relvative encoder (using the absolute encoder)
*/
void Pivot::ZeroRelative(){
    double pos = -2*M_PI * encoder_.GetAbsolutePosition().GetValueAsDouble() + offset_;

    relOffset_ = pos - (2*M_PI * motor_.GetPosition().GetValueAsDouble() * gearing_);
}

/**
 * Gets pose using absolute encoder
*/
Poses::Pose1D Pivot::GetAbsPose(){
    double pos = -2*M_PI*encoder_.GetAbsolutePosition().GetValueAsDouble() + offset_;
    double vel = -2*M_PI*encoder_.GetVelocity().GetValueAsDouble(); //Rotations -> Radians
    return {pos, vel, 0.0};
}

/**
 * Gets pose using relative encoder
*/
Poses::Pose1D Pivot::GetRelPose(){
    double pos = 2*M_PI*motor_.GetPosition().GetValueAsDouble()*gearing_ + relOffset_;
    double vel = 2*M_PI*motor_.GetVelocity().GetValueAsDouble()*gearing_; //Rotations -> Radians
    return {pos, vel, 0.0};
}

/**
 * Is at target
*/
bool Pivot::AtTarget(){
    //std::cout<<"Pivot: "<<StateToString(state_)<<std::endl;
    return (state_ == AT_TARGET);
}

/**
 * Sets positon tolerance
 * 
 * internally changes vel tol
*/
void Pivot::SetTolerance(double posTol){
    posTol_ = posTol;
    velTol_ = posTol * (ShooterConstants::PIVOT_VEL_TOL / ShooterConstants::PIVOT_POS_TOL); //Scale vel tol by how pos tol scales
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
        case UNHOOK : return "Unhook";
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
    shuff_.addButton("zero", [&](){Zero(); std::cout<<"Zeroed"<<std::endl;}, {1,1,6,2});

    shuff_.add("hooked", &hooked_, {1,1,7,2}, true);

    // shuff_.PutNumber("relPos", 0.0, {1,1,8,1});
    // shuff_.PutNumber("relVel", 0.0, {1,1,9,1});
    // shuff_.PutNumber("absPos", 0.0, {1,1,8,2});
    // shuff_.PutNumber("absVel", 0.0, {1,1,9,2});

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

    shuff_.add("inch volts", &inch_.volts, {1,1,0,5}, true);
    shuff_.add("inch onCycles", &inch_.onCycles, {1,1,1,5}, true);
    shuff_.add("inch numCycles", &inch_.numCycles, {1,1,2,5}, true);
    shuff_.add("inch tol", &inchTol_, {1,1,3,5}, true);
}

void Pivot::CoreShuffleboardPeriodic(){
    shuff_.PutString("State", StateToString(state_));

    // Poses::Pose1D relPose = GetRelPose();
    // Poses::Pose1D absPose = GetAbsPose();
    // shuff_.PutNumber("relPos", relPose.pos);
    // shuff_.PutNumber("relVel", relPose.vel);
    // shuff_.PutNumber("absPos", absPose.pos);
    // shuff_.PutNumber("absVel", absPose.vel);

    shuff_.update(true);

    if(maxVolts_ < 0.0){
        maxVolts_ = 0.0;
    }
}