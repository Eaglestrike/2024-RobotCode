#include "Shooter/Submechanisms/Flywheel.h"

#include "iostream"

#include "Util/Utils.h"

/**
 * Constructor
*/
Flywheel::Flywheel(ShooterConstants::FlywheelConfig config, bool enabled, bool shuffleboard):
    Mechanism{config.name, enabled, shuffleboard},
    motor_{config.id, ShooterConstants::SHOOTER_CANBUS},
    volts_{0.0},
    maxVolts_{ShooterConstants::FLYWHEEL_MAX_VOLTS},
    state_{State::STOP},
    profile_{ShooterConstants::FLYWHEEL_MAX_A},
    feedforward_{ShooterConstants::FLYWHEEL_FF},
    pid_{ShooterConstants::FLYWHEEL_PID},
    velTol_{ShooterConstants::FLYWHEEL_VEL_TOL},
    shuff_{config.name, shuffleboard}
{
    motor_.SetInverted(config.inverted);

    prevT_ = Utils::GetCurTimeS();
}

//Core Functions
void Flywheel::CorePeriodic(){
    double pos = ShooterConstants::FLYWHEEL_GEARING * 2*M_PI * motor_.GetPosition().GetValueAsDouble() * ShooterConstants::FLYWHEEL_R;
    double vel = ShooterConstants::FLYWHEEL_GEARING * 2*M_PI * motor_.GetVelocity().GetValueAsDouble() * ShooterConstants::FLYWHEEL_R;
    double acc = (vel - currPose_.vel)/0.02; //Sorry imma assume
    currPose_ = {pos, vel, acc};
};

void Flywheel::CoreTeleopPeriodic(){
    double t = Utils::GetCurTimeS();
    double dt = t - prevT_;
    if(dt > 0.04){
        dt = 0.0;
    }
    switch(state_){
        case State::STOP:
            volts_ = 0.0;
            break;
        case State::RAMPING:
            // if(profile_.isFinished()){
            //     state_ = State::AT_TARGET;
            // }
            [[fallthrough]]; //FF + PID always
        case State::AT_TARGET:
        {
            Poses::Pose1D targetPose = profile_.GetPose();
            double ff = (Utils::Sign(targetPose.vel) * feedforward_.ks) + (targetPose.vel * feedforward_.kv) + (targetPose.acc * feedforward_.ka);
            Poses::Pose1D error = targetPose - currPose_;
            if(profile_.isFinished()){
                accum_ += error.vel * dt;
            }
            double pid = (error.vel*pid_.kp) + (accum_*pid_.ki) + (error.acc*pid_.kd);
            volts_ = ff + pid;

            bool atTarget = (std::abs(error.vel) < velTol_); //TODO check if need acc tol
            if(state_ == State::RAMPING && profile_.isFinished()){ //if case deal with fallthrough
                if(atTarget){
                    state_ = State::AT_TARGET; //At target due to tolerances
                }
                else{
                    profile_.Regenerate(currPose_);
                    accum_ = 0.0;
                }
            }
            if (state_ == State::AT_TARGET){
                if(!atTarget){ //Regenerate profile if it shifts out of bounds (TODO test)
                    profile_.Regenerate(currPose_);
                    state_ = State::RAMPING;
                    accum_ = 0.0;
                }
            }

            //Shuffleboard errors (row 2 right side)
            if(shuff_.isEnabled()){
                shuff_.PutNumber("Vel error", error.vel, {1,1,5,2});
                shuff_.PutNumber("Acc error", error.acc, {1,1,6,2});
            }
            break;
        }
        case State::JUST_VOLTAGE:
            break; //Voltage already set
        default:
            volts_ = 0.0;
    }
    volts_ = std::clamp(volts_, -maxVolts_, maxVolts_);
    motor_.SetVoltage(units::volt_t{volts_});
    prevT_ = t;
};

void Flywheel::Stop(){
    state_ = State::STOP;
}

/**
 * Set target velocity
 * 
 * @param vel m/s
 * @param spin rad/s
*/
void Flywheel::SetTarget(double vel){
    bool atTarget = (std::abs(vel - currPose_.vel) < velTol_);

    Poses::Pose1D startPose;
    if(profile_.isFinished()){
        startPose = currPose_;
        if(!atTarget){
            accum_ = 0.0;
        }
    }
    else{
        startPose = profile_.GetPose();
    }
    profile_.SetTarget(vel, startPose);

    if(atTarget){
        state_ = State::AT_TARGET;
    }
    else{
        state_ = State::RAMPING;
    }
}

/**
 * Set target voltage
*/
void Flywheel::SetVoltage(double volts){
    volts_ = volts;
    state_ = State::JUST_VOLTAGE;
}

/**
 * Get information
*/

Flywheel::State Flywheel::GetState(){
    return state_;
}

/**
 * If the flywheel is at the target speed
*/
bool Flywheel::AtTarget(){
    //std::cout<<"State: "<<StateToString(state_)<<std::endl;
    return (state_ == State::AT_TARGET);
}

/**
 * Getters and Setters
*/
Poses::Pose1D Flywheel::GetPose(){
    return currPose_;
}

void Flywheel::SetFeedforward(double ks, double kv, double ka){
    feedforward_.ks = ks;
    feedforward_.kv = kv;
    feedforward_.ka = ka;
}

std::string Flywheel::GetStateStr() {
    return StateToString(state_);
}

void Flywheel::CoreShuffleboardInit(){
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

    //Velocity Control (row 2 and 3)
    shuff_.PutNumber("Vel Targ", 0.0, {1,1,0,2});
    shuff_.PutNumber("Max Acc", profile_.GetMaxA(), {1,1,1,2});
    shuff_.addButton("Set Target",
                    [&](){
                            double maxA = shuff_.GetNumber("Max Acc", ShooterConstants::FLYWHEEL_MAX_A);
                            profile_.SetMaxA(maxA);
                            std::cout<<"Set maxA to " << maxA << std::endl;
                            double targ = shuff_.GetNumber("Vel Targ", 0.0);
                            SetTarget(targ);
                            std::cout<<"Set Target to " << targ << std::endl;
                        },
                    {1,1,2,2}
                    );
    shuff_.add("vel tol", &velTol_, {1,1,3,2});

    shuff_.add("kS", &feedforward_.ks, {1,1,0,3}, true);
    shuff_.add("kV", &feedforward_.kv, {1,1,1,3}, true);
    shuff_.add("kA", &feedforward_.ka, {1,1,2,3}, true);

    //PID (row 4)
    shuff_.add("kP", &pid_.kp, {1,1,0,4}, true);
    shuff_.add("kI", &pid_.ki, {1,1,1,4}, true);
    shuff_.add("kD", &pid_.kd, {1,1,2,4}, true);
    shuff_.add("accum", &accum_, {1,1,3,4}, false);
};

void Flywheel::CoreShuffleboardPeriodic(){
    shuff_.PutString("State", StateToString(state_));

    shuff_.update(true);

    if(maxVolts_ < 0.0){
        maxVolts_ = 0.0;
    }
};

void Flywheel::CoreShuffleboardUpdate(){

};

std::string Flywheel::StateToString(State state){
    switch(state){
        case State::STOP:           return "Stop";
        case State::RAMPING:        return "Ramping";
        case State::AT_TARGET:      return "At Target";
        case State::JUST_VOLTAGE:   return "Voltage";
        default:                    return "Unknown";
    }
}