#include "Shooter/Flywheel.h"

#include "iostream"

#include "Util/Util.h"

/**
 * Constructor
*/
Flywheel::Flywheel(std::string name, bool enabled, bool shuffleboard):
    Mechanism{name, enabled, shuffleboard},
    motor_{ShooterConstants::FLYWHEEL_ID},
    volts_{0.0},
    maxVolts_{ShooterConstants::FLYWHEEL_MAX_VOLTS},
    state_{State::IDLE},
    profile_{ShooterConstants::FLYWHEEL_MAX_A},
    feedforward_{ShooterConstants::FLYWHEEL_FF}
{

}

/**
 * Set target velocity
*/
void Flywheel::SetTarget(double vel){
    profile_.SetTarget(vel, currPose_);
    state_ = State::RAMPING;
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

bool Flywheel::AtTarget(){
    return state_ == State::AT_TARGET;
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

//Core Functions
void Flywheel::CorePeriodic(){
    double pos = motor_.GetSelectedSensorPosition()/2048.0;
    double vel = motor_.GetSelectedSensorVelocity()/2048.0;
    double acc = (vel - currPose_.vel)/0.02; //Sorry imma assume
    currPose_ = {pos, vel, acc};
};

void Flywheel::CoreTeleopPeriodic(){
    switch(state_){
        case State::IDLE:
            volts_ = 0.0;
            break;
        case State::RAMPING:
            if(profile_.isFinished()){
                state_ = State::AT_TARGET;
            }
            [[fallthrough]];
        case State::AT_TARGET:
        {
            Poses::Pose1D targetPose = profile_.GetPose();
            volts_ = (Utils::sign(targetPose.vel) * feedforward_.ks) + (targetPose.vel * feedforward_.kv) + (targetPose.acc * feedforward_.ka);
            break;
        }
        case State::JUST_VOLTAGE:
            break; //Voltage already set
        default:
            volts_ = 0.0;
    }
    volts_ = std::clamp(volts_, -maxVolts_, maxVolts_);
    motor_.SetVoltage(units::volt_t{volts_});
};

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

    //Info (middle-right)
    shuff_.PutString("State", StateToString(state_), {2,1,4,0});
    shuff_.add("pos", &currPose_.pos, {1,1,4,1}, false);
    shuff_.add("vel", &currPose_.vel, {1,1,5,1}, false);
    shuff_.add("acc", &currPose_.acc, {1,1,6,1}, false);

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
    shuff_.add("kS", &feedforward_.ks, {1,1,0,3}, true);
    shuff_.add("kV", &feedforward_.kv, {1,1,1,3}, true);
    shuff_.add("kA", &feedforward_.ka, {1,1,2,3}, true);
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
        case State::IDLE:           return "Idle";
        case State::RAMPING:        return "Ramping";
        case State::AT_TARGET:      return "At Target";
        case State::JUST_VOLTAGE:   return "Voltage";
        default:                    return "Unknown";
    }
}