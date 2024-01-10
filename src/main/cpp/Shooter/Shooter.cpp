#include "Shooter/Shooter.h"

Shooter::Shooter(std::string name, bool enabled, bool shuffleboard):
    Mechanism{name, enabled, shuffleboard},
    state_{IDLE},
    flywheel_{name + " flywheel", enabled, shuffleboard},
    pivot_{name + " pivot", enabled, shuffleboard},
    shootData_{ShooterConstants::SHOOT_DATA},
    shootSpin_{ShooterConstants::SHOOT_SPIN},
    shuff_{name, shuffleboard}
{

}

void Shooter::Idle(){
    flywheel_.Idle();
    pivot_.Idle();
}

void Shooter::Stroll(){
    state_ = STROLL;
}

void Shooter::Prepare(vec::Vector2D toSpeaker){
    double dist = toSpeaker.magn();
    double ang = toSpeaker.angle();

    shootData_.lower_bound(dist);
}

bool Shooter::CanShoot(){
    return state_ == PREPARED;
}

void Shooter::CoreInit(){
    flywheel_.Init();
    pivot_.Init();
}

void Shooter::CorePeriodic(){
    flywheel_.Periodic();
    pivot_.Periodic();
}

void Shooter::CoreTeleopPeriodic(){
    flywheel_.TeleopPeriodic();
    pivot_.TeleopPeriodic();
}

void Shooter::CoreShuffleboardInit(){

}

void Shooter::CoreShuffleboardPeriodic(){

}