#include "Shooter/Shooter.h"

Shooter::Shooter(std::string name, bool enabled, bool shuffleboard):
    Mechanism{name, enabled, shuffleboard},
    state_{STOP},
    lflywheel_{ShooterConstants::LEFT_FLYWHEEL, enabled, shuffleboard},
    rflywheel_{ShooterConstants::RIGHT_FLYWHEEL, enabled, shuffleboard},
    pivot_{"Pivot", enabled, shuffleboard},
    strollSpeed_{ShooterConstants::STROLL_SPEED},
    shootData_{ShooterConstants::SHOOT_DATA},
    kSpin_{ShooterConstants::K_SPIN},
    shuff_{name, shuffleboard}

    #if SHOOTER_AUTO_TUNE
    ,lflyTuning_{false}, rflyTuning_{false}, pivotTuning_{false},
    lflyTuner_{"left flywheel tuner", FFAutotuner::SIMPLE},
    rflyTuner_{"right flywheel tuner", FFAutotuner::SIMPLE}, 
    pivotTuner_{"pivot tuner", FFAutotuner::ARM}
    #endif
{

}

/**
 * Core functions
*/

void Shooter::CoreInit(){
    lflywheel_.Init();
    rflywheel_.Init();
    pivot_.Init();
}

void Shooter::CorePeriodic(){
    #if SHOOTER_AUTO_TUNE
    rflyTuner_.ShuffleboardUpdate();
    lflyTuner_.ShuffleboardUpdate();
    pivotTuner_.ShuffleboardUpdate();
    #endif

    lflywheel_.Periodic();
    rflywheel_.Periodic();
    pivot_.Periodic();
}

void Shooter::CoreTeleopPeriodic(){
    switch(state_){
        case STOP:
            break;
        case PREPARING:
            if(lflywheel_.AtTarget() && rflywheel_.AtTarget() && pivot_.AtTarget()){
                state_ = PREPARED;
            }
            break;
        case PREPARED:
            break;
        default:
            break;
    }

    #if SHOOTER_AUTO_TUNE
    if(lflyTuning_){
        lflyTuner_.setPose(lflywheel_.GetPose());
        lflywheel_.SetVoltage(lflyTuner_.getVoltage());
    }
    if(rflyTuning_){
        rflyTuner_.setPose(rflywheel_.GetPose());
        rflywheel_.SetVoltage(rflyTuner_.getVoltage());
    }
    if(pivotTuning_){
        pivotTuner_.setPose(pivot_.GetPose());
        pivot_.SetVoltage(pivotTuner_.getVoltage());
    }
    #endif

    lflywheel_.TeleopPeriodic();
    rflywheel_.TeleopPeriodic();
    pivot_.TeleopPeriodic();
}

/**
 * Turns off (no voltage)
*/
void Shooter::Stop(){
    lflywheel_.Stop();
    rflywheel_.Stop();
    pivot_.Stop();
    state_ = STOP;
}

/**
 * Sets to low speed/voltage to constantly spin
*/
void Shooter::Stroll(){
    // lflywheel_.SetTarget(strollSpeed_);
    // rflywheel_.SetTarget(strollSpeed_);
    lflywheel_.SetVoltage(strollSpeed_);
    rflywheel_.SetVoltage(strollSpeed_);
    pivot_.Stop();
    state_ = STROLL;
}

/**
 * Prepares the shot to the speaker
 * 
 * @param toSpeaker field-oriented vector to the speaker 
*/
void Shooter::Prepare(vec::Vector2D toSpeaker){
    double dist = toSpeaker.magn();
    double ang = toSpeaker.angle();

    auto shot = shootData_.lower_bound(dist);
    if(shot == shootData_.begin() || shot == shootData_.end()){ //No shot in data (too far or too close)
        //Check functionality (maybe idle at some distance)
        Stroll();
        return;
    }

    //Interpolate between nearby data points
    double upperDist = shot->first;
    ShooterConstants::ShootConfig upperShot = shot->second;
    shot--;
    double lowerDist = shot->first;
    ShooterConstants::ShootConfig lowerShot = shot->second;

    double percent = (dist - lowerDist) / (upperDist - lowerDist); //Will break if there are 2 data points with the same distance
    double upperPercent = 1.0 - percent;

    double pivotAng = percent*lowerShot.ang + upperPercent*upperShot.ang;
    double shotVel = percent*lowerShot.vel + upperPercent*upperShot.vel;

    //Add spin
    double spin = -ang * kSpin_; //Spin opposite to way pointing

    SetUp(shotVel, spin, pivotAng);
}

/**
 * Internal method to shoot at vel, spin and pivot angle
*/
void Shooter::SetUp(double vel, double spin, double pivot){
    lflywheel_.SetTarget(vel - spin);
    rflywheel_.SetTarget(vel + spin);
    pivot_.SetAngle(pivot);

    state_ = PREPARING;
}

/**
 * Returns if you can shoot
*/
bool Shooter::CanShoot(){
    return state_ == PREPARED;
}

std::string Shooter::StateToString(State state){
    switch(state){
        case STOP: return "Stop";
        case PREPARING: return "Preparing";
        case PREPARED: return "Prepared";
        case STROLL: return "Stroll";
        default: return "Unknown";
    }
}

/**
 * Shuffleboard
 * 
 * Button functionality should be used w/o calling prepare in robot
*/
void Shooter::CoreShuffleboardInit(){
    //Strolling (row 0)
    shuff_.add("Stroll Speed", &strollSpeed_, {1,1,0,0}, true);
    shuff_.addButton("Stroll",
        [&](){
            Stroll();
            std::cout<<"Strolling: "<<strollSpeed_<<std::endl;
        }, {1,1,1,0});

    //State (rightside)
    shuff_.PutString("State", StateToString(state_), {2,1,4,0});

    #if SHOOTER_AUTO_TUNE
    shuff_.add("Lfly Autotune", &lflyTuning_, {1,1,3,1}, true);
    shuff_.add("Rfly Autotune", &rflyTuning_, {1,1,4,1}, true);
    shuff_.add("Pivot Autotune", &pivotTuning_, {1,1,5,1}, true);
    #endif

    //Setup or add to ShootData (row 2)
    shuff_.PutNumber("Vel", 0.0, {1,1,0,2});
    shuff_.PutNumber("Spin", 0.0, {1,1,1,2});
    shuff_.PutNumber("Pivot", 0.0, {1,1,2,2});
    shuff_.addButton("Set Up",
        [&](){
            double vel = shuff_.GetNumber("Vel", 0.0);
            double spin = shuff_.GetNumber("Spin", 0.0);
            double pivot = shuff_.GetNumber("Pivot", 0.0);
            SetUp(vel, spin, pivot);
            std::cout<<"Set Up"<<" vel:"<<vel<<" spin:"<<spin<<" pivot:"<<pivot<<std::endl;
        }, {1,1,1,2});
    shuff_.PutNumber("Distance", 0.0, {1,1,3,2});
    shuff_.addButton("Add to Data",
        [&](){
            double distance = shuff_.GetNumber("Distance", 0.0);
            double vel = shuff_.GetNumber("Vel", 0.0);
            double pivot = shuff_.GetNumber("Pivot", 0.0);
            shootData_.insert({distance, {vel, pivot}});
            std::cout<<"Added"<<" distance:"<<distance<<" vel:"<<vel<<" pivot:"<<pivot<<std::endl;
        }, {1,1,4,2});

    //KSpin (row 3)
    shuff_.add("kSpin", &kSpin_, {1,1,0,3}, true);
}

void Shooter::CoreShuffleboardPeriodic(){
    shuff_.PutString("State", StateToString(state_));

    shuff_.update(true);
}