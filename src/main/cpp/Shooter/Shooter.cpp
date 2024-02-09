#include "Shooter/Shooter.h"
#include "Util/SideHelper.h"

Shooter::Shooter(std::string name, bool enabled, bool shuffleboard):
    Mechanism{name, enabled, shuffleboard},
    state_{STOP},
    lflywheel_{ShooterConstants::LEFT_FLYWHEEL, enabled, shuffleboard},
    rflywheel_{ShooterConstants::RIGHT_FLYWHEEL, enabled, shuffleboard},
    pivot_{"Pivot", enabled, false},
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
        case LOADPIECE:
            if(hasPiece_){
                state_ = SHOOT;
            }
            break;
        case SHOOT:
            if((Utils::GetCurTimeS() - shootTimer_ > shootTimer_) && (!hasPiece_)){
                Stroll(); //Stroll after shooting (not seeing piece for some time)
            }
            break;
        case STROLL:
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
 * Sets pivot down to intake into shooter
*/
void Shooter::BringDown(){
    pivot_.SetAngle(pivotIntake_);
}

/**
 * Prepares the shot to the speaker
 * 
 * @param toSpeaker field-oriented vector to the speaker 
*/
void Shooter::Prepare(vec::Vector2D robotPos, vec::Vector2D robotVel, bool blueSpeaker){
    targetPos_ = robotPos;
    targetVel_ = robotVel;

    vec::Vector2D toSpeaker;
    if(blueSpeaker){
        toSpeaker = ShooterConstants::BLUE_SPEAKER - targetPos_;
    }
    else{
        toSpeaker = ShooterConstants::RED_SPEAKER - targetPos_;
    }

    double dist = toSpeaker.magn();
    targetYaw_ = toSpeaker.angle();

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
    double angToSpeaker = targetYaw_;
    if(angToSpeaker > M_PI/2.0){
        angToSpeaker -= M_PI;
    }
    if(angToSpeaker < -M_PI/2.0){
        angToSpeaker += M_PI;
    }
    double spin = -angToSpeaker * kSpin_; //Spin opposite to way pointing

    if(!hasPiece_){
        BringDown();
        return;
    }
    SetUp(shotVel, spin, pivotAng);
}

/**
 * Sets if there is a game piece loaded into the shooter (beambreak)
*/
void Shooter::SetGamepiece(bool hasPiece){
    hasPiece_ = hasPiece;
    if(hasPiece){
        shootTimer_ = Utils::GetCurTimeS(); //Zero timer whenever have piece (timer starts when switches off)
    }
}

/**
 * Internal method to shoot at vel, spin and pivot angle
*/
void Shooter::SetUp(double vel, double spin, double pivot){
    shot_.vel = vel;
    shot_.ang = pivot;
    spin_ = spin;

    lflywheel_.SetTarget(vel - spin);
    rflywheel_.SetTarget(vel + spin);
    pivot_.SetAngle(pivot);

    state_ = SHOOT;
    hasShot_ = true;
}

/**
 * Calculates the forward kinematics of the shot
 * 
 * Returns if it will make the shot and how much off
*/
Shooter::FKRes Shooter::CalculateForwardKinematics(vec::Vector2D robotPos, vec::Vector2D robotVel, double robotYaw, vec::Vector2D target, ShooterConstants::ShootConfig shot){
    const FKRes miss{
        .score = false,
        .aimed = false,
        .error = ShooterConstants::ABSOLUTE_MISS
    };

    FKRes res;
    vec::Vector2D toTarget = target - robotPos;

    vec::Vector2D shotVel{cos(robotYaw), sin(robotYaw)};
    shotVel *= shot.vel * cos(shot.ang);
    shotVel += robotVel;

    if(shotVel.x() == 0.0){
        return miss;
    }

    double t = toTarget.x()/shotVel.x();
    if(t < 0.0){
        return miss;
    }

    res.error.x(toTarget.y() - (shotVel.y() * t)); // error = target - v*t

    double z = -9.81/2.0*t*t + shot.vel*sin(shot.ang)*t + ShooterConstants::SHOOTER_HEIGHT;
    res.error.y(ShooterConstants::SPEAKER_CENTER - z);

    res.score = (abs(res.error.x()) < ShooterConstants::SPEAKER_WIDTH/2.0) && 
                (abs(res.error.y()) < ShooterConstants::SPEAKER_HEIGHT/2.0); //If it makes it into the box
    res.aimed  = true;
    return res;
}

/**
 * Calculate inverse kinematics of a shot
 * 
 * target -> shot
*/
Shooter::IKRes Shooter::CalculateInverseKinematics(vec::Vector2D target){ 
    IKRes res;

    return res;
}

/**
 * Returns if you can shoot
*/
bool Shooter::CanShoot(vec::Vector2D robotPos, vec::Vector2D robotVel, double robotYaw){
    if(state_ != SHOOT){
        return false;
    }
    //Can only shoot within target
    if((robotPos - targetPos_).magn() < posTol_){
        return false;
    }
    if((robotVel - targetVel_).magn() < velTol_){
        return false;
    }
    double yawError = std::fmod(robotYaw - targetYaw_, 2.0*M_PI);
    if(yawError > M_PI){
        yawError -= 2.0*M_PI;
    }
    if(yawError < -M_PI){
        yawError += 2.0*M_PI;
    }
    if(std::abs(yawError) < yawTol_){
        return false;
    }
    //Return if everything's prepared
    return lflywheel_.AtTarget() && rflywheel_.AtTarget() && pivot_.AtTarget();
}

/**
 * Returns ideal robot angle
*/
double Shooter::GetTargetRobotYaw(){
    return targetYaw_;
}

/**
 * Debug odometry
*/
void Shooter::SetOdometry(vec::Vector2D robotPos, vec::Vector2D robotVel, double robotYaw){
    robotPos_ = robotPos;
    robotVel_ = robotVel;
    robotYaw_ = robotYaw;
}

std::string Shooter::StateToString(State state){
    switch(state){
        case STOP: return "Stop";
        case LOADPIECE: return "Load Piece";
        case SHOOT: return "Shoot";
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
    shuff_.add("Has piece", &hasPiece_, {2,1,4,1}, false);
    shuff_.PutBoolean("Can Shoot", false, {1,1,5,0});

    // Kinematics
    // shuff_.PutBoolean("FK make it", false, {1,1,6,0});
    // shuff_.PutBoolean("Aimed", false, {1,1,7,0});
    // shuff_.PutString("Shot Error", "", {2,1,6,1});

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
            std::cout<<"Added distance:"<<distance<<" vel:"<<vel<<" pivot:"<<pivot<<std::endl;
        }, {1,1,4,2});
    shuff_.add("Shoot timer", &shootTimer_, {1,1,5,2}, true);

    //KSpin (row 3)
    shuff_.add("kSpin", &kSpin_, {1,1,0,3}, true);

    //Tolerance (row 4)
    shuff_.add("pos tol", &posTol_, {1,1,0,4}, true);
    shuff_.add("vel tol", &velTol_, {1,1,1,4}, true);
    shuff_.add("yaw tol", &yawTol_, {1,1,2,4}, true);

}

void Shooter::CoreShuffleboardPeriodic(){
    shuff_.PutString("State", StateToString(state_));
    shuff_.PutBoolean("Can Shoot", CanShoot(robotPos_, robotVel_, robotYaw_));

    //Kinematics
    // vec::Vector2D targ = SideHelper::IsBlue? ShooterConstants::BLUE_SPEAKER : ShooterConstants::RED_SPEAKER;
    // FKRes fk = CalculateForwardKinematics(robotPos_, robotVel_, robotYaw_, targ, shot_);
    // shuff_.PutBoolean("FK make it", fk.score);
    // shuff_.PutBoolean("Aimed", fk.aimed);
    // shuff_.PutString("Shot Error", fk.error.toString());

    shuff_.update(true);
}