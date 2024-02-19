#include "Shooter/Shooter.h"
#include "Util/SideHelper.h"

Shooter::Shooter(std::string name, bool enabled, bool shuffleboard):
    Mechanism{name, enabled, shuffleboard},
    state_{STOP},
    lflywheel_{ShooterConstants::LEFT_FLYWHEEL, enabled, shuffleboard},
    rflywheel_{ShooterConstants::RIGHT_FLYWHEEL, enabled, shuffleboard},
    pivot_{"Pivot", enabled, shuffleboard},
    shuff_{name, shuffleboard}

    #if PIVOT_AUTO_TUNE
    ,pivotTuning_{false},
    pivotTuner_{"pivot tuner", FFAutotuner::ARM, ShooterConstants::PIVOT_MIN, ShooterConstants::PIVOT_MAX}
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
            if(hasPiece_ && hasShot_){
                SetUp(shot_.vel, spin_, shot_.ang); //Reload
            }
            break;
        case SHOOT:
            if((!hasPiece_) && (Utils::GetCurTimeS() - shootTimer_ > shootTimer_)){
                hasShot_ = false;
                Stroll(); //Stroll after shooting (not seeing piece for some time)
            }
            break;
        case STROLL:
            Stroll();
            break;
        default:
            break;
    }

    #if PIVOT_AUTO_TUNE
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
    if(!hasPiece_){
        BringDown();
        return;
    }

    vec::Vector2D toSpeaker;
    if(SideHelper::IsBlue()){
        toSpeaker = ShooterConstants::BLUE_SPEAKER - robotPos_;
    }
    else{
        toSpeaker = ShooterConstants::RED_SPEAKER - robotPos_;
    }

    double dist = toSpeaker.magn();

    pivot_.SetAngle(0.7);
    if(dist < 6.0){
        lflywheel_.SetTarget(15.0);
        rflywheel_.SetTarget(15.0);
    }
    else{
        lflywheel_.SetVoltage(strollSpeed_);
        rflywheel_.SetVoltage(strollSpeed_);
    }
    //pivot_.Stop();
    state_ = STROLL;
}

/**
 * Sets pivot down to intake into shooter
*/
void Shooter::BringDown(){
    pivot_.SetAngle(pivotIntake_);
    
    lflywheel_.SetVoltage(strollSpeed_);
    rflywheel_.SetVoltage(strollSpeed_);  
}

/**
 * Prepares the shot to the speaker
 * 
 * @param toSpeaker field-oriented vector to the speaker 
*/
void Shooter::Prepare(vec::Vector2D robotPos, vec::Vector2D robotVel, bool blueSpeaker){    
    targetPos_ = robotPos;
    targetVel_ = {0.0, 0.0};
    //targetVel_ = robotVel;

    //Speaker targetting
    vec::Vector2D speaker = blueSpeaker? ShooterConstants::BLUE_SPEAKER : ShooterConstants::RED_SPEAKER;
    vec::Vector2D trim{0.0, trim_.x() * (blueSpeaker? 1.0: -1.0)};

    vec::Vector2D toSpeaker = speaker - targetPos_ + trim;

    double dist = toSpeaker.magn();
    targetYaw_ = toSpeaker.angle();

    const std::vector<vec::Vector2D>& speakerBox = blueSpeaker? ShooterConstants::BLUE_SPEAKER_BOX : ShooterConstants::RED_SPEAKER_BOX;
    posYawTol_ = 0.0;
    negYawTol_ = 0.0;
    //Find the min/max angles to shoot into the box
    for(const vec::Vector2D boxPoint : speakerBox){
        vec::Vector2D toVertex = boxPoint - targetPos_ + trim;
        double vertexAng = toVertex.angle();

        double yawDiff = Utils::NormalizeAng(vertexAng - targetYaw_);
        if(yawDiff > posYawTol_){
            posYawTol_ = yawDiff;
        }
        else if(yawDiff < negYawTol_){
            negYawTol_ = yawDiff;
        }
    }
    //Multiply by tolerance percent
    posYawTol_ = std::clamp(posYawTol_, 0.01, M_PI/2.0); //Tol cannot be greater than 90 degrees
    negYawTol_ = std::clamp(negYawTol_, -M_PI/2.0, -0.01);

    // Shooting while moving
    // https://www.desmos.com/calculator/5hd2snnrwz

    /**
    double px = toSpeaker.x();
    double py = toSpeaker.y();

    double vx = -robotVel.x();
    double vy = -robotVel.y();    

    double posSquare = px*px + py*py;
    double dotPosVel = px*vx + py*vy;
    double velSquare = vx*vx + vy*vy;

    const double kD = ShooterConstants::kD;
    const double cT = ShooterConstants::cT;

    double a = kD*kD * velSquare - 1.0;
    double b = 2.0*kD*(dotPosVel + cT*velSquare);
    double c = posSquare + 2.0*cT + cT*cT*velSquare;

    double determinant = b*b - 4*a*c;
    if((determinant < 0.0) || (a == 0.0)){
        std::cout << "Shot not possible" << std::endl;
        Stroll();
        return;
    }

    double dist = (-b-std::sqrt(determinant))/(2.0*a);
    double t = kD*dist + cT;

    targetYaw_ = static_cast<vec::Vector2D>((toSpeaker - (robotVel*t))).angle();
    **/

    if(!hasPiece_){
        Stroll();
        return;
    }

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

    double percent = (dist - lowerDist) / (upperDist - lowerDist);
    double upperPercent = 1.0 - percent;

    double pivotAng = percent*lowerShot.ang + upperPercent*upperShot.ang;
    double shotVel = percent*lowerShot.vel + upperPercent*upperShot.vel;

    //Pivot set tolerance (max - min)/2.0
    double pivotTol = (std::atan(dist/ShooterConstants::SPEAKER_MIN) - std::atan((dist-ShooterConstants::SPEAKER_DEPTH)/ShooterConstants::SPEAKER_MIN))/2.0;
    pivot_.SetTolerance(pivotTol * pivotAngPercent_);

    //Add spin
    double angToSpeaker = targetYaw_;
    if(angToSpeaker > M_PI/2.0){
        angToSpeaker -= M_PI;
    }
    if(angToSpeaker < -M_PI/2.0){
        angToSpeaker += M_PI;
    }
    double spin = -angToSpeaker * kSpin_; //Spin opposite to way pointing

    SetUp(shotVel, spin, pivotAng);
    if(!hasPiece_){
        BringDown();
    }
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
 * 
 * Sets state to SHOOT
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
 * Shifts target by some distance (along y axis)
*/
void Shooter::Trim(vec::Vector2D trim){
    trim_ += trim;
}

/**
 * Returns if you can shoot
*/
bool Shooter::CanShoot(){
    vec::Vector2D toSpeaker;
    if(SideHelper::IsBlue()){
        toSpeaker = ShooterConstants::BLUE_SPEAKER - robotPos_;
    }
    else{
        toSpeaker = ShooterConstants::RED_SPEAKER - robotPos_;
    }

    double dist = toSpeaker.magn();

    if(state_ != SHOOT){
        if(shuff_.isEnabled()){
            shuff_.PutBoolean("Can Shoot", false);
        }
        return false;
    }
    //Can only shoot within target
    double posError = (targetPos_ - robotPos_).magn();
    double velError = (targetVel_ - robotVel_).magn();
    double yawError = Utils::NormalizeAng(targetYaw_ - robotYaw_);

    bool yawGood = (yawError < posYawTol_*shootYawPercent_) && (yawError > negYawTol_ * shootYawPercent_);
    bool canShoot = (posError < posTol_) && (velError < velTol_) && yawGood;
    if(shuff_.isEnabled()){
        shuff_.PutNumber("Pos Error", posError, {1,1,8,2});
        shuff_.PutNumber("Vel Error", velError, {1,1,9,2});
        shuff_.PutNumber("Yaw Error", yawError, {1,1,10,2});
        
        shuff_.PutBoolean("lfly at targ", lflywheel_.AtTarget(), {1,1,8,3});
        shuff_.PutBoolean("rfly at targ", rflywheel_.AtTarget(), {1,1,9,3});
        shuff_.PutBoolean("pivot at targ", pivot_.AtTarget(), {1,1,10,3});

        shuff_.PutBoolean("Can Shoot", canShoot);
    }
    //Return if everything's prepared
    return canShoot && lflywheel_.AtTarget() && rflywheel_.AtTarget() && pivot_.AtTarget();
}

bool Shooter::ShouldAutoLineup(){
    double yawError = Utils::NormalizeAng(targetYaw_ - robotYaw_);

    return (yawError < posYawTol_*lineupYawPercent_) && (yawError > negYawTol_*lineupYawPercent_);
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
    shuff_.PutString("State", StateToString(state_), {2,1,3,0});
    shuff_.add("Has piece", &hasPiece_, {2,1,3,1}, false);
    shuff_.PutBoolean("Can Shoot", false, {1,1,5,0});

    // Kinematics
    // shuff_.PutBoolean("FK make it", false, {1,1,6,0});
    // shuff_.PutBoolean("Aimed", false, {1,1,7,0});
    // shuff_.PutString("Shot Error", "", {2,1,6,1});

    #if PIVOT_AUTO_TUNE
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
    // shuff_.addButton("Add to Data",
    //     [&](){
    //         double distance = shuff_.GetNumber("Distance", 0.0);
    //         double vel = shuff_.GetNumber("Vel", 0.0);
    //         double pivot = shuff_.GetNumber("Pivot", 0.0);
    //         shootData_.insert_or_assign({distance, {vel, pivot}});
    //         std::cout<<"Added distance:"<<distance<<" vel:"<<vel<<" pivot:"<<pivot<<std::endl;
    //     }, {1,1,4,2});
    shuff_.add("Shoot timer", &shootTimer_, {1,1,5,2}, true);

    //Shot data (row 3)
    shuff_.add("kSpin", &kSpin_, {1,1,0,3}, true);
    shuff_.add("Shot Vel", &shot_.vel, {1,1,1,3});
    shuff_.add("Shot Ang", &shot_.ang, {1,1,2,3});
    

    //Tolerance (row 4)
    shuff_.add("pos tol", &posTol_, {1,1,0,4}, true);
    shuff_.add("vel tol", &velTol_, {1,1,1,4}, true);
    shuff_.add("yaw percent", &shootYawPercent_, {1,1,2,4}, true);
    shuff_.add("lineup percent", &lineupYawPercent_, {1,1,3,4}, true);
    shuff_.add("pivot percent", &pivotAngPercent_, {1,1,3,4}, true);

    
    shuff_.add("yaw pos", &posYawTol_, {1,1,3,4}, false);
    shuff_.add("yaw neg", &negYawTol_, {1,1,4,4}, false);    
}

void Shooter::CoreShuffleboardPeriodic(){
    shuff_.PutString("State", StateToString(state_));
    //shuff_.PutBoolean("Can Shoot", CanShoot(robotPos_, robotVel_, robotYaw_));

    //Kinematics
    // vec::Vector2D targ = SideHelper::IsBlue? ShooterConstants::BLUE_SPEAKER : ShooterConstants::RED_SPEAKER;
    // FKRes fk = CalculateForwardKinematics(robotPos_, robotVel_, robotYaw_, targ, shot_);
    // shuff_.PutBoolean("FK make it", fk.score);
    // shuff_.PutBoolean("Aimed", fk.aimed);
    // shuff_.PutString("Shot Error", fk.error.toString());

    shuff_.update(true);

    #if PIVOT_AUTO_TUNE
    pivotTuner_.ShuffleboardUpdate();
    #endif
}