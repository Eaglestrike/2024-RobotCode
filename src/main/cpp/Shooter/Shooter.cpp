#include "Shooter/Shooter.h"
#include "Util/SideHelper.h"
#include "Util/Utils.h"

#include "DebugConfig.h"

#define SHOOT_WHILE_MOVE true

Shooter::Shooter(std::string name, bool enabled, bool shuffleboard):
    Mechanism{name, enabled, shuffleboard},
    state_{STOP}, autoStroll_{true},
    bflywheel_{ShooterConstants::BOTTOM_FLYWHEEL, enabled, DebugConfig::SHOOTER.LEFT_FLY},
    tflywheel_{ShooterConstants::TOP_FLYWHEEL, enabled, DebugConfig::SHOOTER.RIGHT_FLY},
    pivot_{"Pivot", enabled, DebugConfig::SHOOTER.PIVOT},
    shuff_{name, shuffleboard}

    #if PIVOT_AUTO_TUNE
    ,pivotTuning_{false},
    pivotTuner_{"pivot tuner", FFAutotuner::ARM, ShooterConstants::PIVOT_MIN, ShooterConstants::PIVOT_MAX}
    #endif
{}

/**
 * Core functions
*/

void Shooter::CoreInit(){
    bflywheel_.Init();
    tflywheel_.Init();
    pivot_.Init();
}

void Shooter::CorePeriodic(){
    #if SHOOTER_AUTO_TUNE
    rflyTuner_.ShuffleboardUpdate();
    lflyTuner_.ShuffleboardUpdate();
    pivotTuner_.ShuffleboardUpdate();
    #endif

    bflywheel_.Periodic();
    tflywheel_.Periodic();
    pivot_.Periodic();
}

void Shooter::CoreTeleopPeriodic(){
    switch(state_){
        case STOP:
            break;
        case SHOOT:
            [[fallthrough]]
        case FERRY:
            if(autoStroll_ && (!hasPiece_) && (Utils::GetCurTimeS() - timerStart_ > shootTime_)){
                hasShot_ = false;
                Stroll(); //Stroll after shooting (not seeing piece for some time)
            }
            break;
        case AMP:
            break;
        case STROLL:
            // Stroll();
            break;
        case MANUAL_TARGET:
            break;
        case EJECT:
            break;
        case THROW:
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

    bflywheel_.TeleopPeriodic();
    tflywheel_.TeleopPeriodic();
    pivot_.TeleopPeriodic();
}

/**
 * Turns off (no voltage)
 * 
 * (only used for testing)
*/
void Shooter::Stop(){
    bflywheel_.Stop();
    tflywheel_.Stop();
    pivot_.Stop();
    state_ = STOP;
}

/**
 * Exits the current state (STOP)
*/
void Shooter::ExitState(){
    state_ = STOP;
}

/**
 * Sets to low speed/voltage to constantly spin
*/
void Shooter::Stroll(){
    if((state_ == AMP) && hasPiece_){ //Don't exit amp
        return;
    }

    bool blueSpeaker = SideHelper::IsBlue();
    vec::Vector2D speaker = blueSpeaker? ShooterConstants::BLUE_SPEAKER : ShooterConstants::RED_SPEAKER;
    vec::Vector2D trim{trim_.y(), trim_.x()};
    trim *= (blueSpeaker? 1.0: -1.0);

    vec::Vector2D toSpeaker = speaker - targetPos_ + trim;

    double dist = toSpeaker.magn();

    // pivot_.SetAngle(ShooterConstants::PIVOT_MIN);
    // pivot_.SetTolerance(0.02);
    pivot_.Stop();

    if(dist < 6.0 && hasPiece_){
        bflywheel_.SetTarget(15.0);
        tflywheel_.SetTarget(15.0);
    }
    else{
        bflywheel_.SetTarget(strollSpeed_);
        tflywheel_.SetTarget(strollSpeed_);
    }
    state_ = STROLL;
}

void Shooter::Amp(){
    if(state_ == MANUAL_TARGET){ //Don't exit manual when called
        return;
    }
    if(!hasPiece_){
        return;
    }
    pivot_.SetTolerance(0.04);
    SetUp(ShooterConstants::FLYWHEEL_SPEED_AMP, ShooterConstants::FLYWHEEL_SPIN_AMP, ShooterConstants::PIVOT_AMP);
    state_ = AMP;
}

/**
 * Sets the pivot to a position
 * 
 * also reverses flywheels if not ejecting
*/
void Shooter::ManualTarget(double target){
    pivot_.SetAngle(target);
    
    if(state_ != EJECT){
        bflywheel_.SetVoltage(-strollSpeed_);
        tflywheel_.SetVoltage(-strollSpeed_);
        
        state_ = MANUAL_TARGET;
    }
}

/**
 * Ejects the game piece at low speed
 * 
 * only controls the flywheels
*/
void Shooter::Eject(){
    pivot_.SetAngle(ShooterConstants::PIVOT_MIN);
    pivot_.SetTolerance(ShooterConstants::SHOOT_POS_TOL);

    bflywheel_.SetVoltage(ejectSpeed_);
    tflywheel_.SetVoltage(ejectSpeed_);

    state_ = EJECT;
}

/**
 * Shoots low
*/
void Shooter::Throw() {
    pivot_.SetAngle(ShooterConstants::PIVOT_MIN);
    pivot_.SetTolerance(ShooterConstants::FERRY_POS_TOL);

    bflywheel_.SetVoltage(shootAmpSpeed_);
    tflywheel_.SetVoltage(shootAmpSpeed_);
    state_ = THROW;
}

/**
 * pivot at target
*/
bool Shooter::PivotAtTarget() {
    return pivot_.AtTarget();
}

/**
 * zero rel to abs
*/
void Shooter::ZeroRelative() {
    pivot_.ZeroRelative();
}

/**
 * Prepares the shot to the speaker
 * 
 * @param toSpeaker field-oriented vector to the speaker 
*/
void Shooter::Prepare(vec::Vector2D robotPos, vec::Vector2D robotVel, bool needGamePiece){
    if(state_ == MANUAL_TARGET){ //Don't exit manual when called
        return;
    }
    if((!hasPiece_) && needGamePiece){
        return;
    }

    state_ = SHOOT;
    targetPos_ = robotPos;

    //Speaker targetting
    bool blueSpeaker = SideHelper::IsBlue();
    vec::Vector2D speaker = blueSpeaker? ShooterConstants::BLUE_SPEAKER : ShooterConstants::RED_SPEAKER;
    vec::Vector2D trim{trim_.y(), trim_.x()};
    trim *= (blueSpeaker? 1.0: -1.0);
    
    vec::Vector2D toSpeaker = speaker - targetPos_ + trim;

    #if SHOOT_WHILE_MOVE
    // Shooting while moving (modify speaker location)
    // https://www.desmos.com/calculator/5hd2snnrwz
    targetVel_ = robotVel;
    double px = toSpeaker.x();
    double py = toSpeaker.y();

    double vx = -robotVel.x();
    double vy = -robotVel.y();    

    double posSquare = px*px + py*py;
    double dotPosVel = px*vx + py*vy;
    double velSquare = vx*vx + vy*vy;

    double a = kD_*kD_ * velSquare - 1.0;
    double b = 2.0*kD_*(dotPosVel + cT_*velSquare);
    double c = posSquare + 2.0*cT_ + cT_*cT_*velSquare;

    double determinant = b*b - 4*a*c;
    if((determinant < 0.0) || (a == 0.0)){
        Stroll();
        return;
    }

    double dist = (-b-std::sqrt(determinant))/(2.0*a);
    double t = kD_*dist + cT_;

    if (Utils::NearZero(robotVel, 0.15)) {
        dist = toSpeaker.magn();
    }

    toSpeaker -= (robotVel*t);
    #else
    double dist = toSpeaker.magn();

    targetVel_ = {0.0, 0.0};
    #endif

    if(shuff_.isEnabled()){
        shuff_.PutNumber("Shot dist", dist, {1,1,2,3});
    }

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

    targetYaw_ += shootYawOffset_;

    // targetYaw_ = (2 * targetYaw_ + posYawTol_ + negYawTol_) / 2;
    // posYawTol_ = (posYawTol_ - negYawTol_) / 2;
    // negYawTol_ = -posYawTol_;

    auto shot = shootData_.lower_bound(dist);
    if((shot == shootData_.begin()) || (shot == shootData_.end())){ //No shot in data (too far or too close)
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

    double upperPercent = (dist - lowerDist) / (upperDist - lowerDist);
    double lowerPercent = 1.0 - upperPercent;

    double pivotAng = lowerPercent*lowerShot.ang + upperPercent*upperShot.ang;
    double shotVel = lowerPercent*lowerShot.vel + upperPercent*upperShot.vel;

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
    //double spin = -angToSpeaker * kSpin_; //Spin opposite to way pointing

    autoStroll_ = needGamePiece;
    SetUp(shotVel, 0.0, pivotAng);
}

void Shooter::Ferry(vec::Vector2D robotPos, vec::Vector2D robotVel){
    if(state_ == MANUAL_TARGET){ //Don't exit manual when called
        return;
    }
    // if(!hasPiece_){
    //     return;
    // }

    //Speaker targetting
    bool blue = SideHelper::IsBlue();
    vec::Vector2D corner = blue? ShooterConstants::BLUE_CORNER : ShooterConstants::RED_CORNER;
    
    targetPos_ = robotPos;
    targetVel_ = {0.0, 0.0};

    vec::Vector2D toAmp = corner - targetPos_;
    double dist = toAmp.magn();

    if(shuff_.isEnabled()){
        shuff_.PutNumber("Shot dist", dist, {1,1,2,3});
    }

    targetYaw_ = toAmp.angle() + shootYawOffset_;

    double angTol = std::atan2(ferryR_, dist);
    posYawTol_ = angTol;
    negYawTol_ = -angTol;

    auto shot = ferryData_.lower_bound(dist);
    if((shot == ferryData_.begin())){ //No shot in data (too far or too close)
        //Check functionality (maybe idle at some distance)
        Stroll();
        return;
    }
    double pivotAng;
    double shotVel;
    if(shot == ferryData_.end()){
        pivotAng = 0.9;
        shotVel = 16.0;
    }
    else{
        //Interpolate between nearby data points
        double upperDist = shot->first;
        ShooterConstants::ShootConfig upperShot = shot->second;
        shot--;
        double lowerDist = shot->first;
        ShooterConstants::ShootConfig lowerShot = shot->second;

        double upperPercent = (dist - lowerDist) / (upperDist - lowerDist);
        double lowerPercent = 1.0 - upperPercent;

        pivotAng = lowerPercent*lowerShot.ang + upperPercent*upperShot.ang;
        shotVel = lowerPercent*lowerShot.vel + upperPercent*upperShot.vel;
    }

    pivot_.SetTolerance(ShooterConstants::PIVOT_POS_TOL);

    //Add spin
    double angToSpeaker = targetYaw_;
    if(angToSpeaker > M_PI/2.0){
        angToSpeaker -= M_PI;
    }
    if(angToSpeaker < -M_PI/2.0){
        angToSpeaker += M_PI;
    }
    
    autoStroll_ = true;
    SetUp(shotVel, 0.0, pivotAng);

    state_ = FERRY;
}

/**
 * Sets if there is a game piece loaded into the shooter (beambreak)
*/
void Shooter::SetGamepiece(bool hasPiece){
    hasPiece_ = hasPiece;
    if(hasPiece){
        timerStart_ = Utils::GetCurTimeS(); //Zero timer whenever have piece (timer starts when switches off)
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

    bflywheel_.SetTarget(vel - spin);
    tflywheel_.SetTarget(vel + spin);
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
bool Shooter::CanShoot(int posVal){
    bool canShoot = false;
    switch(state_){
        case SHOOT:
            [[falllthrough]];
        case FERRY:
        {
            double posError = (targetPos_ - robotPos_).magn();
            double velError = (targetVel_ - robotVel_).magn();
            double yawError = Utils::NormalizeAng(targetYaw_ - robotYaw_);

            if (state_ == FERRY) {
                velError = 0;
            }

            if ((posVal != 0)) {
                posError = 0;
                velError = 0;
                if ((posVal == 3)) {
                    yawError = 0;
                }
            }

            bool yawGood = (yawError < posYawTol_*shootYawPercent_) && (yawError > negYawTol_ * shootYawPercent_);
            canShoot = (posError < posTol_) && (velError < velTol_) && yawGood;

            if (posVal == 3) {
                canShoot = true;
            }

            if(shuff_.isEnabled()){
                shuff_.PutNumber("Pos Error", posError, {1,1,8,2});
                shuff_.PutNumber("Vel Error", velError, {1,1,9,2});
                shuff_.PutNumber("Yaw Error", yawError, {1,1,10,2});
                
                shuff_.PutBoolean("lfly at targ", bflywheel_.AtTarget(), {1,1,8,3});
                shuff_.PutBoolean("rfly at targ", tflywheel_.AtTarget(), {1,1,9,3});
                shuff_.PutBoolean("pivot at targ", pivot_.AtTarget(), {1,1,10,3});
            }
            canShoot = canShoot && bflywheel_.AtTarget() && tflywheel_.AtTarget() && pivot_.AtTarget();
            break;
        }
        case AMP:
        {
            canShoot = bflywheel_.AtTarget() && tflywheel_.AtTarget() && pivot_.AtTarget();
            break;
        }
        case EJECT:
        {
            canShoot = pivot_.AtTarget();
            break;
        }
        case THROW:
        {
            canShoot = pivot_.AtTarget();
            break;
        }
    }

    //Return if everything's prepared
    return canShoot;
}

bool Shooter::UseAutoLineup(){
    double yawError = Utils::NormalizeAng(targetYaw_ - robotYaw_);

    return (yawError > posYawTol_*lineupYawPercent_) || (yawError < negYawTol_*lineupYawPercent_);
}

/**
 * Returns ideal robot angle
*/
double Shooter::GetTargetRobotYaw(){
    return targetYaw_;
}

/**
 * get trim
*/
vec::Vector2D Shooter::GetTrim() {
    return trim_;
}

/**
 * gets if manual
*/
bool Shooter::IsManual() {
    return state_ == MANUAL_TARGET;
}

/**
 * Debug odometry
*/
void Shooter::SetOdometry(vec::Vector2D robotPos, vec::Vector2D robotVel, double robotYaw){
    robotPos_ = robotPos;
    robotVel_ = robotVel;
    robotYaw_ = robotYaw;
}

void Shooter::SetHooked(bool hooked){
    pivot_.SetHooked(hooked);
}

void Shooter::SetNavX(AHRS* navx){
    pivot_.SetNavX(navx);
}

void Shooter::Log(FRCLogger &logger) {
    logger.LogNum("Shot Vel", shot_.vel);
    logger.LogNum("Shot Ang", shot_.ang);
    logger.LogStr("Pivot state", pivot_.GetStateStr());
    logger.LogStr("Top Flywheel state", tflywheel_.GetStateStr());
    logger.LogStr("Bottom flywheel state", bflywheel_.GetStateStr());
    logger.LogStr("Shooter state", StateToString(state_));
    logger.LogBool("Can shoot", CanShoot());
    logger.LogNum("Pivot tol", pivot_.GetTolerance());
    logger.LogNum("Pivot pos", pivot_.GetPose().pos);
    logger.LogNum("Pivot vel", pivot_.GetPose().vel);
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
        case SHOOT: return "Shoot";
        case FERRY: return "Ferry";
        case AMP: return "Amp";
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
    shuff_.add("Eject Speed", &ejectSpeed_, {2,1,0,1}, true);
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
            autoStroll_ = false;
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
    // shuff_.add("Shoot time", &shootTime_, {1,1,5,2}, true);

    //Shot data (row 3)
    //shuff_.add("kSpin", &kSpin_, {1,1,0,3}, true);
    shuff_.add("Shot Vel", &shot_.vel, {1,1,0,3});
    shuff_.add("Shot Ang", &shot_.ang, {1,1,1,3});
    shuff_.add("kD", &kD_, {1,1,4,3}, true);
    shuff_.add("cT", &cT_, {1,1,5,3}, true);
    shuff_.add("Yaw Offset", &shootYawOffset_, {1,1,6,3}, true);
    
    //Tolerance (row 4)
    shuff_.add("pos tol", &posTol_, {1,1,0,4}, true);
    shuff_.add("vel tol", &velTol_, {1,1,1,4}, true);
    shuff_.add("yaw percent", &shootYawPercent_, {1,1,2,4}, true);
    shuff_.add("lineup percent", &lineupYawPercent_, {1,1,3,4}, true);
    shuff_.add("pivot percent", &pivotAngPercent_, {1,1,4,4}, true);

    shuff_.add("yaw pos", &posYawTol_, {1,1,6,4}, false);
    shuff_.add("yaw neg", &negYawTol_, {1,1,7,4}, false);    
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
    
    shuff_.PutBoolean("Can Shoot", CanShoot(0));

    bool blueSpeaker = SideHelper::IsBlue();
    vec::Vector2D speaker = blueSpeaker? ShooterConstants::BLUE_SPEAKER : ShooterConstants::RED_SPEAKER;
    vec::Vector2D trim{trim_.y(), trim_.x()};
    trim *= (blueSpeaker? 1.0: -1.0);
    vec::Vector2D toSpeaker = speaker - targetPos_ + trim;
    shuff_.PutNumber("Distance", toSpeaker.magn());
    
    shuff_.update(true);

    #if PIVOT_AUTO_TUNE
    pivotTuner_.ShuffleboardUpdate();
    #endif
}