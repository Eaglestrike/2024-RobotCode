#include "Auto/Auto.h"

#include <string>

#include "Util/Utils.h"
#include "Util/SideHelper.h"

using AutoPath = AutoConstants::AutoPath;
using AutoElement = AutoConstants::AutoElement;
using enum AutoConstants::AutoType;
using enum AutoConstants::AutoAction;

/**
 * Auto class
 * 
 * @note does not call periodic calls of mechanisms
*/
Auto::Auto(bool shuffleboard, SwerveControl &swerve, Odometry &odom, AutoAngLineup &autolineup, Intake &intake, Shooter &shooter):
    segments_{shuffleboard, swerve, odom},
    odometry_{odom},
    autoLineup_{autolineup},
    intake_{intake},
    shooter_{shooter},
    shuff_{"Auto", shuffleboard}
{
    ResetTiming(intakeTiming_);
    ResetTiming(shooterTiming_);
    ResetTiming(driveTiming_);

    //SetPath(0, {{SHOOT, AFTER});
}

/**
 * Sets the path to run at some index
*/
void Auto::SetPath(uint index, AutoPath path){
    for(uint i = paths_.size(); i <= index; i++){
        paths_.push_back({});
    }
    paths_[index] = path;
    LoadPath(path);
}

/**
 * Sets an auto segment to get a piece and shoot
 * 
 * @param to filename of path to gamepiece (ex: to.csv)
 * @param back filename of path back from gamepiece (ex: back.csv)
 * @param index index of the path (starting at 1)
*/
void Auto::SetSegment(uint index, std::string to, std::string back){
    if(index == 0){
        std::cout<<"bad index 0 Auto::SetSegment"<<std::endl;
        return;
    }
    if((to == "") || (back  == "")){ //Check if it's an empty path
        SetPath(index, {});
        return;
    }
    SetPath(
        index,
        {
            {DRIVE, AFTER, to},
            {INTAKE, AT_START}, //Can keep intake down if needed (then use stow)
            {DRIVE, AFTER, back},
            {SHOOT, BEFORE_END},
        }
    );
}

/**
 * Sets an auto segment to get a piece and shoot
 * 
 * @param path filename of drive path (ex: to.csv)
*/
void Auto::SetSegment(uint index, std::string path){
    if(index == 0){
        std::cout<<"bad index 0 Auto::SetSegment"<<std::endl;
        return;
    }
    if(path == ""){ //Check if it's an empty path
        SetPath(index, {});
        return;
    }
    SetPath(
        index,
        {{DRIVE, AFTER, path}}
    );
}

/**
 * Autonomous Init
*/
void Auto::AutoInit(){
    pathNum_ = 0;
    index_ = 0;
    
    //Find starting pos
    segments_.Clear();
    vec::Vector2D startPos = odometry_.GetPos();
    for(int i = 0; i < paths_.size(); i++){
        for(AutoElement element : paths_[i]){
            if(element.action == DRIVE){
                segments_.SetAutoPath(element.data);
                //std::cout<<element.data<<std::endl;
                startPos = segments_.GetPos(0.0);
                //std::cout<<"Starting pos " << startPos.toString() <<std::endl;
                i = paths_.size();
                break;
            }
        }
    }
    if((startPos - odometry_.GetPos()).magn() > 0.5){ //If starting pos too far away
        pathNum_ = 100000; //Give up
    }

    segments_.Clear();

    ResetTiming(channelTiming_);
    ResetTiming(intakeTiming_);
    ResetTiming(shooterTiming_);
    ResetTiming(driveTiming_);

    autoStart_ = Utils::GetCurTimeS();
    
    NextBlock();
    //std::cout << "End Init" <<std::endl;
}

/**
 * Autonomous Periodic
*/
void Auto::AutoPeriodic(){
    //std::cout << "Periodic Start" <<std::endl;
    double t = Utils::GetCurTimeS() - autoStart_;

    if(driveTiming_.finished && shooterTiming_.finished && intakeTiming_.finished){
        //std::cout << "Next Block Periodic" <<std::endl;
        NextBlock();
    }
    
    bool useAngLineup = shooterTiming_.hasStarted && (!shooterTiming_.finished) && intake_.HasGamePiece();
    
    if(useAngLineup && false){
        autoLineup_.SetTarget(shooter_.GetTargetRobotYaw());
        autoLineup_.Start();
        autoLineup_.Periodic();
        segments_.Periodic(autoLineup_.GetAngVel());
    }
    else{
        segments_.Periodic();
    }
    
    DrivePeriodic(t);
    //std::cout<<"drive"<<std::endl;
    ShooterPeriodic(t);
    //std::cout<<"shoot"<<std::endl;
    IntakePeriodic(t);
    //std::cout<<"intake"<<std::endl;
}

/**
 * Execution for drive
*/
void Auto::DrivePeriodic(double t){
    if(!driveTiming_.hasStarted && t > driveTiming_.start){
        segments_.Start();
        driveTiming_.hasStarted = true;
    }

    if(segments_.AtTarget()){
        driveTiming_.finished = true;
    }

    if(t > driveTiming_.end + DRIVE_PADDING){
        driveTiming_.finished = true;
    }
}

/**
 * Execution for shooter
 * 
 * @returns if it needs to autolineup
*/
void Auto::ShooterPeriodic(double t){
    if(!shooterTiming_.hasStarted && t > shooterTiming_.start){
        shooterTiming_.hasStarted = true;
    }

    if(shooterTiming_.finished){
        shooter_.Stroll();
    }
    else if(shooterTiming_.hasStarted){
        vec::Vector2D pos{odometry_.GetPos()};
        //Feed into shooter when can shoot
        if(shooter_.CanShoot() || (t > shooterTiming_.end + SHOOT_PADDING)){ 
            intake_.FeedIntoShooter();
        }

        if(inChannel_ && intake_.InChannel()){
            inChannel_ = false;
        }
        
        //Finish shooting when can't see piece for a bit
        if(!inChannel_ && !intake_.HasGamePiece()){ 
            if(!channelTiming_.hasStarted){ //Start channel timer
                channelTiming_.end = t + CHANNEL_TIME;
                channelTiming_.hasStarted = true;
            }
            else if(t > channelTiming_.end){ 
                shooterTiming_.finished = true;
            }
        }

        if((pos - shootPos_).magn() < SHOOT_POS_TOL){ //Constantly prepare to current position if within some distance to the target
            shooter_.Prepare(pos, odometry_.GetVel(), SideHelper::IsBlue());
        }
        else{
            shooter_.Prepare(shootPos_, {0,0}, SideHelper::IsBlue()); //Prepare for the target shot
        }
        
    }
    else{
        shooter_.Stroll();
    }
}

/**
 * Execution for intake
*/
void Auto::IntakePeriodic(double t){
    //First Action
    if(!intakeTiming_.hasStarted && t > intakeTiming_.start){
        if(intaking_){
            intake_.Passthrough();
        }
        else{
            intake_.Stow();
        }
        intakeTiming_.hasStarted = true;
    }
    //Check if finished
    if(intaking_){
        double maxTime = std::max(intakeTiming_.end, driveTiming_.end);
        if( (intake_.HasGamePiece()) || // End intake if has game piece
            (t > maxTime + INTAKE_PADDING)){ 
            intakeTiming_.finished = true;
            inChannel_ = true;
        }
    }
    else{
        if(t > intakeTiming_.end + STOW_PADDING){
            intakeTiming_.finished = true;
        }
    }
}

/**
 * Sets up the next block (the actions between 2 AFTERS)
*/
void Auto::NextBlock(){
    if(pathNum_ >= (int)paths_.size()){
        return;
    }
    //std::cout<<pathNum_<<" "<<index_<<std::endl;
    AutoPath path = paths_[pathNum_];
    if(index_ >= (int)path.size()){
        pathNum_++;
        index_ = 0;
        NextBlock();
        return;
    }
    AutoElement firstElement = path[index_];
    //std::cout<<"Next block get element"<<std::endl;

    ResetTiming(channelTiming_);
    ResetTiming(intakeTiming_);
    ResetTiming(shooterTiming_);
    ResetTiming(driveTiming_);

    // std::cout<<Utils::GetCurTimeS()<<std::endl;
    // std::cout<<autoStart_<<std::endl;
    // std::cout<<firstElement.offset<<std::endl;
    
    blockStart_ = Utils::GetCurTimeS() - autoStart_ + firstElement.offset;

    //Figure out the block duration/initialize the first element
    switch(firstElement.action){
        case DRIVE:
            segments_.SetAutoPath(firstElement.data);
            blockEnd_ = blockStart_ + segments_.GetDuration();
            break;
        case SHOOT:
            blockEnd_ = blockStart_ + SHOOT_TIME;
            break;
        case INTAKE:
            blockEnd_ = blockStart_ + INTAKE_TIME;
            break;
        case STOW:
            blockEnd_ = blockStart_ + STOW_TIME;
            break;
        default:
            std::cout<<"Did not deal with auto action case NB "<< firstElement.action <<std::endl;
    }
    EvaluateElement(firstElement);
    index_++;

    //Lookahead for execution in this block
    for(;index_ < (int)path.size();index_++){
        AutoElement element = path[index_];
        if(element.type == AFTER){
            break;
        }
        EvaluateElement(element);
    }

    //Get the shooter position target
    if(!shooterTiming_.finished){
        shootPos_ = segments_.GetPos(shooterTiming_.end + autoStart_); 
    }

    if(index_ >= (int)path.size()){//Finished this path
        pathNum_++;
        index_ = 0;
        return;
    }
}

/**
 * Timing for the elements
 * 
 * Returns if it has found the next block
*/
void Auto::EvaluateElement(AutoConstants::AutoElement element){
    switch(element.action){
        case DRIVE:
            EvaluateDriveElement(element);
            return;
        case SHOOT:
            EvaluateShootElement(element);
            return;
        case INTAKE: [[fallthrough]];
        case STOW:
            EvaluateIntakeElement(element);
            return;
        default:
            std::cout<<"Did not deal with auto action case EE "<< element.action <<std::endl;
    }
}

/**
 * Evaluates a drive element
 * Sets the timing for this action
*/
void Auto::EvaluateDriveElement(AutoConstants::AutoElement element){
    //Set timing
    driveTiming_.hasStarted = false;
    driveTiming_.finished = false;
    switch(element.type){
        case AFTER:
            driveTiming_.start = blockStart_;
            driveTiming_.end = blockEnd_;
            return;
        default:
            std::cout<<"Bad case for driving "<< element.data <<std::endl;
    }
}

/**
 * Evaluates a shoot element
 * Sets the timing for this action
*/
void Auto::EvaluateShootElement(AutoConstants::AutoElement element){
    //Set timing
    shooterTiming_.hasStarted = false;
    shooterTiming_.finished = false;
    switch(element.type){
        case AT_START:
            shooterTiming_.start = blockStart_ + element.offset;
            break;
        case BEFORE_END:
            shooterTiming_.start = blockEnd_ - SHOOT_TIME - element.offset;
            break;
        case AFTER:
            shooterTiming_.start = blockStart_;
            break;
        default:
            std::cout<<"forgor deal with case ESE "<< element.action <<std::endl;
    }
    shooterTiming_.end = shooterTiming_.start + SHOOT_TIME;
}

/**
 * Evaluates a intake element
 * Sets the timing for this action
*/
void Auto::EvaluateIntakeElement(AutoConstants::AutoElement element){
    intakeTiming_.hasStarted = false;
    intakeTiming_.finished = false;
    intaking_ = (element.action == INTAKE);
    switch(element.type){
        case AT_START:
            intakeTiming_.start = blockStart_ + element.offset;
            break;
        case BEFORE_END:
            if(intaking_){
                intakeTiming_.start = blockEnd_ - INTAKE_TIME - element.offset;
            }
            else{
                intakeTiming_.start = blockEnd_ - STOW_TIME - element.offset;
            }
            break;
        case AFTER:
            intakeTiming_.start = blockStart_;
            break;
        default:
            std::cout<<"forgor deal with case EIE "<< element.action <<std::endl;
    }
    if(intaking_){
        intakeTiming_.end = intakeTiming_.start + INTAKE_TIME;
    }
    else{
        intakeTiming_.end = intakeTiming_.start + STOW_TIME;
    }
}

/**
 * Sets the timing to be done
*/
void Auto::ResetTiming(Auto::SubsystemTiming& timing){
    timing.start = 0.0;
    timing.end = 0.0;
    timing.finished = true;
    timing.hasStarted = true;
}

/**
 * Reads all paths from file
*/
void Auto::LoadPath(const AutoPath& path){
    for(const AutoElement& elem : path){
        if(elem.action == DRIVE){
            segments_.LoadAutoPath(elem.data);
        }
    }
}

std::string Auto::ElementToString(const AutoElement element){
    std::string str = "";
    switch(element.action){
        case DRIVE:     str += "DRIVE    "; break;
        case INTAKE:    str += "INTAKE   "; break;
        case STOW:      str += "STOW     "; break;
        case SHOOT:     str += "SHOOTING "; break;
        default:        str += "UNKNOWN  ";
    }
    switch(element.type){
        case AFTER:     str += "AFTER     ,"; break;
        case AT_START:  str += "AT_START  ,"; break;
        case BEFORE_END:str += "BEFORE_END,"; break;
        default:        str += "UNKNOWN   ,";
    }
    str += " " + element.data + " ";
    str += std::to_string(element.offset);
    return str;
}

void Auto::ShuffleboardInit(){
    if(!shuff_.isEnabled()){
        return;
    }
    segments_.ShuffleboardInit();
    //Block (row 0)
    shuff_.add("Path", &pathNum_, {1,1,0,0});
    shuff_.add("Index", &index_, {1,1,1,0});
    shuff_.add("Block Start", &blockStart_, {1,1,2,0});
    shuff_.add("Block End", &blockEnd_, {1,1,3,0});
    shuff_.PutNumber("time", 0.0, {1,1,4,0});

    //Drive Timing (row 1)
    shuff_.add("Drive Start", &driveTiming_.start, {1,1,0,1});
    shuff_.add("Drive End", &driveTiming_.end, {1,1,1,1});
    shuff_.add("Drive Has Started", &driveTiming_.hasStarted, {1,1,2,1});
    shuff_.add("Drive Has Finished", &driveTiming_.finished, {1,1,3,1});

    shuff_.add("Drive Padding", &DRIVE_PADDING, {1,1,4,1}, true);

    shuff_.PutNumber("Drive Progress", 0.0, {1,1,5,1});

    //Shooter Timing (row 2)
    shuff_.add("Shooter Start", &shooterTiming_.start, {1,1,0,2});
    shuff_.add("Shooter End", &shooterTiming_.end, {1,1,1,2});
    shuff_.add("Shooter Has Started", &shooterTiming_.hasStarted, {1,1,2,2});
    shuff_.add("Shooter Has Finished", &shooterTiming_.finished, {1,1,3,2});
    
    shuff_.add("Shooter Padding", &SHOOT_PADDING, {1,1,4,2}, true);
    shuff_.add("Shooter Time", &SHOOT_TIME, {1,1,5,2}, true);
    shuff_.add("Shoot pos tol", &SHOOT_POS_TOL, {1,1,6,2}, true);

    shuff_.add("Shooter Padding", &CHANNEL_TIME, {1,1,4,2}, true);

    //Intake Timing (row 3)
    shuff_.add("Intake Start", &intakeTiming_.start, {1,1,0,3});
    shuff_.add("Intake End", &intakeTiming_.end, {1,1,1,3});
    shuff_.add("Intake Has Started", &intakeTiming_.hasStarted, {1,1,2,3});
    shuff_.add("Intake Has Finished", &intakeTiming_.finished, {1,1,3,3});
    
    shuff_.add("Intake Padding", &INTAKE_PADDING, {1,1,4,3}, true);
    shuff_.add("Intake Time", &INTAKE_TIME, {1,1,5,3}, true);

    //Print Pathing
    shuff_.addButton("Print Path", [&]{
        for(uint i = 0; i < paths_.size(); i++){
            std::cout<<"Path "<<i<<std::endl;
            for(AutoElement element : paths_[i]){
                std::cout<<ElementToString(element)<<std::endl;
            }
        };
        std::cout<<"Printed Path"<<std::endl;
    },{3,2,6,0});
}

void Auto::ShuffleboardPeriodic(){
    if(!shuff_.isEnabled()){
        return;
    }
    segments_.ShuffleboardPeriodic();

    shuff_.PutNumber("time", Utils::GetCurTimeS() - autoStart_);
    shuff_.PutNumber("Drive Progress", segments_.GetProgress());

    shuff_.update(true);
}