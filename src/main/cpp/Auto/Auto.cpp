#include "Auto/Auto.h"

#include "Util/Utils.h"

using AutoPath = AutoConstants::AutoPath;
using AutoElement = AutoConstants::AutoElement;
using enum AutoConstants::AutoType;
using enum AutoConstants::AutoAction;

Auto::Auto(SwerveControl &swerve, Odometry &odom, bool shuffleboard):
    segments_{shuffleboard, swerve, odom},
    shuff_{"Auto", shuffleboard}
{
}

void Auto::SetPath(AutoPath path, int index){
    while(paths_.size() < index){
        paths_.push_back({});
    }
    paths_[index] = path;
    LoadPath(path);
}

void Auto::AutoInit(){
    pathNum_ = 0;
    index_ = 0;

    segments_.Clear();
    shooterTiming_.finished = true;
    intakeTiming_.finished = true;

    NextBlock();
}

void Auto::AutoPeriodic(){
    double t = Utils::GetCurTimeS();

    if(segments_.AtTarget() && shooterTiming_.finished && intakeTiming_.finished){
        NextBlock();
    }

    segments_.Periodic();
    ShooterPeriodic(t);
    IntakePeriodic(t);
}

void Auto::ShooterPeriodic(double t){
    if(!shooterTiming_.hasStarted && t > shooterTiming_.start){
        //shooter_.shoot()
        shooterTiming_.hasStarted = true;
    }

    //shooter_.prepare();
    //shooter_.periodic();
}

void Auto::IntakePeriodic(double t){
    if(!intakeTiming_.hasStarted && t > intakeTiming_.start){
        if(intaking_){
            //intake_.intake()
        }
        else{
            //intake.stow()
        }
        intakeTiming_.hasStarted = true;
    }
    if(intaking_){
        //intakeTiming_.finished = intake_.hasGamePiece();
    }
    //intake_.periodic();
}

/**
 * Sets up the next block (the actions between 2 AFTERS)
*/
void Auto::NextBlock(){
    if(pathNum_ >= paths_.size()){
        return;
    }
    AutoPath path = paths_[pathNum_];
    AutoElement firstElement = path[index_];

    blockStart_ = Utils::GetCurTimeS();

    //Figure out the block duration/initialize the first element
    switch(firstElement.action){
        case DRIVE:
            segments_.SetAutoPath(firstElement.data);
            segments_.Start();
            blockEnd_ = blockStart_ + segments_.GetDuration();
            break;
        case SHOOT:
            blockEnd_ = blockStart_ + AutoConstants::SHOOT_TIME;
            firstElement.type = AT_START;
            EvaluateShootElement(firstElement);
            break;
        case INTAKE:
            blockEnd_ = blockStart_ + AutoConstants::INTAKE_TIME;
            firstElement.type = AT_START;
            EvaluateIntakeElement(firstElement);
            break;
        case STOW:
            blockEnd_ = blockStart_ + AutoConstants::STOW_TIME;
            firstElement.type = AT_START;
            EvaluateIntakeElement(firstElement);
            break;
        default:
            std::cout<<"Did not deal with auto action case NB "<< firstElement.action <<std::endl;
    }
    index_++;

    //Lookahead for execution in this block
    for(;index_ < path.size();index_++){
        AutoElement element = path[index_];
        if(EvaluateElement(element)){
            break;
        }
    }

    if(index_ >= path.size()){//Finished this path
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
bool Auto::EvaluateElement(AutoConstants::AutoElement element){
    if(element.type == AFTER){
        return true;
    }
    switch(element.action){
        case DRIVE:
            return true;
        case SHOOT:
            EvaluateShootElement(element);
            return false;
        case INTAKE: [[fallthrough]];
        case STOW:
            EvaluateIntakeElement(element);
            return false;
        default:
            std::cout<<"Did not deal with auto action case EE "<< element.action <<std::endl;
    }
    return true;
}

/**
 * Evaluates a shoot element
 * Sets the timing for this block
*/
void Auto::EvaluateShootElement(AutoConstants::AutoElement element){
    shooterTiming_.finished = false;
    //Set offset
    double offset = 0.0;
    if(element.data != ""){
        offset = std::stod(element.data); //TODO CHECK IF THIS IS SAFE
    }
    //Set timing
    if(element.type == AT_START){
        shooterTiming_.start = blockStart_ + offset;
    }
    else if(element.type == BEFORE_END){
        shooterTiming_.start = blockEnd_ + AutoConstants::SHOOT_TIME - offset;
    }
    else{
        std::cout<<"forgor deal with case SE "<< element.action <<std::endl;
    }
}

/**
 * Evaluates a intake element
 * Sets the timing for this block
*/
void Auto::EvaluateIntakeElement(AutoConstants::AutoElement element){
    intaking_ = (element.action == INTAKE);
    intakeTiming_.finished = false;
    //Offset
    double offset = 0.0;
    if(element.data != ""){
        offset = std::stod(element.data); //TODO CHECK IF THIS IS SAFE
    }
    //Set timing
    if(element.type == AT_START){
        intakeTiming_.start = blockStart_ + offset;
    }
    else if(element.type == BEFORE_END){
        if(intaking_){
            intakeTiming_.start = blockEnd_ - AutoConstants::INTAKE_TIME - offset;
        }
        else{
            intakeTiming_.start = blockEnd_ - AutoConstants::STOW_TIME - offset;
        }
    }
    else{
        std::cout<<"forgor deal with case IE "<< element.action <<std::endl;
    }
}

void Auto::LoadPath(const AutoPath& path){
    for(const AutoElement& elem : path){
        if(elem.action == DRIVE){
            segments_.LoadAutoPath(elem.data);
        }
    }
}