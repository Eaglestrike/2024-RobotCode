#include "FFAutotuner/FFAutotuner.h"

#include "FFAutotuner/FFHelpers.hpp"
#include <random>

using namespace Poses;

/**
 * Constructor
 * 
 * @param name name
 * @param type type of mechanism
 * @param min min, can be set later
 * @param max max, can be set later
 * @param targTim target time to hit for the max distance
 * @param testTime starting testing time
*/
FFAutotuner::FFAutotuner(std::string name, FFType type, double min, double max, double targTime, double testTime):
    name_(name),
    ffType_(type),
    state_(TUNING),
    currPose_({.pos = 0.0, .vel = 0.0, .acc = 0.0}),
    lastTime_(frc::Timer::GetFPGATimestamp().value()),
    profile_(0.0, 0.0),
    targTime_(targTime), testTime_(testTime < targTime? targTime*3.0 : testTime),
    bounds_({.min = min, .max = max}),
    ffTesting_({.ks = 0.0, .kv = 0.0, .ka = 0.0, .kg = 0.0}),
    ShuffData_(name)
{
    resetError();
    
    ShuffData_.add("s", &s_, {2,2,0,0}, true);
    ShuffData_.add("ks", &ffTesting_.ks, {2,1,0,3}, true);
    ShuffData_.add("kv", &ffTesting_.kv, {2,1,2,3}, true);
    ShuffData_.add("ka", &ffTesting_.ka, {2,1,4,3}, true);
    ShuffData_.add("kg", &ffTesting_.kg, {2,1,6,3}, true);

    ShuffData_.add("Target Time", &targTime_, {2,1,2,0}, true);
    ShuffData_.add("Testing Time", &testTime_, {2,1,2,1}, true);
    
    ShuffData_.add("eks", &error_.gainError.ks, {2,1,0,4});
    ShuffData_.add("ekv", &error_.gainError.kv, {2,1,2,4});
    ShuffData_.add("eka", &error_.gainError.ka, {2,1,4,4});
    ShuffData_.add("ekg", &error_.gainError.kg, {2,1,6,4});

    ShuffData_.add("total abs pos error", &error_.absTotalError.pos, {2,1,9,2});
    ShuffData_.add("total abs vel error", &error_.absTotalError.vel, {2,1,9,3});

    ShuffData_.add("precision", &precision_, {2,1,5,0}, true);
    ShuffData_.add("min", &bounds_.min, {1,1,5,1}, true);
    ShuffData_.add("max", &bounds_.max, {1,1,6,1}, true);

    ShuffData_.PutNumber("avg abs pos error", 0.0, {1,1, 8, 2});
    ShuffData_.PutNumber("avg pos error", 0.0, {1, 1, 8, 3});
}

FFAutotuner::State FFAutotuner::getState(){
    return state_;
}

void FFAutotuner::setPose(Pose1D currPose){
    currPose_ = currPose;

    double currT = frc::Timer::GetFPGATimestamp().value();
    double dt = currT - lastTime_;
    lastTime_ = currT;
    
    if(currPose_.pos > bounds_.max && state_ != RECENTER_FROM_MAX){ // Out of bounds
        state_ = RECENTER_FROM_MAX;
        resetProfile(true);
        std::cout<<name_ <<" Recentering"<<std::endl;
        return;
    }
    if(currPose_.pos < bounds_.min && state_ != RECENTER_FROM_MIN){ // Out of bounds
        state_ = RECENTER_FROM_MIN;
        resetProfile(true);
        std::cout<<name_ <<" Recentering"<<std::endl;
        return;
    }
    if(profile_.isFinished() || dt > 0.1){
        state_ = TUNING;
        resetProfile(false);
        return;
    }

    Pose1D expectedPose = profile_.currentPose();
    Pose1D error = (expectedPose - currPose_)*dt; //Error*dt

    double maxVel = profile_.getMaxVel();
    double maxAcc = profile_.getMaxAcc();

    double velComp = expectedPose.vel / maxVel;
    double stcComp = FFHelpers::sign(expectedPose.vel) - velComp; //static component will be inverted trapezoid
    double accComp = expectedPose.acc / maxAcc;
    double grvComp;
    switch(ffType_){
        case SIMPLE:   grvComp = 0.0;                        break;
        case ARM:      grvComp = std::cos(expectedPose.pos); break;
        case ELEVATOR: grvComp = 1.0;                        break;
        default:       grvComp = 0.0;
    }

    double absStcComp = std::abs(stcComp);
    double absVelComp = std::abs(velComp);
    double absAccComp = std::abs(accComp);
    double absGrvComp = std::abs(grvComp);
    double totAbsComp = absVelComp + absAccComp + absStcComp + absGrvComp;
    if(totAbsComp != 0){
        error_.gainError.ks += error.vel * stcComp/totAbsComp;
        error_.gainError.kv += error.vel * velComp/totAbsComp;
        error_.gainError.ka += error.vel * accComp/totAbsComp;
        error_.gainError.kg += error.vel * grvComp/totAbsComp;
        error_.totalError += error;
        error_.absTotalError += abs(error);
    }
}

double FFAutotuner::getVoltage(){
    Pose1D expectedPose = profile_.currentPose();
    switch(ffType_){
        case SIMPLE:
            return FFHelpers::sign(expectedPose.vel)*ffTesting_.ks + expectedPose.vel*ffTesting_.kv + expectedPose.acc*ffTesting_.ka;
        case ARM:
            return FFHelpers::sign(expectedPose.vel)*ffTesting_.ks + expectedPose.vel*ffTesting_.kv + expectedPose.acc*ffTesting_.ka + std::cos(expectedPose.pos)*ffTesting_.kg;
        case ELEVATOR:
            return FFHelpers::sign(expectedPose.vel)*ffTesting_.ks + expectedPose.vel*ffTesting_.kv + expectedPose.acc*ffTesting_.ka + ffTesting_.kg;
        default:
            return 0.0;
    }
}

void FFAutotuner::resetProfile(bool center){
    //Update feedforwards gains by average error for each term
    double duration = profile_.getDuration();
    double maxDist = bounds_.max - bounds_.min;
    if(duration != 0.0){
        double dKs = error_.gainError.ks/duration * s_;
        double dKv = error_.gainError.kv/duration * s_;
        double dKa = error_.gainError.ka/duration * s_;
        double dKg = error_.gainError.kg/duration * s_;

        ffTesting_.ks += dKs;
        ffTesting_.kv += dKv;
        ffTesting_.ka += dKa;
        ffTesting_.kg += dKg;

        //Calculate new profile, making it faster if it is reaching the target
        double avgAbsPosError = error_.absTotalError.pos/duration;
        double avgAbsVelError = error_.absTotalError.vel/duration;
        if((avgAbsPosError < maxDist/precision_) && //Check if round was accurate enough
           (avgAbsVelError < maxDist/precision_)){
            testTime_ = (testTime_ - targTime_)*0.8 + targTime_; //0.8 is decay rate
        }
        
        double avgPosError = error_.totalError.pos/duration * FFHelpers::sign(profile_.getDisplacement()); // Have avg error point in + direction
        if(pastPosErrors_.size() > 0){
            double prevPosError = pastPosErrors_[pastPosErrors_.size() - 1];
            if(FFHelpers::sign(prevPosError) != FFHelpers::sign(avgPosError)){ //Is oscillating
                s_ *= 0.75; //Scale down step size
            }
            else if(std::abs(avgPosError - prevPosError) < std::abs(prevPosError * 0.001)){ // Is not approaching fast enough; 0.001 threshold
                s_ *= 1.25; //Scale up 
            }
        }
        pastPosErrors_.push_back(avgPosError);

        ShuffData_.PutNumber("avg abs pos error", avgAbsPosError);
        ShuffData_.PutNumber("avg pos error", avgPosError);
    }
    double maxVel = maxDist/testTime_;
    double maxAcc = maxVel;
    profile_.setMaxVel(maxVel);
    profile_.setMaxAcc(maxAcc);

    double nextTarget;
    if(center){
        nextTarget = (bounds_.min + bounds_.max)/2.0;
    }
    else{
        nextTarget = bounds_.min + (maxDist * (random() % 100000L) / 100000.0); //Random next target
    }
    profile_.setTarget(currPose_, {.pos = nextTarget, .vel = 0.0, .acc = 0.0});

    resetError();
}

void FFAutotuner::zeroBounds(double val){
    bounds_.min = val;
    bounds_.max = val;
}

void FFAutotuner::expandBounds(double val){
    if(val > bounds_.max){
        bounds_.max = val;
    }
    if(val < bounds_.min){
        bounds_.min = val;
    }
}

void FFAutotuner::setMin(double min){
    bounds_.min = min;
}

void FFAutotuner::setMax(double max){
    bounds_.max = max;
}

FFAutotuner::FFConfig FFAutotuner::getFeedforward(){
    return ffTesting_;
}

void FFAutotuner::setFeedforward(FFConfig config){
    ffTesting_ = config;
}

void FFAutotuner::ShuffleboardUpdate(){
    ShuffData_.update(true);
    if(precision_ == 0.0){
        precision_ = 10.0;
    }
    if(targTime_ == 0.0){
        targTime_ = 1.0;
    }
    if(testTime_ < targTime_){
        testTime_ = targTime_;
    }
}

void FFAutotuner::resetError(){
    error_ = {
        .gainError = {.ks = 0.0, .kv = 0.0, .ka = 0.0, .kg = 0.0},
        .totalError = {.pos = 0.0, .vel = 0.0, .acc = 0.0},
        .absTotalError = {.pos = 0.0, .vel = 0.0, .acc = 0.0}
    };
}