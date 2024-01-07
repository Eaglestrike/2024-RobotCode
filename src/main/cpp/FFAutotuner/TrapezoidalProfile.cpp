#include "FFAutotuner/TrapezoidalProfile.h"

#include "FFAutotuner/FFHelpers.hpp"

TrapezoidalProfile::TrapezoidalProfile(double maxVel, double maxAcc):
    maxVel_(std::abs(maxVel)),
    maxAcc_(std::abs(maxAcc))
{
    Zero({0.0, 0.0, 0.0});
    setTarget(startPose_, finalPose_);
}

/**
 * Calculates the current expected pose on the profile
 * 
 * @returns profile pose
*/
Poses::Pose1D TrapezoidalProfile::currentPose(){
    double time = timer_.Get().value();
    //If statement done in reverse order to deal with triangular profiles
    if(time > calcTimes_.deaccTime){
        return Poses::extrapolate(finalPose_, time - calcTimes_.deaccTime); //After done with profile, just moves at constant velocity
    }
    else if(time > calcTimes_.coastTime){ //Deaccelerating
        return Poses::extrapolate(calcTimes_.coastEnd, time -  calcTimes_.coastTime);
    }
    else if(time > calcTimes_.accTime){ //Coasting
        return Poses::extrapolate(calcTimes_.accEnd, time - calcTimes_.accTime);
    }
    else if(time >= 0.0){ //Accelerating
        return Poses::extrapolate(startPose_, time);
    }
    else{
        return {
            .pos = startPose_.pos,
            .vel = 0.0,
            .acc = 0.0
        };
    }
}

bool TrapezoidalProfile::setTarget(Poses::Pose1D currPose, Poses::Pose1D finalPose){
    //https://www.desmos.com/calculator/yy6g6utcwy <- refer to "Math" folder
    timer_.Reset();
    timer_.Start();
    startPose_ = currPose;
    if((maxAcc_ == 0) || (maxVel_ == 0) || (std::abs(currPose.vel) > maxVel_) || (std::abs(finalPose.vel) > maxVel_)){
        Zero(currPose); //Should stop if profile is bad
        return false; 
    }

    finalPose_ = finalPose;
    finalPose_.acc = 0.0;

    double dx = finalPose_.pos - startPose_.pos;
    double dv = finalPose_.vel - startPose_.vel;

    double sVel = FFHelpers::sign(dv);
    double aVel = sVel * maxAcc_;
    double tVel = 0.0;  
    if(dv != 0){
        tVel = dv/aVel;
    }
    double xVel = (finalPose_.vel + startPose_.vel)/2.0 * tVel; //x = (vf+vi)/2 * t

    double xTpzd = dx - xVel;
    double sTpzd = FFHelpers::sign(xTpzd);
    double vTpzdM = sTpzd * maxVel_;
    double aTpzdM = sTpzd * maxAcc_;
    double vTpzdI;
    if(sTpzd > 0.0){
        vTpzdI = std::max(finalPose_.vel, startPose_.vel);
    }
    else{
        vTpzdI = std::min(finalPose_.vel, startPose_.vel);
    }
    double xTpzdAcc = (vTpzdM*vTpzdM - vTpzdI*vTpzdI)/(2.0*aTpzdM); //vf^2 = vi^2 + 2ad
    double xCoast = xTpzd - 2.0*xTpzdAcc;
    double tCoast, vTpzd;
    if(xCoast * sTpzd > 0.0){
        tCoast = xCoast/vTpzdM;
        vTpzd = vTpzdM;
    }
    else{
        tCoast = 0.0;
        vTpzd = sTpzd * std::sqrt(vTpzdI*vTpzdI + aTpzdM*xTpzd); //vf = sqrt(vi^2 + ad)
    }
    double tTpzdAcc = (vTpzd - vTpzdI)/aTpzdM;
    
    startPose_.acc = aTpzdM;
    calcTimes_.accTime = tTpzdAcc;
    if(sTpzd * sVel > 0.0){
        calcTimes_.accTime += tVel;
    }
    double xAccEnd = startPose_.pos + (startPose_.vel + vTpzd)/2.0 * calcTimes_.accTime;
    calcTimes_.accEnd = {
        .pos = xAccEnd,
        .vel = vTpzd,
        .acc = 0.0
    };
    calcTimes_.coastTime = calcTimes_.accTime + tCoast;
    double xCoastEnd = xAccEnd + vTpzd*tCoast;
    calcTimes_.coastEnd = {
        .pos = xCoastEnd,
        .vel = vTpzd,
        .acc = -aTpzdM
    };
    calcTimes_.deaccTime = 2.0*tTpzdAcc + tCoast + tVel;
    return true;
}

bool TrapezoidalProfile::isFinished(){
    return timer_.Get().value() > calcTimes_.deaccTime;
}

double TrapezoidalProfile::getTime(){
    return timer_.Get().value();
}

double TrapezoidalProfile::getDuration(){
    return calcTimes_.deaccTime;
}

double TrapezoidalProfile::getMaxVel(){
    return maxVel_;
}

double TrapezoidalProfile::getDisplacement(){
    return finalPose_.pos - startPose_.pos;
}

double TrapezoidalProfile::getMaxAcc(){
    return maxAcc_;
}

void TrapezoidalProfile::setMaxVel(double maxVel){
    maxVel_ = maxVel;
}

void TrapezoidalProfile::setMaxAcc(double maxAcc){
    maxAcc_ = maxAcc;
}

void TrapezoidalProfile::Zero(Poses::Pose1D pose){
    startPose_ = pose;
    startPose_.vel = 0.0;
    startPose_.acc = 0.0;
    finalPose_ = startPose_;
    calcTimes_ = {
        .accTime = 0.0,
        .accEnd = startPose_,
        .coastTime = 0.0,
        .coastEnd = startPose_,
        .deaccTime = 0.0
    };
}