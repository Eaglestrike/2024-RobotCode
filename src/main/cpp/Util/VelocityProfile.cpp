#include "Util/VelocityProfile.h"

#include "Util/Util.h"

/**
 * Constructors
*/
VelocityProfile::VelocityProfile(double maxA):
    maxA_{maxA},
    startPose_{0.0, 0.0, 0.0},
    startTime_{0.0},
    finalPose_{0.0, 0.0, 0.0},
    finalTime_{0.0}
{
}

/**
 * Sets the target and starts the profile
*/
void VelocityProfile::SetTarget(double finalVel, Poses::Pose1D startPose){
    startPose_ = startPose;
    startTime_ = Utils::GetCurTimeS();

    if(maxA_ == 0){ //Can't accelerate = can't get to finalVel
        startPose_.acc = 0.0;
        finalTime_ = 0.0;
        finalPose_ = startPose_;
        return;
    }

    double dV = finalVel - startPose.vel;//Change in velocity
    double sV = Utils::sign(dV);
    startPose_.acc = sV*maxA_;

    if(startPose_.acc != 0.0){
        double t = dV/startPose_.acc; //Duration of profile, v = a*t
        finalPose_ = Poses::extrapolate(startPose_, t);
        finalTime_ = startTime_ + t;
    }
    else{
        finalPose_ = startPose_;
        finalTime_ = startTime_;
    }
}

/**
 * Gets the profile pose at the current time
*/
Poses::Pose1D VelocityProfile::GetPose(){
    double t = Utils::GetCurTimeS();
    if(t < startTime_){//Weird case
        return startPose_;
    }
    else if(t < finalTime_){
        return Poses::extrapolate(startPose_, t - startTime_);
    }
    else{
        return Poses::extrapolate(finalPose_, t - finalTime_);
    }
}

bool VelocityProfile::isFinished(){
    return Utils::GetCurTimeS() > finalTime_;
}

double VelocityProfile::GetDuration(){
    return finalTime_ - startTime_;
}

/**
 * Getters and Setters
*/
double VelocityProfile::GetMaxA(){
    return maxA_;
}

void VelocityProfile::SetMaxA(double maxA){
    maxA_ = maxA;
}