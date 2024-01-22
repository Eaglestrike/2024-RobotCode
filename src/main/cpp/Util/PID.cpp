#include "Util/PID.h"

#include <iostream>
#include "Util/Utils.h"

/**
 * Constructor
 * 
 * @param config PID values
 * @param posTol position tolerance
 * @param velTol velocity tolerance
*/
PID::PID(PID::PIDConfig config, double posTol, double velTol):
    config_{config},
    posTol_{posTol},
    velTol_{velTol},
    accum_{0.0},
    prevPose_{0.0,0.0,0.0}
{

}

/**
 * Calculates PID controller value
 * 
 * @param pose current pose
 * @param target target pose
*/
double PID::Calculate(Poses::Pose1D pose, Poses::Pose1D target){
    double t = Utils::GetCurTimeS();
    double dt = t - prevT_;

    double error = target.pos - pose.pos;
    double velError = target.vel - pose.vel;
    accum_ += error * dt;

    atTarget_ = (std::abs(error) < posTol_) && (std::abs(velError) < velTol_);

    prevPose_ = pose;
    prevT_ = t;

    return (config_.kp*error) + (config_.ki*accum_) + (config_.kd*velError);
}

/**
 * Resets integral gain
*/
void PID::Reset(){
    accum_ = 0.0;
    prevT_ = Utils::GetCurTimeS();
}

/**
 * Returns if the PID is within the tolerance
*/
bool PID::AtTarget() const{
    return atTarget_;
}

/**
 * Sets PID values
*/
void PID::SetPID(PID::PIDConfig config){
    config_ = config;
}

/**
 * Gets PID values
*/
PID::PIDConfig PID::GetPID(){
    return config_;
}

/**
 * Sets position and velocity tolerances
*/
void PID::SetTolerance(double posTol, double velTol){
    posTol_ = posTol;
    velTol_ = velTol;
}

/**
 * Gets position tolerance
*/
double PID::GetPosTolerance(){
    return posTol_;
}

/**
 * Gets velocity tolerance
*/
double PID::GetVelTolerance(){
    return velTol_;
}