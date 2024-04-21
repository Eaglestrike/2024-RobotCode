#include "Drive/AutoAngLineup.h"

#include <climits>
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>

#include "Constants/AutoLineupConstants.h"
#include "Util/Utils.h"

/**
 * Constructor
 * 
 * @param shuffleboard whether shuffleboard enabled
 * @param odom Odometry
*/
AutoAngLineup::AutoAngLineup(bool shuffleboard, Odometry &odom):
    m_odom{odom},
    m_profile{AutoLineupConstants::MAX_SPEED, AutoLineupConstants::MAX_ACCEL},
    m_prevTime{0.0},
    m_kP{AutoLineupConstants::ANG_P}, m_kI{AutoLineupConstants::ANG_I}, m_kD{AutoLineupConstants::ANG_D},
    m_totalAngErr{0.0},
    m_posTol{AutoLineupConstants::ANG_TOL}, m_velTol{AutoLineupConstants::VEL_TOL},
    m_shuff{"Ang Lineup", shuffleboard}
{
  
}

double AutoAngLineup::GetAngVel(){
  double curTime = Utils::GetCurTimeS();
  double deltaT = curTime - m_prevTime;

  if(deltaT > 0.1){
    deltaT = 0.0;
  }
  //FF
  Poses::Pose1D targPose = m_profile.currentPose();
  double ff = targPose.vel;

  //PID
  Poses::Pose1D currPose = {m_odom.GetAng(), m_odom.GetAngVel(), 0.0};
  Poses::Pose1D error = targPose - currPose;

  double pid = (error.pos * m_kP) + (m_totalAngErr * m_kI) + (error.vel * m_kD);

  double totalVel = ff + pid;

  m_prevTime = curTime;

  return totalVel;
}

/**
 * Sets angular position target for robot
 * 
 * @param ang Absolute angle, if rel is true, otherwise delta position
 * 
 * @note ang parameter will be normalized to -180 to 180 if outside range
*/
void AutoAngLineup::SetTarget(double targAng) {
  if(targAng == m_profile.getTargetPose().pos){
    return;
  }
  Poses::Pose1D currPose;
  if(m_profile.isFinished()){
    currPose = {m_odom.GetAng(), m_odom.GetAngVel(), 0.0};
  }
  else{
    currPose = m_profile.currentPose();
  }
  double targNorm = currPose.pos + Utils::NormalizeAng(targAng - currPose.pos);
  //std::cout << "curAng: " << m_odom.GetAng() << " targ ang: " << targNorm << std::endl; 
  m_profile.setTarget(currPose, {targNorm, 0.0, 0.0});
}

void AutoAngLineup::Stop(){
  m_profile.Zero({0.0, 0.0, 0.0});
}

/**
 * Sets profile config
 * 
 * @param maxSpeed max speed
 * @param maxAccel max accel
*/
void AutoAngLineup::SetProfileConfig(double maxSpeed, double maxAccel) {
  m_profile.setMaxVel(maxSpeed);
  m_profile.setMaxAcc(maxAccel);
}

/**
 * SEts PID
 * 
 * @param kP p
 * @param kI i
 * @param kD d
*/
void AutoAngLineup::SetPID(double kP, double kI, double kD) {
  m_kP = kP;
  m_kI = kI;
  m_kD = kD;
}

/**
 * Returns whether robot angular position is at the target
 * 
 * Assumes autoconstant error tolerance
 * 
 * @returns Whether robot is where it is supposed to be angularly at the end of profile
*/
bool AutoAngLineup::AtTarget() {
  if(!m_profile.isFinished()){
    return false;
  }
  Poses::Pose1D error = m_profile.currentPose() - Poses::Pose1D{m_odom.GetAng(), m_odom.GetAngVel(), 0.0};
  return (std::abs(error.pos) < m_posTol) && (std::abs(error.vel) < m_velTol);
}

double AutoAngLineup::GetTargAng(){
  return m_profile.getTargetPose().pos;
}

double AutoAngLineup::GetExpAng(){
  return m_profile.currentPose().pos;
}

/**
 * Shuffleboard init
*/
void AutoAngLineup::ShuffleboardInit() {
  if (!m_shuff.isEnabled()) {
    return;
  }

  m_shuff.PutNumber("lineup max speed", m_profile.getMaxVel(), {1,1,0,0});
  m_shuff.PutNumber("lineup max accel", m_profile.getMaxAcc(), {1,1,1,0});
  m_shuff.addButton("Set Profile", [&]{
    SetProfileConfig(
      m_shuff.GetNumber("", 0.0),
      m_shuff.GetNumber("", 0.0)
    );
  }, {2,1,2,0});

  m_shuff.add("lineup ang kp", &m_kP, {1,1,0,1});
  m_shuff.add("lineup ang ki", &m_kI, {1,1,1,1});
  m_shuff.add("lineup ang kd", &m_kD, {1,1,2,1});

  m_shuff.PutNumber("Ang Target", 0.0, {1,1,0,2});
  m_shuff.addButton("Set Target", [&]{
    SetTarget(m_shuff.GetNumber("Ang Target", 0.0));
  }, {2,1,1,2});

  m_shuff.PutNumber("pos error", 0, {1,1,0,3});
  m_shuff.PutNumber("vel error", 0, {1,1,1,3});

  m_shuff.add("pos tol", &m_posTol, {1,1,2,3});
  m_shuff.add("vel tol", &m_velTol, {1,1,2,3});

  
  m_shuff.PutNumber("curr pos", 0, {1,1,5,1});
  m_shuff.PutNumber("curr vel", 0, {1,1,6,1});
  m_shuff.PutNumber("exp pos", 0, {1,1,5,2});
  m_shuff.PutNumber("exp vel", 0, {1,1,6,2});

}

/**
 * Shuffleboard Periodic
*/
void AutoAngLineup::ShuffleboardPeriodic() {
  if (!m_shuff.isEnabled()) {
    return;
  }

  Poses::Pose1D targPos = m_profile.currentPose();
  Poses::Pose1D currPos = {m_odom.GetAng(), m_odom.GetAngVel(), 0.0};
  Poses::Pose1D error = targPos - currPos;

  m_shuff.PutNumber("pos error", Utils::NormalizeAng(error.pos));
  m_shuff.PutNumber("vel error", error.vel);

  m_shuff.PutNumber("cur ang", currPos.pos);
  m_shuff.PutNumber("cur vel", currPos.vel);

  m_shuff.PutNumber("exp ang", targPos.pos);
  m_shuff.PutNumber("exp vel", targPos.vel);
}