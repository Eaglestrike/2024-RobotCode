#include "Drive/SwerveControl.h"

#include <algorithm>
#include <cstddef>
#include <iostream>
#include <vector>

#include <frc/smartdashboard/SmartDashboard.h>

#include "Constants/DriveConstants.h"
#include "Util/Utils.h"

/**
 * Constructor
 *
 * @param enabled If mechanism is enabled
 * @param shuffleboard Shuffleboard enabled
*/
SwerveControl::SwerveControl(bool enabled, bool shuffleboard) :
  Mechanism("Swerve Control", enabled, shuffleboard),
  m_kS{SwerveConstants::kS}, m_kV{SwerveConstants::kV}, m_kA{SwerveConstants::kA},
  m_autoEnabled{0},
  m_fr{SwerveConstants::FR_CONFIG, true, false},
  m_br{SwerveConstants::BR_CONFIG, true, false},
  m_fl{SwerveConstants::FL_CONFIG, true, false},
  m_bl{SwerveConstants::BL_CONFIG, true, false},
  m_pfr{0}, m_pbr{0}, m_pfl{0}, m_pbl{0},
  m_curAngle{0}, m_angCorrection{0}, m_prevTime{0}
  {}


/**
 * Gets robot velocity by averaging the velocities of the modules
 *
 * @note Assumes modules have the same mass
 * 
 * @param ang NavX angle
 *
 * @returns Robot velocity
 */
vec::Vector2D SwerveControl::GetRobotVelocity(double ang)
{
  std::vector<vec::Vector2D> vectors;
  for (auto module : m_modules)
  {
    vectors.push_back(module.get().GetVelocity());
  }

  auto avg = Utils::GetVecAverage(vectors);
  return vec::rotate(avg, ang); // rotate by navx ang
}

/**
 * Resets angle correction angle to a certain angle
 *
 * @note Always call this after calling navx::ZeroYaw()
 */
void SwerveControl::ResetAngleCorrection(double startAng)
{
  m_curAngle = startAng;
}

/**
 * Sets Angle Correction PID
 *
 * @param kP kP term
 * @param kI kI term
 * @param kD kD term
 */
void SwerveControl::SetAngleCorrectionPID(double kP, double kI, double kD)
{
  m_angleCorrector.SetPID(kP, kI, kD);
}

/**
 * Sets robot velocity
 *
 * @param vel Velocity to set
 * @param angVel Angular velocity, + is counterclockwise, - is clockwise
 * @param ang Current navX angle, in radians
 * @param time Time between readings
 */
void SwerveControl::SetRobotVelocity(vec::Vector2D vel, double angVel, double ang)
{
  if (!Utils::NearZero(angVel))
  {
    // if turning, track current angle
    m_curAngle = ang;
  }

  if (!Utils::NearZero(vel) && Utils::NearZero(angVel) && m_angCorrection)
  {
    // if not turning, correct robot so that it doesnt turn
    angVel = m_angleCorrector.Calculate(ang, m_curAngle);
    // frc::SmartDashboard::PutNumber("pidout", angVel);
    angVel = std::clamp(angVel, -SwerveConstants::MAX_VOLTS, SwerveConstants::MAX_VOLTS);
  }

  SetModuleVelocity(m_fr, m_pfr, vel, angVel, ang);
  SetModuleVelocity(m_br, m_pbr, vel, angVel, ang);
  SetModuleVelocity(m_fl, m_pfl, vel, angVel, ang);
  SetModuleVelocity(m_bl, m_pbl, vel, angVel, ang);
}

/**
 * Sets velocity for individuaal module
 * 
 * @param module Swerve module object
 * @param prevSpeed Previous speed
 * @param vel Target velocity of robot
 * @param angVel Target angular velocity of robot
 * @param ang current navX angle
*/
void SwerveControl::SetModuleVelocity(SwerveModule &module, double &prevSpeed, vec::Vector2D vel, double angVel, double ang) {
  double curTimeS = Utils::GetCurTimeS();

  module.SetLock(false);

  // computes vectors in 3D
  vec::Vector3D vel3D = {x(vel), y(vel), 0};
  vec::Vector3D angVel3D = {0, 0, angVel};
  vec::Vector2D radius = module.getPosition();
  vec::Vector3D module3D = {x(radius), y(radius), 0};

  // vector addition for velocity
  vec::Vector3D moduleWorld = rotateGamma(module3D, ang);        // rotates radius vector to world frame
  vec::Vector3D velWorld = vel3D + cross(angVel3D, moduleWorld); // linear + tangential velocity for module

  // rotates velocity vector to body frame
  vec::Vector3D velBody3D = rotateGamma(velWorld, -ang);

  // sets module velocity
  vec::Vector2D velBody = {x(velBody3D), y(velBody3D)};

  // apply voltage
  double speed = m_kS + m_kV * magn(velBody) + m_kA * (magn(velBody) - prevSpeed) / (curTimeS - m_prevTime);

  if (!m_autoEnabled) {
    speed = magn(velBody);
  }

  prevSpeed = magn(velBody);
  if (!Utils::NearZero(velBody) && !Utils::NearZero(speed))
  {
    velBody = normalize(velBody) * speed;
  }

  // set vector
  module.SetVector(velBody);

  m_prevTime = curTimeS;
}

/**
 * Locks wheels so drivebase cannot move or rotate
 * 
 * @note Calling SetRobotVelocity() will cause wheels to unlock
*/
void SwerveControl::Lock() {
  m_fr.SetSpeed(0);
  m_br.SetSpeed(0);
  m_fl.SetSpeed(0);
  m_bl.SetSpeed(0);

  m_fr.SetAngle(-45);
  m_br.SetAngle(45);
  m_fl.SetAngle(45);
  m_bl.SetAngle(-45);

  m_fr.SetLock(true);
  m_br.SetLock(true);
  m_fl.SetLock(true);
  m_bl.SetLock(true);
}

/**
 * Init function
*/
void SwerveControl::CoreInit(){
  ResetAngleCorrection();
}

/**
 * Sets robot absolute velocity given relative joystick inputs
 *
 * @param vel Velocity to set
 * @param angVel Angular velocity, + is counterclockwise, - is clockwise
 * @param ang Current navX angle, in radians
 * @param time Time between readings
 * @param angOfJoystick angle of joystick relative to field
 */
void SwerveControl::SetRobotVelocityTele(vec::Vector2D vel, double angVel, double ang, double angOfJoystick) {
  vec::Vector2D velAbs = vec::rotate(vel, angOfJoystick);
  SetRobotVelocity(velAbs, angVel, ang, time);
}

/**
 * Periodic function
 */
void SwerveControl::CorePeriodic(){
  m_fr.Periodic();
  m_br.Periodic();
  m_fl.Periodic();
  m_bl.Periodic();
}

/**
 * Shuffleboard init
*/
void SwerveControl::CoreShuffleboardInit(){
  frc::SmartDashboard::PutNumber("ang correct kP", SwerveConstants::ANG_CORRECT_P);
  frc::SmartDashboard::PutNumber("ang correct kI", SwerveConstants::ANG_CORRECT_I);
  frc::SmartDashboard::PutNumber("ang correct kD", SwerveConstants::ANG_CORRECT_D);
}

/**
 * Shuffleboard update update
*/
void SwerveControl::CoreShuffleboardUpdate(){
  double kP2 = frc::SmartDashboard::GetNumber("ang correct kP", SwerveConstants::ANG_CORRECT_P);
  double kI2 = frc::SmartDashboard::GetNumber("ang correct kI", SwerveConstants::ANG_CORRECT_I);
  double kD2 = frc::SmartDashboard::GetNumber("ang correct kD", SwerveConstants::ANG_CORRECT_D);

  SetAngleCorrectionPID(kP2, kI2, kD2);
}

/**
 * Sets if angle correction is on
 * 
 * @param angCorrection the angle correction
*/
void SwerveControl::SetAngCorrection(bool angCorrection) {
  m_angCorrection = angCorrection;
}

/**
 * Sets auto mode
 * 
 * If true, sets feed forward onto drivebase (speed of 1 translataes to 1m/s)
 * If false, feedforward is 0, 1, 0 (speed of 1 translates to 1V to drivebase)
 * 
 * @param enabled Whether auto mode should be enabled
*/
void SwerveControl::SetAutoMode(bool enabled) {
  m_autoEnabled = enabled;
}

/**
 * Sets feedforward constants
 * 
 * @param kS kS
 * @param kV kV
 * @param kA kA
*/
void SwerveControl::SetFFConstants(double kS, double kV, double kA) {
  m_kS = kS;
  m_kV = kV;
  m_kA = kA;
}