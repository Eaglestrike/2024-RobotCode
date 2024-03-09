#include "Drive/SwerveControl.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include <frc/smartdashboard/SmartDashboard.h>

#include "Constants/SwerveConstants.h"
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
  m_angCorrectorInverted{SwerveConstants::ANG_CORRECT_INVERTED},
  m_fr{SwerveConstants::FR_CONFIG, enabled, shuffleboard},
  m_br{SwerveConstants::BR_CONFIG, enabled, shuffleboard},
  m_fl{SwerveConstants::FL_CONFIG, enabled, shuffleboard},
  m_bl{SwerveConstants::BL_CONFIG, enabled, shuffleboard},
  m_pfr{0}, m_pbr{0}, m_pfl{0}, m_pbl{0},
  m_curAngle{0}, m_angCorrection{0},
  m_angleCorrector{SwerveConstants::ANG_CORRECT_P, SwerveConstants::ANG_CORRECT_I, SwerveConstants::ANG_CORRECT_D},
  m_prevTime{0},
  m_deltaT{0.02},
  m_speedNoAngCorrect{SwerveConstants::SPEED_NO_ANG_CORRECT}
{
  m_angleCorrector.EnableContinuousInput(-M_PI, M_PI);
  m_angleCorrector.SetTolerance(SwerveConstants::ANG_CORRECT_TOL, std::numeric_limits<double>::infinity());

  ResetFF();

  SetAngleCorrectionPID(SwerveConstants::ANG_CORRECT_P, SwerveConstants::ANG_CORRECT_I, SwerveConstants::ANG_CORRECT_D);
}

/**
 * Resets previous speeds for acceleration feed forward
 */
void SwerveControl::ResetFF()
{
  m_pfr = 0;
  m_pbr = 0;
  m_pfl = 0;
  m_pbl = 0;
}

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
  vectors.push_back(m_fl.GetVelocity());
  vectors.push_back(m_fr.GetVelocity());
  vectors.push_back(m_bl.GetVelocity());
  vectors.push_back(m_br.GetVelocity());

  auto avg = Utils::GetVecAverage(vectors);
  return vec::rotate(avg, ang); // rotate by navx ang
}

/**
 * Gets robot angular velocity by taking the dot product with the tangent to the circle
 *
 * @note Assumes modules have 
 *
 * @returns Robot angular velocity
 */
double SwerveControl::GetRobotAngularVel()
{
  const double r = SwerveConstants::CENTER_TO_EDGE*std::sqrt(2.0);
  double angVel = 0.0;
  angVel += m_fl.GetVelocity().dot(vec::Vector2D{-1,1}.normalize());
  angVel += m_fr.GetVelocity().dot(vec::Vector2D{1,1}.normalize());
  angVel += m_bl.GetVelocity().dot(vec::Vector2D{-1,-1}.normalize());
  angVel += m_br.GetVelocity().dot(vec::Vector2D{1,-1}.normalize());
  angVel /= r*4.0;
  return angVel;
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
 */
void SwerveControl::SetRobotVelocity(vec::Vector2D vel, double angVel, double ang)
{
  if (!Utils::NearZero(angVel))
  {
    // if turning, track current angle
    m_curAngle = ang;
  }

  if (!Utils::NearZero(vel) && Utils::NearZero(angVel) && m_angCorrection && magn(vel) > m_speedNoAngCorrect)
  {
    // if not turning, correct robot so that it doesnt turn
    angVel = m_angleCorrector.Calculate(ang, m_curAngle);
    angVel = m_angCorrectorInverted ? -angVel : angVel;
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
  double volts = m_kS + m_kV * magn(velBody) + m_kA * (magn(velBody) - prevSpeed) / m_deltaT;

  if (!m_autoEnabled) {
    volts = magn(velBody);
  }

  prevSpeed = magn(velBody);
  if (!Utils::NearZero(velBody) && !Utils::NearZero(volts))
  {
    velBody = normalize(velBody) * volts;
  }

  // set vector
  module.SetVector(velBody);
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
  m_fr.Init();
  m_br.Init();
  m_fl.Init();
  m_bl.Init();
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
  SetRobotVelocity(velAbs, angVel, ang);
}

/**
 * Periodic function
 */
void SwerveControl::CorePeriodic(){
  double curTime = Utils::GetCurTimeS();
  m_deltaT = curTime - m_prevTime;
  m_prevTime = curTime;

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
  frc::SmartDashboard::PutNumber("Speed No Ang Correct", SwerveConstants::SPEED_NO_ANG_CORRECT);

  frc::SmartDashboard::PutNumber("swerve kS", SwerveConstants::kS);
  frc::SmartDashboard::PutNumber("swerve kV", SwerveConstants::kV);
  frc::SmartDashboard::PutNumber("swerve kA", SwerveConstants::kA);
}

void SwerveControl::CoreShuffleboardPeriodic() {
  m_speedNoAngCorrect = frc::SmartDashboard::GetNumber("Speed No Ang Correct", SwerveConstants::SPEED_NO_ANG_CORRECT);
}

/**
 * Shuffleboard update update
*/
void SwerveControl::CoreShuffleboardUpdate(){
  double kP2 = frc::SmartDashboard::GetNumber("ang correct kP", SwerveConstants::ANG_CORRECT_P);
  double kI2 = frc::SmartDashboard::GetNumber("ang correct kI", SwerveConstants::ANG_CORRECT_I);
  double kD2 = frc::SmartDashboard::GetNumber("ang correct kD", SwerveConstants::ANG_CORRECT_D);

  SetAngleCorrectionPID(kP2, kI2, kD2);

  double kS = frc::SmartDashboard::GetNumber("swerve kS", SwerveConstants::kS);
  double kV = frc::SmartDashboard::GetNumber("swerve kV", SwerveConstants::kV);
  double kA = frc::SmartDashboard::GetNumber("swerve kA", SwerveConstants::kA);

  SetFFConstants(kS, kV, kA);
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