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
 * @param modules An std::array of references to 4 swerve modules
 * @param radii An std::array of 4 vec::Vector2D objects, each one representing a vector originating
 * from the center of the robot and pointing to the corresponding swerve module index in the modules parameter
 * @param kS kS value for feedforward
 * @param kV kV value for feedforward
 * @param kA kA value for feedforward
 *
 * @note feedforward relates speed to voltage
 */
SwerveControl::SwerveControl(RefArray<SwerveModule> modules, bool enabled, bool shuffleboard):
    Mechanism("Swerve Control", enabled, shuffleboard),
    m_modules{modules}, 
    m_kS{0.0}, m_kV{0.0}, m_kA{0.0}, m_curAngle{0},
    m_angCorrection{true},
    m_angleCorrector{SwerveConstants::ANG_CORRECT_P, SwerveConstants::ANG_CORRECT_I, SwerveConstants::ANG_CORRECT_D}
{
  m_angleCorrector.EnableContinuousInput(-M_PI, M_PI);
  ResetFeedForward();

  SetAngleCorrectionPID(SwerveConstants::ANG_CORRECT_P, SwerveConstants::ANG_CORRECT_I, SwerveConstants::ANG_CORRECT_D);
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
  for (auto module : m_modules)
  {
    vectors.push_back(module.get().GetVelocity());
  }

  auto avg = Utils::GetVecAverage(vectors);
  return vec::rotate(avg, ang); // rotate by navx ang
}

/**
 * Resets previous speeds for acceleration feed forward
 */
void SwerveControl::ResetFeedForward()
{
  m_prevSpeeds.fill(0);
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
 * Sets feed foward terms relating speed to voltage
 *
 * @param kS kS term
 * @param kV kV term
 * @param kA kA term
 */
void SwerveControl::SetFeedForward(double kS, double kV, double kA)
{
  m_kS = kS;
  m_kV = kV;
  m_kA = kA;
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
void SwerveControl::SetRobotVelocity(vec::Vector2D vel, double angVel, double ang, double time)
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

  for (std::size_t i = 0; i < 4; i++)
  {
    m_modules[i].get().SetLock(false);

    // computes vectors in 3D
    vec::Vector3D vel3D = {x(vel), y(vel), 0};
    vec::Vector3D angVel3D = {0, 0, angVel};
    vec::Vector2D radius = m_modules[i].get().getPosition();
    vec::Vector3D module3D = {x(radius), y(radius), 0};

    // vector addition for velocity
    vec::Vector3D moduleWorld = rotateGamma(module3D, ang);        // rotates radius vector to world frame
    vec::Vector3D velWorld = vel3D + cross(angVel3D, moduleWorld); // linear + tangential velocity for module

    // rotates velocity vector to body frame
    vec::Vector3D velBody3D = rotateGamma(velWorld, -ang);

    // sets module velocity
    vec::Vector2D velBody = {x(velBody3D), y(velBody3D)};

    // speed from ff calculations, then resize velBody to match ff calculations
    double speed = m_kS + m_kV * magn(velBody) + m_kA * (magn(velBody) - m_prevSpeeds[i]) / time;
    m_prevSpeeds[i] = magn(velBody);
    if (!Utils::NearZero(velBody) && !Utils::NearZero(speed))
    {
      velBody = normalize(velBody) * speed;
    }

    // set vector
    m_modules[i].get().SetVector(velBody);
  }
}

void SwerveControl::Lock() {
  m_modules[0].get().SetSpeed(0);
  m_modules[1].get().SetSpeed(0);
  m_modules[2].get().SetSpeed(0);
  m_modules[3].get().SetSpeed(0);

  m_modules[0].get().SetAngle(-45);
  m_modules[1].get().SetAngle(45);
  m_modules[2].get().SetAngle(45);
  m_modules[3].get().SetAngle(-45);


  m_modules[0].get().SetLock(true);
  m_modules[1].get().SetLock(true);
  m_modules[2].get().SetLock(true);
  m_modules[3].get().SetLock(true);
}

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
void SwerveControl::SetRobotVelocityTele(vec::Vector2D vel, double angVel, double ang, double time, double angOfJoystick) {
  vec::Vector2D velAbs = vec::rotate(vel, angOfJoystick);
  SetRobotVelocity(velAbs, angVel, ang, time);
}

/**
 * Periodic function
 */
void SwerveControl::CorePeriodic(){
  for (auto module : m_modules)
  {
    module.get().TeleopPeriodic();
  }
}

void SwerveControl::CoreShuffleboardInit(){
  frc::SmartDashboard::PutNumber("ang correct kP", SwerveConstants::ANG_CORRECT_P);
  frc::SmartDashboard::PutNumber("ang correct kI", SwerveConstants::ANG_CORRECT_I);
  frc::SmartDashboard::PutNumber("ang correct kD", SwerveConstants::ANG_CORRECT_D);
}

void SwerveControl::CoreShuffleboardUpdate(){
  double kP2 = frc::SmartDashboard::GetNumber("ang correct kP", SwerveConstants::ANG_CORRECT_P);
  double kI2 = frc::SmartDashboard::GetNumber("ang correct kI", SwerveConstants::ANG_CORRECT_I);
  double kD2 = frc::SmartDashboard::GetNumber("ang correct kD", SwerveConstants::ANG_CORRECT_D);

  SetAngleCorrectionPID(kP2, kI2, kD2);
}

void SwerveControl::SetAngCorrection(bool angCorrection) {
  m_angCorrection = angCorrection;
}