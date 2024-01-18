#include "Util/Odometry.h"

#include "Constants/OdometryConstants.h"
#include "Util/Utils.h"

/**
 * Constructor
*/
Odometry::Odometry() :
  m_curAng{0}, m_startAng{0}, m_angVel{0}, m_joystickAng{0}, 
  m_estimator{OdometryConstants::SYS_STD_DEV}, m_prevTime{Utils::GetCurTimeS()} {}

/**
 * Sets starting configuration, then resets position.
 * 
 * @param pos Position
 * @param ang Angle, in radians
 * @param joystickAng Joystick angle, in radians
*/
void Odometry::SetStartingConfig(const vec::Vector2D &pos, const double &ang, const double &joystickAng) {
  m_startPos = pos;
  m_startAng = ang;
  m_joystickAng = joystickAng;
  
  Reset();
}

/**
 * Resets odometry to initial position set in SetStartingConfig()
 * 
 * @note Call navX->Reset() before this
*/
void Odometry::Reset() {
  m_curPos = m_startPos;
  m_vel = {0, 0};
  m_curAng = m_startAng;
  m_angVel = 0;

  m_estimator.SetPos(m_curPos);
}

/**
 * Gets current position
 * 
 * @returns current position
*/
vec::Vector2D Odometry::GetPos() const {
  return m_curPos;
}

/**
 * Gets velocity
 * 
 * @returns current velocity
*/
vec::Vector2D Odometry::GetVel() const {
  return m_vel;
}

/**
 * Gets ang vel
 * 
 * @returns angular velocity
*/
double Odometry::GetAngVel() const {
  return m_angVel;
}

/**
 * Gets current angle, in radians
 * 
 * @returns Current angle, in radians
*/
double Odometry::GetAng() const {
  return m_curAng;
}

/**
 * Gets normalized ang from -pi to pi
 * 
 * @returns Normalized Angle
*/
double Odometry::GetAngNorm() const {
  return Utils::NormalizeAng(GetAng());
}

/**
 * Gets joystick angle, in radians
 * 
 * @returns joystick angle, in radians
*/
double Odometry::GetJoystickAng() const {
  return m_joystickAng;
}

/**
 * Gets starting position
 * 
 * @returns Starting position
*/
vec::Vector2D Odometry::GetStartPos() const {
  return m_startPos;
}

/**
 * Gets starting angle
 * 
 * @returns Starting angle
*/
double Odometry::GetStartAng() const {
  return m_startAng;
}

/**
 * Updates encoder
 * 
 * @param vel Velocity
 * @param angNavXAbs navX absolute angle, in radians
*/
void Odometry::UpdateEncoder(const vec::Vector2D &vel, const double &angNavXAbs) {
  double curTime = Utils::GetCurTimeS();
  double deltaT = curTime - m_prevTime;

  vec::Vector2D deltaPos = vel * deltaT;
  m_estimator.UpdateDrivebase(curTime, deltaPos);

  double prevAng = m_curAng;
  m_curAng = angNavXAbs + m_startAng; 
  Update(deltaT, prevAng);

  m_prevTime = curTime;
}

/**
 * Updates odometry
 * 
 * @param deltaT change in time
 * @param prevAng previous angle
*/
void Odometry::Update(const double &deltaT, const double &prevAng) {
  vec::Vector2D prevPos = m_curPos;
  m_curPos = m_estimator.GetCurPos();
  m_vel = (m_curPos - prevPos) / deltaT;
  m_angVel = (m_curAng - prevAng) / deltaT;
}
