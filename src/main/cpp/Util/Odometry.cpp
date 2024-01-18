#include "Util/Odometry.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>

#include "Constants/FieldConstants.h"
#include "Constants/OdometryConstants.h"
#include "Util/Utils.h"

/**
 * Constructor
*/
Odometry::Odometry() :
  m_curAng{0}, m_startAng{0}, m_angVel{0}, m_joystickAng{0}, 
  m_prevDriveTime{Utils::GetCurTimeS()}, m_estimator{OdometryConstants::SYS_STD_DEV},
  m_uniqueId{-1000}, m_prevCamTime{-1000} {}

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
  double deltaT = curTime - m_prevDriveTime;

  vec::Vector2D deltaPos = vel * deltaT;
  m_estimator.UpdateDrivebase(curTime, deltaPos);

  double prevAng = m_curAng;
  m_curAng = angNavXAbs + m_startAng; 
  Update(deltaT, prevAng);

  m_prevDriveTime = curTime;
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

/**
 * Updates cams for apriltags
 * 
 * @param relPos position of tags relative to bobot
 * @param tagId Tag ID
 * @param uniqueId Unique ID to prevent repeat measurements
 * @param age Age of cameras, ms
*/
void Odometry::UpdateCams(const vec::Vector2D &relPos, const int &tagId, const long long &uniqueId, const long long &age) {
  double curTime = Utils::GetCurTimeS();
  frc::SmartDashboard::PutBoolean("Tag Detected", curTime - m_prevCamTime < PoseEstimator::MAX_HISTORY_TIME);

  // filter out repeats
  if (uniqueId == m_uniqueId) {
    return;
  }
  m_uniqueId = uniqueId;

  // rotate relative cam pos to absolute
  double angNavX = GetAng();
  vec::Vector2D vecRot = rotate(relPos, angNavX - M_PI / 2);
  vec::Vector2D tagPos;

  // filter out bad IDs
  if (tagId < 1 || tagId > 16) {
    return;
  }

  // add tag position to get absolute robot position
  tagPos = FieldConstants::TAGS[tagId - 1];
  tagPos = Utils::InToM(tagPos);
  vec::Vector2D robotPosCams = tagPos - vecRot;

  // reject if apriltag pos is too far away
  vec::Vector2D odomPos = GetPos();
  if (magn(odomPos - robotPosCams) > OdometryConstants::AT_REJECT) {
    return;
  }

  // check if outside field
  double margin = Utils::InToM(FieldConstants::FIELD_MARGIN);
  double fWidth = Utils::InToM(FieldConstants::FIELD_WIDTH);
  double fHeight = Utils::InToM(FieldConstants::FIELD_HEIGHT);
  if (robotPosCams.x() < -margin || robotPosCams.x() > fWidth + margin
   || robotPosCams.y() < -margin || robotPosCams.y() > fHeight + margin) {
    return;
  }

  // update cams on pose estimator
  double stdDev = OdometryConstants::CAM_STD_DEV_COEF * magn(robotPosCams) * magn(robotPosCams);
  m_estimator.UpdateCams(curTime - age / 1000.0, robotPosCams, {stdDev, stdDev});

  // update prev cam time
  m_prevCamTime = curTime;
}