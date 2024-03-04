#include "Util/Odometry.h"

#include <cmath>
#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>

#include "Constants/FieldConstants.h"
#include "Constants/OdometryConstants.h"
#include "Util/Utils.h"
#include "Util/SideHelper.h"

/**
 * Constructor
*/
Odometry::Odometry(const bool &shuffleboard) :
  m_isAuto{false}, m_shuffleboard{shuffleboard}, m_curAng{0}, m_startAng{0}, m_angVel{0},
  m_curYaw{0}, m_joystickAng{0}, 
  m_prevDriveTime{Utils::GetCurTimeS()}, m_estimator{OdometryConstants::SYS_STD_DEV_AUTO},
  m_uniqueId{-1000}, m_timeOffset{OdometryConstants::CAM_TIME_OFFSET}, m_prevCamTime{-1000},
  m_camStdDevCoef{OdometryConstants::CAM_STD_DEV_COEF_AUTO}, m_turnStdDevCoef{OdometryConstants::CAM_TURN_STD_DEV_COEF},
  m_trustCamsMore{false} {}

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
 * Sets auto mode
 * 
 * @param autoMode if true, then faster correct on cameras, if false, trusts caeras less
*/
void Odometry::SetAuto(const bool &autoMode) {
  if (autoMode) {
    m_camStdDevCoef = OdometryConstants::CAM_STD_DEV_COEF_AUTO;
    m_estimator.SetQ(OdometryConstants::SYS_STD_DEV_AUTO);
    m_turnStdDevCoef = 0;
    m_trustCamsMore = false;
  } else {
    m_turnStdDevCoef = OdometryConstants::CAM_TURN_STD_DEV_COEF;
    m_camStdDevCoef = OdometryConstants::CAM_STD_DEV_COEF_TELE;
    m_estimator.SetQ(OdometryConstants::SYS_STD_DEV_TELE);
    m_trustCamsMore = true;
  }

  m_isAuto = autoMode;
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
  m_curYaw = Utils::NormalizeAng(m_startAng);
  m_angHistory.clear();

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
 * Gets cam position
 * 
 * @returns cam pos
*/
vec::Vector2D Odometry::GetCamPos() const {
  return m_camPos;
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
 * Gets yaw, which combines navX and drivebase angle data
 * 
 * @returns Current yaw, in radians, from -180 to 180
*/
double Odometry::GetYaw() const {
  return m_curYaw;
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
 * Gets whether tag is detected
*/
bool Odometry::GetTagDetected() const {
  return Utils::GetCurTimeS() - m_prevCamTime < PoseEstimator::MAX_HISTORY_TIME;
}

/**
 * Updates encoder
 * 
 * @param vel Velocity
 * @param angNavXAbs navX absolute angle, in radians
 * @param navXYaw navX getYaw() value
 * @param swerveAngVel swerve angular velocity
*/
void Odometry::UpdateEncoder(const vec::Vector2D &vel, const double &angNavXAbs, const double &navXYaw, const double &swerveAngVel) {
  double curTime = Utils::GetCurTimeS();
  double deltaT = curTime - m_prevDriveTime;

  vec::Vector2D deltaPos = (vel + m_prevVel) * deltaT /2.0;
  m_prevVel = vel;
  m_estimator.UpdateDrivebase(curTime, deltaPos);

  double prevAng = m_curAng;
  m_curAng = angNavXAbs + m_startAng; 
  Update(deltaT, prevAng);

  if (std::abs(swerveAngVel) >= OdometryConstants::USE_SWERVE_ANG_VEL) {
    m_curYaw += swerveAngVel * deltaT;
    m_curYaw = Utils::NormalizeAng(m_curYaw);
  } else {
    m_curYaw = Utils::NormalizeAng(navXYaw + m_startAng);
  }
  m_angVel = swerveAngVel;
  m_angHistory[curTime] = m_curAng;
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
  // m_angVel = (m_curAng - prevAng) / deltaT;

  double curTime = Utils::GetCurTimeS();
  while (m_angHistory.size() > 1 && m_angHistory.begin()->first < curTime - PoseEstimator::MAX_HISTORY_TIME) {
    m_angHistory.erase(m_angHistory.begin());
  }
}

/**
 * Gets interpolated angle from vision updates at a certain time,
 * assuming linear movement between measurements
 * 
 * @param camTime time to get interpolated angle from
 * 
 * @returns A pair. The first element determines whether the value is valid (true if valid),
 * and the second element determines the angle.
*/
std::pair<bool, double> Odometry::GetInterpolAng(const double &camTime) {
  double angAtTime = 0.0;
  if (m_angHistory.find(camTime) != m_angHistory.end()) {
    angAtTime = m_angHistory[camTime];
  } else {
    auto ub = m_angHistory.upper_bound(camTime);
    if (ub == m_angHistory.begin() || ub == m_angHistory.end()) {
      return {false, 0.0};
    }
    auto lb = ub;
    lb--;
    if (lb == m_angHistory.end()) {
      return {false, 0.0};
    }

    double time0 = lb->first;
    double time1 = ub->first;
    double deltaAng = ub->second - lb->second;

    double deltaAng0 = deltaAng * ((camTime - time0) / (time1 - time0));
    angAtTime = lb->second + deltaAng0;
  }

  return {true, angAtTime};
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
  double camTime = curTime - age / 1000.0 - m_timeOffset;

  if (curTime - camTime >= PoseEstimator::MAX_HISTORY_TIME) {
    // std::cout << "too old" << std::endl;
    return;
  }

  // filter out repeats
  if (uniqueId == m_uniqueId) {
    // std::cout << "not unique" << std::endl;
    return;
  }
  m_uniqueId = uniqueId;

  // rotate relative cam pos to absolute
  // using angle from when camera data was read
  std::pair<bool, double> histAng = GetInterpolAng(camTime);
  double angNavX = histAng.second;
  if (!histAng.first) {
    angNavX = GetAng();
  }
  const vec::Vector2D axisRelPos = {relPos.y(), -relPos.x()};
  vec::Vector2D vecRot = rotate(axisRelPos, angNavX);
  vec::Vector2D tagPos;

  // filter out bad IDs
  if (tagId < 1 || tagId > 16) {
    // std::cout << "bad ID" << std::endl;
    return;
  }

  // filter out IDs on the other side of the field
  if ((SideHelper::IsBlue() && FieldConstants::BLUE_TAGS.find(tagId) == FieldConstants::BLUE_TAGS.end()) || (!SideHelper::IsBlue() && FieldConstants::RED_TAGS.find(tagId) == FieldConstants::RED_TAGS.end())) {
    // std::cout << "other side of field" << std::endl;
    return;
  }

  // add tag position to get absolute robot position
  tagPos = FieldConstants::TAGS[tagId - 1];
  tagPos = Utils::InToM(tagPos);
  vec::Vector2D robotPosCams = tagPos - vecRot;

  // reject if apriltag pos is too far away
  vec::Vector2D odomPos = GetPos();
  if (magn(odomPos - robotPosCams) > OdometryConstants::AT_REJECT) {
    // std::cout << "too faar" << std::endl;
    return;
  }

  // check if outside field
  double margin = Utils::InToM(FieldConstants::FIELD_MARGIN);
  double fWidth = Utils::InToM(FieldConstants::FIELD_WIDTH);
  double fHeight = Utils::InToM(FieldConstants::FIELD_HEIGHT);
  if (robotPosCams.x() < -margin || robotPosCams.x() > fWidth + margin
   || robotPosCams.y() < -margin || robotPosCams.y() > fHeight + margin) {
    // std::cout << "outside field" << std::endl;
    return;
  }

  if (m_shuffleboard) {
    frc::SmartDashboard::PutString("Raw Cam Pos", relPos.toString());
    frc::SmartDashboard::PutString("Robot To Tag", vecRot.toString());
    frc::SmartDashboard::PutNumber("Hist Ang Nav X", angNavX);
  }

  // update cams in pose estimator
  double stdDev = m_camStdDevCoef /* * magn(vecRot) * magn(vecRot) */ + m_turnStdDevCoef * GetAngVel();
  if (magn(odomPos - robotPosCams) > OdometryConstants::TRUST_CAMS_MORE_THRESH && m_trustCamsMore) {
    stdDev = 0;
  }

  m_estimator.UpdateCams(camTime, robotPosCams, {stdDev, stdDev});
  m_camPos = robotPosCams;

  // update prev cam time
  m_prevCamTime = curTime;
}

void Odometry::ShuffleboardInit() {
  if (!m_shuffleboard) {
    return;
  }

  frc::SmartDashboard::PutNumber("Pos stddev", OdometryConstants::SYS_STD_DEV_TELE.x());
  frc::SmartDashboard::PutNumber("Cam stddev", m_camStdDevCoef);
  frc::SmartDashboard::PutNumber("Time offset", m_timeOffset);
  frc::SmartDashboard::PutNumber("Cam turn stddev", m_turnStdDevCoef);
}

/**
 * Shuffleboard update, for debug
*/
void Odometry::ShuffleboardPeriodic() {
  if (!m_shuffleboard) {
    return;
  }

  double def = m_isAuto ? OdometryConstants::CAM_STD_DEV_COEF_AUTO : OdometryConstants::CAM_STD_DEV_COEF_TELE;
  double stdDev = frc::SmartDashboard::GetNumber("Pos stddev", def);
  m_camStdDevCoef = frc::SmartDashboard::GetNumber("Cam stddev", m_camStdDevCoef);
  m_turnStdDevCoef = frc::SmartDashboard::GetNumber("Cam turn stddev", m_turnStdDevCoef);
  m_timeOffset = frc::SmartDashboard::GetNumber("Time offset", m_timeOffset);
  m_estimator.SetQ({stdDev, stdDev});

  vec::Vector2D pos = GetPos();
  double ang = GetAng();
  // m_field.SetRobotPose(frc::Pose2d{units::meter_t{x(pos)}, units::meter_t{y(pos)}, units::radian_t{ang}});
  // frc::SmartDashboard::PutData("Field", &m_field);
  frc::SmartDashboard::PutString("Robot Pos From Cam", m_camPos.toString());
  frc::SmartDashboard::PutNumber("Cur Yaw", m_curYaw);
}