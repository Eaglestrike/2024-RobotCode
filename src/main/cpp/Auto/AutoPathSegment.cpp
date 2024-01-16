#include "Auto/AutoPathSegment.h"

#include <algorithm>
#include <cmath>

#include "Constants/AutoConstants.h"
#include "Util/AutoPathReader.h"
#include "Util/Utils.h"

/**
 * Auto Path Segment constructor
 * 
 * @param swerve Swerve object
 * @param odom Odometry object
*/
AutoPathSegment::AutoPathSegment(SwerveControl &swerve, Odometry &odom)
  : m_posSpline{1000}, m_angSpline{1000}, m_swerve{swerve}, m_odom{odom}, m_startTime{0},
  m_hasStarted{false},
  m_posCorrect{AutoConstants::DRIVE_P, AutoConstants::DRIVE_I, AutoConstants::DRIVE_D},
  m_angCorrect{AutoConstants::ANG_P, AutoConstants::ANG_I, AutoConstants::ANG_D}
  {}

/**
 * Sets auto path
 * 
 * @param path Filename of auto path 
*/
void AutoPathSegment::SetAutoPath(const std::string path) {
  AutoPathReader reader{path};
  m_posSpline = reader.GetSplinePos();
  m_angSpline = reader.GetSplineAng();
}

/**
 * Starts auto path
*/
void AutoPathSegment::Start() {
  m_startTime = Utils::GetCurTimeS();
  m_hasStarted = true;
}

/**
 * Sets drive PID
 * 
 * @param kP P
 * @param kI I
 * @param kD D
*/
void AutoPathSegment::SetDrivePID(double kP, double kI, double kD) {
  m_posCorrect.SetPID(kP, kI, kD);
}

/**
 * Sets angle PID
 * 
 * @param kP P
 * @param kI I
 * @param kD D
*/
void AutoPathSegment::SetAngPID(double kP, double kI, double kD) {
  m_angCorrect.SetPID(kP, kI, kD);
}

/**
 * Periodic
*/
void AutoPathSegment::Periodic() {
  // get relative time
  double curTimeRel = Utils::GetCurTimeS() - m_startTime;
  const bool outOfBounds = curTimeRel < 0 || curTimeRel > m_posSpline.getHighestTime();
  curTimeRel = curTimeRel < 0 ? 0 : (curTimeRel > m_posSpline.getHighestTime() ? m_posSpline.getHighestTime() : curTimeRel);

  // get current expected position
  const vec::Vector2D curExpectedPos = m_posSpline(curTimeRel);
  const double curExpectedAng = 0; // only relying on feedback for ang

  // get feed forward velocity 
  const vec::Vector2D curVel = m_posSpline.getVel(curTimeRel);
  const double curAngVel = m_angSpline.getVel(curTimeRel)[0];

  // get current pos
  const vec::Vector2D curPos = m_odom.GetPos();
  const double curAng = m_odom.GetAng();

  // get correction velocity
  const double correctVelX = m_posCorrect.Calculate(curPos.x(), curExpectedPos.x());
  const double correctVelY = m_posCorrect.Calculate(curPos.y(), curExpectedPos.y());
  const double correctAngVel = m_angCorrect.Calculate(curAng, curExpectedAng);
  const vec::Vector2D correctVel = {correctVelX, correctVelY};

  // set velocity to swerve
  const vec::Vector2D setVel = curVel + correctVel;
  const double setAngVel = curAngVel + correctAngVel;
  m_swerve.SetRobotVelocity(setVel, setAngVel, curAng);
}

/**
 * Gets progress of hermite spline
 * 
 * @note If at 100%, does not mean that it's at target because of PID corrections.
 * Use AtTarget() to check that instead.
 * 
 * @returns Decimal number from 0 to 1 representing fraction of completion.
 * If not started or after completion, will be 1.
*/
double AutoPathSegment::GetProgress() const {
  return std::clamp(GetAbsProgress(), 0.0, 1.0);
}

/**
 * Determines whether hermite auto path is done
 * 
 * @note If true, does not mean that it's at target because of PID corrections.
 * Use AtTarget() to check that instead.
 * 
 * @returns If auto path is done.
*/
bool AutoPathSegment::IsDoneHermite() const {
  if (!m_hasStarted) {
    return false;
  }

  return GetAbsProgress() >= 1.0;
}

/**
 * If the robot is translationally within tolerance of target
 * 
 * @note If hermite spline is not done then returns false
 * 
 * @returns If robot is within translational tolerance of target
*/
bool AutoPathSegment::AtPosTarget() const {
  if (!IsDoneHermite()) {
    return false;
  }

  const vec::Vector2D curPos = m_odom.GetPos();
  const vec::Vector2D targPos = m_posSpline(m_posSpline.getHighestTime());

  return magn(targPos - curPos) < AutoConstants::POS_TOL;
}

/**
 * If the robot is angularly within tolerance of target
 * 
 * @note If hermite spline is not done then returns false
 * 
 * @returns If robot is within angularly tolerance of target
*/
bool AutoPathSegment::AtAngTarget() const {
  if (!IsDoneHermite()) {
    return false;
  }

  const bool curAng = m_odom.GetAng();
  const bool targAng = m_angSpline(m_angSpline.getHighestTime())[0];

  return std::abs(targAng - curAng) < AutoConstants::ANG_TOL;
}

/**
 * If the robot is within tolerance of target
 * 
 * @note If hermite spline is not done then returns false
 * 
 * @returns If robot is within tolerance of target
*/
bool AutoPathSegment::AtTarget() const {
  return AtPosTarget() && AtAngTarget();
}

/**
 * Gets absolute progress
 * 
 * Means that progress is not clamped (if after completion, will return number greater than 1)
 * 
 * @returns Abs progress
*/
double AutoPathSegment::GetAbsProgress() const {
  double endTime = m_posSpline.getHighestTime();
  double curRelTime = Utils::GetCurTimeS() - m_startTime;

  return curRelTime / endTime;
}