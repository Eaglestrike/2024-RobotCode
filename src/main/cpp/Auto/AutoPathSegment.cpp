#include "Auto/AutoPathSegment.h"

#include <algorithm>

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
  const double curTimeRel = Utils::GetCurTimeS() - m_startTime;

  // get current expected position
  const vec::Vector2D curExpectedPos = m_posSpline(curTimeRel);
  const double curExpectedAng = m_angSpline(curTimeRel)[0];

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
 * Gets progress of spline
 * 
 * @returns Decimal number from 0 to 1 representing fraction of completion.
 * If not started or after completion, will be 1.
*/
double AutoPathSegment::GetProgress() const {
  return std::clamp(GetAbsProgress(), 0.0, 1.0);
}

/**
 * Determines whether auto path is done
 * 
 * @returns If auto path is done.
*/
bool AutoPathSegment::IsDone() const {
  if (!m_hasStarted) {
    return false;
  }

  return GetAbsProgress() >= 1.0;
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