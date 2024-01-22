#include "Auto/AutoPathSegment.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include <frc/smartdashboard/SmartDashboard.h>

#include "Constants/AutoConstants.h"
#include "Util/AutoPathReader.h"
#include "Util/Utils.h"

/**
 * Auto Path Segment constructor
 * 
 * @param swerve Swerve object
 * @param odom Odometry object
*/
AutoPathSegment::AutoPathSegment(bool shuffleboard, SwerveControl &swerve, Odometry &odom)
  : m_shuffleboard{shuffleboard},
  m_posSpline{1000}, m_angSpline{1000}, m_swerve{swerve}, m_odom{odom}, m_startTime{0},
  m_hasStarted{false},
  m_posCorrectX{{AutoConstants::DRIVE_P, AutoConstants::DRIVE_I, AutoConstants::DRIVE_D}},
  m_posCorrectY{{AutoConstants::DRIVE_P, AutoConstants::DRIVE_I, AutoConstants::DRIVE_D}},
  m_angCorrect{{AutoConstants::ANG_P, AutoConstants::ANG_I, AutoConstants::ANG_D}}
{
  m_posCorrectX.SetTolerance(AutoConstants::POS_TOL, std::numeric_limits<double>::infinity());
  m_posCorrectY.SetTolerance(AutoConstants::POS_TOL, std::numeric_limits<double>::infinity());
  m_angCorrect.SetTolerance(AutoConstants::ANG_TOL, std::numeric_limits<double>::infinity());
}

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
  m_posCorrectX.Reset();
  m_posCorrectY.Reset();
  m_angCorrect.Reset();
}

/**
 * Sets drive PID
 * 
 * @param kP P
 * @param kI I
 * @param kD D
*/
void AutoPathSegment::SetDrivePID(double kP, double kI, double kD) {
  m_posCorrectX.SetPID({kP, kI, kD});
  m_posCorrectY.SetPID({kP, kI, kD});
}

/**
 * Sets angle PID
 * 
 * @param kP P
 * @param kI I
 * @param kD D
*/
void AutoPathSegment::SetAngPID(double kP, double kI, double kD) {
  m_angCorrect.SetPID({kP, kI, kD});
}

/**
 * Sets drive tol
 * 
 * @param tol The tol, in m
*/
void AutoPathSegment::SetDriveTol(double tol) {
  m_posCorrectX.SetTolerance(tol, std::numeric_limits<double>::infinity());
  m_posCorrectY.SetTolerance(tol, std::numeric_limits<double>::infinity());
}

/**
 * Sets ang tol
 * 
 * @param tol The tol, in rad
*/
void AutoPathSegment::SetAngTol(double tol) {
  m_angCorrect.SetTolerance(tol, std::numeric_limits<double>::infinity());
}

/**
 * Periodic
*/
void AutoPathSegment::Periodic() {
  // get relative time
  double curTimeRel = Utils::GetCurTimeS() - m_startTime;
  curTimeRel = curTimeRel < 0 ? 0 : (curTimeRel > m_posSpline.getHighestTime() ? m_posSpline.getHighestTime() : curTimeRel);

  // get current expected position
  const vec::Vector2D curExpectedPos = m_posSpline(curTimeRel);
  const double curExpectedAng = m_angSpline(curTimeRel)[0];

  // get feed forward velocity 
  const vec::Vector2D curExpectedVel = m_posSpline.getVel(curTimeRel);
  const double curExpectedAngVel = m_angSpline.getVel(curTimeRel)[0]; // only feedback -> set to 0

  // get current pos
  const vec::Vector2D curPos = m_odom.GetPos();
  const double curAng = m_odom.GetAng();

  const vec::Vector2D curVel = m_swerve.GetRobotVelocity(curAng);
  const double curAngVel = m_swerve.GetRobotAngularVel();

  Poses::Pose1D xPose = {curPos.x(), curVel.x(), 0.0};
  Poses::Pose1D yPose = {curPos.y(), curVel.y(), 0.0};
  Poses::Pose1D angPose = {curAng, curAngVel, 0.0};

  // get correction velocity
  const double correctVelX = m_posCorrectX.Calculate(xPose, {curExpectedPos.x(), curExpectedVel.x(), 0.0});
  const double correctVelY = m_posCorrectY.Calculate(yPose, {curExpectedPos.y(), curExpectedVel.y(), 0.0});
  const double correctAngVel = m_angCorrect.Calculate(angPose, {curExpectedAng, curExpectedAngVel, 0.0});
  const vec::Vector2D correctVel = {correctVelX, correctVelY};

  if (m_shuffleboard) {
    frc::SmartDashboard::PutNumber("error x", curExpectedPos.x() - curPos.x());
    frc::SmartDashboard::PutNumber("error y", curExpectedPos.y() - curPos.y());
    frc::SmartDashboard::PutNumber("error ang", curExpectedAng - curAng);
  }

  // set velocity to swerve
  vec::Vector2D setVel = curExpectedVel + correctVel;
  double setAngVel = curExpectedAngVel + correctAngVel;

  if (AtPosTarget()) {
    setVel = {0, 0};
  }

  if (AtAngTarget()) {
    setAngVel = 0;
  }

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

  // const vec::Vector2D curPos = m_odom.GetPos();
  // const vec::Vector2D targPos = m_posSpline(m_posSpline.getHighestTime());
  // return magn(targPos - curPos) < m_posCorrectX.GetPositionTolerance();
  return m_posCorrectX.AtTarget() && m_posCorrectY.AtTarget();
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

  // const double curAng = m_odom.GetAng();
  // const double targAng = m_angSpline(m_angSpline.getHighestTime())[0];
  // return std::abs(targAng - curAng) < m_angCorrect.GetPositionTolerance();

  return m_angCorrect.AtTarget();
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

void AutoPathSegment::ShuffleboardInit() {
  if (!m_shuffleboard) {
    return;
  }

  frc::SmartDashboard::PutNumber("trans kP", AutoConstants::DRIVE_P);
  frc::SmartDashboard::PutNumber("trans kI", AutoConstants::DRIVE_I);
  frc::SmartDashboard::PutNumber("trans kD", AutoConstants::DRIVE_D);

  frc::SmartDashboard::PutNumber("turn kP", AutoConstants::ANG_P);
  frc::SmartDashboard::PutNumber("turn kI", AutoConstants::ANG_I);
  frc::SmartDashboard::PutNumber("turn kD", AutoConstants::ANG_D);

  frc::SmartDashboard::PutNumber("pos tol", AutoConstants::POS_TOL);
  frc::SmartDashboard::PutNumber("ang tol", AutoConstants::ANG_TOL);
}

void AutoPathSegment::ShuffleboardPeriodic() {
  if (!m_shuffleboard) {
    return;
  }

  double kP = frc::SmartDashboard::GetNumber("trans kP", AutoConstants::DRIVE_P);
  double kI = frc::SmartDashboard::GetNumber("trans kI", AutoConstants::DRIVE_I);
  double kD = frc::SmartDashboard::GetNumber("trans kD", AutoConstants::DRIVE_D);

  SetDrivePID(kP, kI, kD);

  double wkP =  frc::SmartDashboard::GetNumber("turn kP", AutoConstants::ANG_P);
  double wkI =  frc::SmartDashboard::GetNumber("turn kI", AutoConstants::ANG_I);
  double wkD =  frc::SmartDashboard::GetNumber("turn kD", AutoConstants::ANG_D);

  SetAngPID(wkP, wkI, wkD);

  double pTol = frc::SmartDashboard::GetNumber("pos tol", AutoConstants::POS_TOL);
  SetDriveTol(pTol);
  double aTol = frc::SmartDashboard::GetNumber("ang tol", AutoConstants::ANG_TOL);
  SetAngTol(aTol);

  frc::SmartDashboard::PutBoolean("At Target", AtTarget());
}