#include "Auto/AutoPathSegment.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include <frc/smartdashboard/SmartDashboard.h>

#include "Constants/AutoConstants.h"
#include "Util/AutoPathReader.h"
#include "Util/Utils.h"
#include "Util/SideHelper.h"

/**
 * Auto Path Segment constructor
 * 
 * @param swerve Swerve object
 * @param odom Odometry object
*/
AutoPathSegment::AutoPathSegment(bool shuffleboard, SwerveControl &swerve, Odometry &odom)
  : m_shuffleboard{shuffleboard},
  m_spline{.pos{1000}, .ang{1000}},
  m_swerve{swerve}, m_odom{odom},
  m_startTime{0},
  m_hasStarted{false},
  m_posCorrectX{{AutoConstants::DRIVE_P, AutoConstants::DRIVE_I, AutoConstants::DRIVE_D}},
  m_posCorrectY{{AutoConstants::DRIVE_P, AutoConstants::DRIVE_I, AutoConstants::DRIVE_D}},
  m_angCorrect{{AutoConstants::ANG_P, AutoConstants::ANG_I, AutoConstants::ANG_D}},
  m_shuff{"Auto Segment", shuffleboard},
  m_isDonePos{false},
  m_isDoneAng{false}
{
  m_posCorrectX.SetTolerance(AutoConstants::POS_TOL, std::numeric_limits<double>::infinity());
  m_posCorrectY.SetTolerance(AutoConstants::POS_TOL, std::numeric_limits<double>::infinity());
  m_angCorrect.SetTolerance(AutoConstants::ANG_TOL, std::numeric_limits<double>::infinity());

  m_hasSpline = false;
}

/**
 * Loads auto path
 * 
 * @param path Filename of auto path 
*/
bool AutoPathSegment::LoadAutoPath(const std::string path) {
  if(m_loadedSplines.find(path) != m_loadedSplines.end()){
    return true;
  }
  AutoPathReader reader{path};
  if (!reader.FileExists()) {
    return false;
  }
  m_loadedSplines.insert({path, {.pos = reader.GetSplinePos(), .ang = reader.GetSplineAng()}});
  return true;
}

/**
 * Sets auto path
 * 
 * @param path Filename of auto path 
*/
void AutoPathSegment::SetAutoPath(const std::string path) {
  m_hasSpline = LoadAutoPath(path);
  if(m_hasSpline){
    m_blueSpline = m_loadedSplines[path];
    m_spline.pos = SideHelper::GetSplinePos(m_blueSpline.pos);
    m_spline.ang = SideHelper::GetSplineAng(m_blueSpline.ang);
  }
  else{
    Clear();
  }
}

/**
 * Starts auto path
*/
void AutoPathSegment::Start() {
  m_startTime = Utils::GetCurTimeS();
  if(!m_hasSpline){
    m_hasStarted = false;
    return;
  }
  
  m_spline.pos = SideHelper::GetSplinePos(m_blueSpline.pos);
  m_spline.ang = SideHelper::GetSplineAng(m_blueSpline.ang);
  
  m_hasStarted = true;

  m_isDonePos = false;
  m_isDoneAng = false;
  double pTol = m_shuff.GetNumber("pos tol", AutoConstants::POS_TOL);
  SetDriveTol(pTol);
  double aTol = m_shuff.GetNumber("ang tol", AutoConstants::ANG_TOL);
  SetAngTol(aTol);

  m_posCorrectX.Reset();
  m_posCorrectY.Reset();
  m_angCorrect.Reset();
}

/**
 * Stops auto path
*/
void AutoPathSegment::Stop() {
  m_hasStarted = false;

  m_swerve.SetRobotVelocity({0.0, 0.0}, 0.0, m_odom.GetAng());
}

/**
 * Clears the path
*/
void AutoPathSegment::Clear(){
  m_startTime = 0.0;
  m_spline.pos = {1000};
  m_spline.ang = {1000};
  m_hasSpline = false;
  Stop();
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
 * Periodic (angular and linear motion)
*/
void AutoPathSegment::Periodic(){
  if(!m_hasStarted){
    // std::cout<<"Has not started segment"<<std::endl;
    return;
  }
  // get relative time
  double curTimeRel = Utils::GetCurTimeS() - m_startTime;
  curTimeRel = curTimeRel < 0 ? 0 : (curTimeRel > m_spline.pos.getHighestTime() ? m_spline.pos.getHighestTime() : curTimeRel);
  
  //Deal with angular correction
  const double curExpectedAng = m_spline.ang(curTimeRel)[0];
  const double curExpectedAngVel = m_spline.ang.getVel(curTimeRel)[0]; // only feedback -> set to 0

  const double curAng = m_odom.GetAng();
  const double curAngVel = m_swerve.GetRobotAngularVel();
  
  Poses::Pose1D angPose = {curAng, curAngVel, 0.0};
  const double correctAngVel = m_angCorrect.Calculate(angPose, {curExpectedAng, curExpectedAngVel, 0.0});

  if(m_shuffleboard){
    m_shuff.PutNumber("error ang", curExpectedAng - curAng, {1,1,6,4});
  }
  
  double setAngVel = curExpectedAngVel + correctAngVel;

  // CALEB FILTER
  // if (AtAngTarget()) {
  //   setAngVel = 0;
  // }

  // JONATHAN FILTER
  if (AtAngTarget()) {
    if (!m_isDoneAng) {
      m_isDoneAng = true;
      double aTol = m_shuff.GetNumber("ang tol big", AutoConstants::ANG_TOL_BIG);
      SetAngTol(aTol);
    }
    m_angCorrect.Reset(); // ???
    setAngVel = 0;
  } else {
    if (m_isDoneAng) {
      m_isDoneAng = false;
      double aTol = m_shuff.GetNumber("ang tol", AutoConstants::ANG_TOL);
      SetAngTol(aTol);
    }
  }

  Periodic(setAngVel);
}

/**
 * Periodic (only linear motion)
*/
void AutoPathSegment::Periodic(double angVel) {
  // get current pos
  const double curAng = m_odom.GetAng();
  const vec::Vector2D curPos = m_odom.GetPos();

  if(!m_hasStarted){
    m_swerve.SetRobotVelocity({0.0, 0.0}, angVel, curAng);
    return;
  }
  // get relative time
  double curTimeRel = Utils::GetCurTimeS() - m_startTime;
  curTimeRel = curTimeRel < 0 ? 0 : (curTimeRel > m_spline.pos.getHighestTime() ? m_spline.pos.getHighestTime() : curTimeRel);

  // get current expected position
  const vec::Vector2D curExpectedPos = m_spline.pos(curTimeRel);

  // get feed forward velocity 
  const vec::Vector2D curExpectedVel = m_spline.pos.getVel(curTimeRel);

  const vec::Vector2D curVel = m_swerve.GetRobotVelocity(curAng);

  Poses::Pose1D xPose = {curPos.x(), curVel.x(), 0.0};
  Poses::Pose1D yPose = {curPos.y(), curVel.y(), 0.0};

  // get correction velocity
  const double correctVelX = m_posCorrectX.Calculate(xPose, {curExpectedPos.x(), curExpectedVel.x(), 0.0});
  const double correctVelY = m_posCorrectY.Calculate(yPose, {curExpectedPos.y(), curExpectedVel.y(), 0.0});
  const vec::Vector2D correctVel = {correctVelX, correctVelY};

  if (m_shuffleboard) {
    m_shuff.PutNumber("error x", curExpectedPos.x() - curPos.x(), {1,1,4,4});
    m_shuff.PutNumber("error y", curExpectedPos.y() - curPos.y(), {1,1,5,4});
  }

  // set velocity to swerve
  vec::Vector2D setVel = curExpectedVel + correctVel;

  // JONATHAN FILTER
  if (AtPosTarget()) {
    if (!m_isDonePos) {
      m_isDonePos = true;
      double pTol = m_shuff.GetNumber("pos tol big", AutoConstants::POS_TOL_BIG);
      SetDriveTol(pTol);
    }
    m_posCorrectX.Reset(); // ???
    m_posCorrectY.Reset(); // ???
    setVel = {0, 0};
  } else {
    if (m_isDonePos) {
      m_isDonePos = false;
      double pTol = m_shuff.GetNumber("pos tol", AutoConstants::POS_TOL);
      SetDriveTol(pTol);
    }
  }

  // CALEB FILTER
  // if(AtPosTarget() && ((curExpectedPos - curPos).magn() < AutoConstants::JITTER_FILTER)){
  //   setVel = {0,0};
  // }

  m_swerve.SetRobotVelocity(setVel, angVel, curAng);
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
 * Gets the duration of the path
 * 
 * @returns the total time elapsed by the path
*/
double AutoPathSegment::GetDuration() const {
  return m_spline.pos.getHighestTime() - m_spline.pos.getLowestTime();
}

/**
 * Gets the position at time t
 * 
 * @param t non-relative time (utils frame)
*/
vec::Vector2D AutoPathSegment::GetPos(double t) const{
  double timeRel = t - m_startTime;
  timeRel = timeRel < 0 ? 0 : (timeRel > m_spline.pos.getHighestTime() ? m_spline.pos.getHighestTime() : timeRel);
  return m_spline.pos.getPos(timeRel);
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
  double endTime = m_spline.pos.getHighestTime();
  double curRelTime = Utils::GetCurTimeS() - m_startTime;

  return curRelTime / endTime;
}

void AutoPathSegment::ShuffleboardInit() {
  if (!m_shuffleboard) {
    return;
  }

  m_shuff.PutNumber("trans kP", AutoConstants::DRIVE_P, {1,1,0,0});
  m_shuff.PutNumber("trans kI", AutoConstants::DRIVE_I, {1,1,1,0});
  m_shuff.PutNumber("trans kD", AutoConstants::DRIVE_D, {1,1,2,0});

  m_shuff.PutNumber("turn kP", AutoConstants::ANG_P, {1,1,0,1});
  m_shuff.PutNumber("turn kI", AutoConstants::ANG_I, {1,1,1,1});
  m_shuff.PutNumber("turn kD", AutoConstants::ANG_D, {1,1,2,1});

  m_shuff.PutNumber("pos tol", AutoConstants::POS_TOL, {1,1,0,2});
  m_shuff.PutNumber("ang tol", AutoConstants::ANG_TOL, {1,1,1,2});
  m_shuff.PutNumber("pos tol big", AutoConstants::POS_TOL_BIG, {1,1,2,2});
  m_shuff.PutNumber("ang tol big", AutoConstants::ANG_TOL_BIG, {1,1,3,2});
}

void AutoPathSegment::ShuffleboardPeriodic() {
  if (!m_shuffleboard) {
    return;
  }

  double kP = m_shuff.GetNumber("trans kP", AutoConstants::DRIVE_P);
  double kI = m_shuff.GetNumber("trans kI", AutoConstants::DRIVE_I);
  double kD = m_shuff.GetNumber("trans kD", AutoConstants::DRIVE_D);
  SetDrivePID(kP, kI, kD);

  double wkP =  m_shuff.GetNumber("turn kP", AutoConstants::ANG_P);
  double wkI =  m_shuff.GetNumber("turn kI", AutoConstants::ANG_I);
  double wkD =  m_shuff.GetNumber("turn kD", AutoConstants::ANG_D);
  SetAngPID(wkP, wkI, wkD);

  // double pTol = m_shuff.GetNumber("pos tol", AutoConstants::POS_TOL);
  // SetDriveTol(pTol);
  // double aTol = m_shuff.GetNumber("ang tol", AutoConstants::ANG_TOL);
  // SetAngTol(aTol);

  m_shuff.PutBoolean("At Target", AtTarget(), {3,3,5,0});

  m_shuff.update(true);
}