#include "Drive/AutoHmLineup.h"

#include <cmath>
#include <vector>

#include "Constants/AutoConstants.h"
#include "Util/Utils.h"

/**
 * constructor
 * 
 * @param shuffleboard If shuffleboard is enabled
 * @param odom Odometry
*/
AutoHmLineup::AutoHmLineup(bool shuffleboard, Odometry &odom):
  m_shuff{"AutoHmLineup", shuffleboard},
  m_odom{odom},
  m_posSpline{1000},
  m_angSpline{1000},
  m_targAng{0},
  m_hasStarted{false},
  m_posCorrectX{{AutoConstants::DRIVE_P, AutoConstants::DRIVE_I, AutoConstants::DRIVE_D}},
  m_posCorrectY{{AutoConstants::DRIVE_P, AutoConstants::DRIVE_I, AutoConstants::DRIVE_D}},
  m_angCorrect{{AutoConstants::DRIVE_P, AutoConstants::DRIVE_I, AutoConstants::DRIVE_D}},
  m_startTime{0},
  m_kTime{1},
  m_curExpAngVel{0},
  m_isDonePos{false},
  m_isDoneAng{false}
{
  m_posCorrectX.SetTolerance(AutoConstants::AMP_POS_TOL, std::numeric_limits<double>::infinity());
  m_posCorrectY.SetTolerance(AutoConstants::AMP_POS_TOL, std::numeric_limits<double>::infinity());
  m_angCorrect.SetTolerance(AutoConstants::AMP_ANG_TOL, std::numeric_limits<double>::infinity());
}

/**
 * Start lineup
*/
void AutoHmLineup::Start() {
  if (m_kTime <= 0) {
    return;
  }

  // find difference for time
  double diffMagn = magn(m_targPos - m_odom.GetPos()) * m_kTime;

  if (Utils::NearZero(diffMagn) || diffMagn < 0) {
    return;
  }

  // reset spline
  m_posSpline = {1000};
  m_angSpline = {1000};

  // load numbers into pos spline
  m_posSpline.insert({0, m_odom.GetPos(), m_odom.GetDBVel()});
  m_posSpline.insert({diffMagn, m_targPos, {0, 0}});

  // normalize angle
  double curAng = m_odom.GetAng();
  int mult = static_cast<int>(std::floor(curAng / (2 * M_PI)));
  double ang1 = m_targAng + mult * 2 * M_PI;
  double ang2 = m_targAng + (mult - 1) * 2 * M_PI;
  double ang3 = m_targAng + (mult + 1) * 2 * M_PI;

  // find closest angle
  std::vector<double> angles = {ang1, ang2, ang3};
  int minIdx = 0;
  double curMin = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < angles.size(); i++) {
    if (std::abs(angles[i] - curAng) < curMin) {
      curMin = std::abs(angles[i] - curAng);
      minIdx = i;
    } 
  }

  // load ang into ang spline
  double targAngAbs = angles[minIdx];
  m_angSpline.insert({0, {m_odom.GetAng()}, {m_odom.GetAngVel()}});
  m_angSpline.insert({diffMagn, {targAngAbs}, {0}});

  m_hasStarted = true;
}

/**
 * stop lineup
*/
void AutoHmLineup::Stop() {
  m_hasStarted = false;
}

/**
 * Gets expected position
 * 
 * @returns Expected pos
*/
vec::Vector2D AutoHmLineup::GetExpPos() const {
  double t = Utils::GetCurTimeS() - m_startTime; 

  // if out of bounds
  if (t < 0) {
    return m_posSpline.getPos(0);
  }
  if (t > m_posSpline.getHighestTime()) {
    return m_posSpline.getPos(m_posSpline.getHighestTime());
  }

  return m_posSpline.getPos(t);
}

/**
 * Gets position taarget
 * 
 * @return pos target
*/
vec::Vector2D AutoHmLineup::GetTargPos() const {
  return m_targPos;
}

/**
 * Gets expected ang
 * 
 * @returns expected ang
*/
double AutoHmLineup::GetExpAng() const {
  double t = Utils::GetCurTimeS() - m_startTime; 

  // if out of bounds
  if (t < 0) {
    return m_angSpline.getPos(0)[0];
  }
  if (t > m_angSpline.getHighestTime()) {
    return m_angSpline.getPos(m_angSpline.getHighestTime())[0];
  }

  return m_angSpline.getPos(t)[0];
}

/**
 * Gets expected translational velocity
 * 
 * @returns expected velocity
*/
vec::Vector2D AutoHmLineup::GetExpVel() const {
  if (!m_hasStarted) {
    return {0, 0};
  }

  return m_curExpVel;
}

/**
 * Gets expected angular velocity
 * 
 * @returns expected velocity
*/
double AutoHmLineup::GetExpAngVel() const {
  if (!m_hasStarted) {
    return 0;
  }

  return m_curExpAngVel;
}

/**
 * Sets target
 * 
 * @param tgtPos target position
 * @param tgtAng target angle
*/
void AutoHmLineup::SetTarget(const vec::Vector2D &tgtPos, const double &tgtAng) {
  m_targPos = tgtPos;
  m_targAng = tgtAng;
}

/**
 * If has started
 * 
 * @returns has started
*/
bool AutoHmLineup::HasStarted() const {
  return m_hasStarted;
}

/**
 * Periodic
*/
void AutoHmLineup::Periodic() {
  if (!m_hasStarted) {
    return;
  }

  // relative time
  double t = Utils::GetCurTimeS() - m_startTime;
  t = t < 0 ? 0 : (t > m_posSpline.getHighestTime() ? m_posSpline.getHighestTime() : t);

  // current expected position and ff velocity
  const double curExpectedAng = m_angSpline.getPos(t)[0];
  const double curExpectedAngVel = m_angSpline.getVel(t)[0];

  // current ang
  const double curAng = m_odom.GetAng();
  const double curAngVel = m_odom.GetAngVel();

  Poses::Pose1D angPose = {curAng, curAngVel, 0.0};
  const double correctAngVel = m_angCorrect.Calculate(angPose, {curExpectedAng, curExpectedAngVel, 0.0});

  if (m_shuff.isEnabled()) {
    m_shuff.PutNumber("error ang", curExpectedAng - curAng, {1,1,6,4});
  }

  double setAngVel = curExpectedAngVel + correctAngVel;

  if (AtAngTarget()) {
    if (!m_isDoneAng) {
      m_isDoneAng = true;
      m_angCorrect.SetTolerance(AutoConstants::ANG_TOL_BIG, std::numeric_limits<double>::infinity());
    }
    m_angCorrect.Reset();
    setAngVel = 0;
  } else {
    if (m_isDoneAng) {
      m_isDoneAng = false;
      double aTol = m_shuff.GetNumber("ang tol", AutoConstants::ANG_TOL);
      m_angCorrect.SetTolerance(AutoConstants::ANG_TOL, std::numeric_limits<double>::infinity());
    }
  }

  Periodic(setAngVel);
}

/**
 * Periodic with custom ang vel
 * 
 * @param angVel ang vel
*/
void AutoHmLineup::Periodic(double angVel) {
  if (!m_hasStarted) {
    return;
  }

  // relative time
  double t = Utils::GetCurTimeS() - m_startTime;
  t = t < 0 ? 0 : (t > m_posSpline.getHighestTime() ? m_posSpline.getHighestTime() : t);

  // current expected position and ff velocity
  const vec::Vector2D curExpectedPos = m_posSpline.getPos(t);
  const vec::Vector2D curExpectedVel = m_posSpline.getVel(t);

  // current pos
  const vec::Vector2D curPos = m_odom.GetPos();
  const vec::Vector2D curVel = m_odom.GetDBVel();
  Poses::Pose1D xPose = {curPos.x(), curVel.x(), 0.0};
  Poses::Pose1D yPose = {curPos.y(), curVel.y(), 0.0}; 

  // get correction velocity
  const double correctVelX = m_posCorrectX.Calculate(xPose, {curExpectedPos.x(), curExpectedVel.x(), 0.0});
  const double correctVelY = m_posCorrectY.Calculate(yPose, {curExpectedPos.y(), curExpectedVel.y(), 0.0});
  const vec::Vector2D correctVel = {correctVelX, correctVelY};

  if (m_shuff.isEnabled()) {
    m_shuff.PutNumber("error x", curExpectedPos.x() - curPos.x(), {1,1,4,4});
    m_shuff.PutNumber("error y", curExpectedPos.y() - curPos.y(), {1,1,5,4});
  }

  // set velocity to swerve
  vec::Vector2D setVel = curExpectedVel + correctVel;

  if (AtPosTarget()) {
    if (!m_isDonePos) {
      m_isDonePos = true;
      m_posCorrectX.SetTolerance(AutoConstants::POS_TOL_BIG, std::numeric_limits<double>::infinity());
      m_posCorrectY.SetTolerance(AutoConstants::POS_TOL_BIG, std::numeric_limits<double>::infinity());
    }

    m_posCorrectX.Reset();
    m_posCorrectY.Reset();
    setVel = {0, 0};
  } else {
    if (m_isDonePos) {
      m_isDonePos = false;
      m_posCorrectX.SetTolerance(AutoConstants::POS_TOL, std::numeric_limits<double>::infinity());
      m_posCorrectY.SetTolerance(AutoConstants::POS_TOL, std::numeric_limits<double>::infinity());
    }
  }

  m_curExpVel = setVel;
  m_curExpAngVel = angVel;
}

/**
 * Gets epxected position
 * 
 * @returns expected position
*/
vec::Vector2D AutoHmLineup::GetPos() const {
  double t = Utils::GetCurTimeS() - m_startTime; 

  // if out of bounds
  if (t < 0) {
    return m_posSpline.getPos(m_posSpline.getLowestTime());
  }
  if (t > m_posSpline.getHighestTime()) {
    return m_posSpline.getPos(m_posSpline.getHighestTime());
  }

  return m_posSpline.getPos(t);
}

/**
 * Determines if hermite spline is done
 * 
 * @returns if hermite spline is done
*/
bool AutoHmLineup::IsDoneHermite() const {
  if (!m_hasStarted) {
    return false;
  }

  double t = Utils::GetCurTimeS() - m_startTime;
  return t >= m_posSpline.getHighestTime();
}

/**
 * if at position target
 * 
 * @returns if at pos target
*/
bool AutoHmLineup::AtPosTarget() const {
  if (!IsDoneHermite()) {
    return false;
  }

  return m_posCorrectX.AtTarget() && m_posCorrectY.AtTarget();
}

/**
 * if at ang target
 * 
 * @returns if at ang target
*/
bool AutoHmLineup::AtAngTarget() const {
  if (!IsDoneHermite()) {
    return false;
  }

  return m_angCorrect.AtTarget();
}

/**
 * if at target
 * 
 * @returns at target
*/
bool AutoHmLineup::AtTarget() const {
  return AtPosTarget() && AtAngTarget();
}

/**
 * shuffleboard init
*/
void AutoHmLineup::ShuffleboardInit() {
  if (!m_shuff.isEnabled()) {
    return;
  }

  m_shuff.add("k time", &m_kTime, {1,1,0,0}, true);
  
  m_shuff.PutNumber("targ x", 0.0, {1,1,0,1});
  m_shuff.PutNumber("targ y", 0.0, {1,1,1,1});
  m_shuff.PutNumber("targ ang", 0.0, {1,1,2,1});
  m_shuff.addButton("Set Target",
    [&]{
      SetTarget({m_shuff.GetNumber("targ x", 0.0), m_shuff.GetNumber("targ y", 0.0)}, m_shuff.GetNumber("targ ang", 0.0));
      Stop();
      std::cout<<"Set AutoHm Target"<<std::endl;
    },
    {1,1,3,1}
  );

  m_shuff.PutNumber("p", m_posCorrectX.GetPID().kp, {1,1,0,2});
  m_shuff.PutNumber("i", m_posCorrectX.GetPID().ki, {1,1,1,2});
  m_shuff.PutNumber("d", m_posCorrectX.GetPID().kd, {1,1,2,2});
  m_shuff.addButton("Set PID",
    [&]{
      double p = m_shuff.GetNumber("p", 0.0);
      double i = m_shuff.GetNumber("i", 0.0);
      double d = m_shuff.GetNumber("d", 0.0);
      m_posCorrectX.SetPID({p, i, d});
      m_posCorrectY.SetPID({p, i, d});
      std::cout<<"Set AutoHm PID"<<std::endl;
    },
    {1,1,3,2}
  );
}

/**
 * shuffleboard periodic
*/
void AutoHmLineup::ShuffleboardPeriodic() {
  if (!m_shuff.isEnabled()) {
    return;
  }
 
  m_shuff.PutBoolean("At Target", AtTarget(), {3,3,5,0});

  m_shuff.update(true);
}