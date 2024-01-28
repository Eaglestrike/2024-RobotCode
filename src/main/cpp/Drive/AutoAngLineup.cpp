#include "Drive/AutoAngLineup.h"

#include <climits>
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>

#include "Constants/AutoLineupConstants.h"
#include "Util/Utils.h"

/**
 * Constructor
 * 
 * @param shuffleboard whether shuffleboard enabled
 * @param odom Odometry
*/
AutoAngLineup::AutoAngLineup(bool shuffleboard, Odometry &odom)
    : m_shuffleboard{shuffleboard}, m_odom{odom}, m_angTimes{0, 0, 0, 0},
    m_state{NOT_EXECUTING}, m_maxSpeed{AutoLineupConstants::MAX_SPEED}, m_maxAccel{AutoLineupConstants::MAX_ACCEL},
    m_kP{AutoLineupConstants::ANG_P}, m_kI{AutoLineupConstants::ANG_I}, m_kD{AutoLineupConstants::ANG_D},
    m_targetAng{0}, m_curExpectedAng{0}, m_curExpectedAngVel{0}, m_prevAngErr{0}, m_totalAngErr{0},
    m_angVecDir{0}, m_prevTime{0}, m_curAngVel{0}
    {}

/**
 * Start execuitng move in angular motion
*/
void AutoAngLineup::Start() {
  if (m_state == EXECUTING_TARGET) {
    return;
  }

  double curAng = m_odom.GetAngNorm();

  // calculates angular distance
  double dist;
  if (std::abs(m_targetAng - curAng) < M_PI) {
    if (curAng < m_targetAng) {
      m_angVecDir = 1;
    } else {
      m_angVecDir = -1;
    }
    dist = std::abs(m_targetAng - curAng);
  } else {
    if (curAng < m_targetAng) {
      m_angVecDir = -1;
    } else {
      m_angVecDir = 1;
    }
    dist = 2 * M_PI - std::abs(m_targetAng - curAng);
  }

  CalcTimes(dist);
  
  m_curExpectedAng = curAng;
  m_prevAngErr = 0;
  m_totalAngErr = 0;
  m_state = EXECUTING_TARGET;
}

/**
 * Stops
*/
void AutoAngLineup::Stop() {
  m_state = NOT_EXECUTING;
}

/**
 * Starts executing the move in one of translational/rotational
 * 
 * @param dist The distance to move
*/
void AutoAngLineup::CalcTimes(double dist) {
  double curT = Utils::GetCurTimeS();

  if (Utils::NearZero(m_maxAccel) || Utils::NearZero(m_maxSpeed)) {
    return;
  }

  // time to accelerate/decelerate
  double increaseT = m_maxSpeed / m_maxAccel;
  // time to maintain max speed
  double maintainT = 0;
  if (increaseT * m_maxSpeed > dist) {
    // check math, may be wrong
    increaseT = std::sqrt(dist / m_maxAccel);
    maintainT = 0;
  } else {
    maintainT = (dist - increaseT * m_maxSpeed) / m_maxSpeed;
  }

  m_angTimes.startT = curT;
  m_angTimes.maxSpeedT = curT + increaseT;
  m_angTimes.descentT = curT + increaseT + maintainT;
  m_angTimes.endT = curT + increaseT * 2 + maintainT;

  // std::cout << "max speed" << m_maxSpeed << std::endl;
  // std::cout << "max ac" << m_maxAccel << std::endl;
  // std::cout << "startT: " << m_angTimes.startT << std::endl;
  // std::cout << "maxSpeedT: " << m_angTimes.maxSpeedT << std::endl;
  // std::cout << "descentT: " << m_angTimes.descentT << std::endl;
  // std::cout << "endT: " << m_angTimes.endT << std::endl;
}

/**
 * Sets angular position target for robot
 * 
 * @param ang Absolute angle, if rel is true, otherwise delta position
 * @param rel Whether pos parameter should be interpreted as absolute on field or relative to current pos
 * 
 * @note ang parameter will be normalized to -180 to 180 if outside range
*/
void AutoAngLineup::SetTarget(double targAng) {
  if (m_state == EXECUTING_TARGET) {
    return;
  }

  m_targetAng = Utils::NormalizeAng(targAng);
}

/**
 * Sets profile config
 * 
 * @param maxSpeed max speed
 * @param maxAccel max accel
*/
void AutoAngLineup::SetProfileConfig(double maxSpeed, double maxAccel) {
  m_maxSpeed = maxSpeed;
  m_maxAccel = maxAccel;
}

/**
 * SEts PID
 * 
 * @param kP p
 * @param kI i
 * @param kD d
*/
void AutoAngLineup::SetPID(double kP, double kI, double kD) {
  m_kP = kP;
  m_kI = kI;
  m_kD = kD;
}

/**
 * Calculates positional PID for angular motion
 * 
 * @param deltaT time diffrence
 * 
 * @returns Velcoity from PID
*/
double AutoAngLineup::CalcPID(double deltaT) {
  double err = CalcError();

  // double deltaErr = (err - m_prevAngErr) / deltaT;
  double deltaErr = (m_curExpectedAngVel - m_odom.GetAngVel()) / deltaT;
  m_totalAngErr += err * deltaT;
  double res = err * m_kP + m_totalAngErr * m_kI + deltaErr * m_kD;

  m_prevAngErr = err;

  return res;
}

double AutoAngLineup::CalcError() const {
  double err, dir;
  double curAng = m_odom.GetAngNorm();
  return Utils::NormalizeAng(m_curExpectedAng - curAng);
}

/**
 * Periodic
*/
void AutoAngLineup::Periodic() {
  double curTime = Utils::GetCurTimeS();
  double deltaT = curTime - m_prevTime;

  switch (m_state) {
    case NOT_EXECUTING:
      m_curAngVel = 0;
      // m_dist = 0;
      break;
    case EXECUTING_TARGET:
    {
      double angSpeed = GetSpeed();
      double ffAngVel = m_angVecDir * angSpeed;
      m_curExpectedAngVel = ffAngVel;

      m_curExpectedAng += ffAngVel * deltaT;
      m_curExpectedAng = Utils::NormalizeAng(m_curExpectedAng);

      // double correctionVel = 0;
      double correctionVel = CalcPID(deltaT);
      double totalVel = ffAngVel + correctionVel;

      // std::cout << "ang vel: " << m_angVecDir << std::endl;

      // frc::SmartDashboard::PutNumber("taarget ang", m_targetAng);
      // frc::SmartDashboard::PutNumber("cur speed", angSpeed);
      // frc::SmartDashboard::PutNumber("ang dir", m_angVecDir);
      // frc::SmartDashboard::PutNumber("cur dist", m_dist);
      // m_dist += angSpeed * 0.02;

      m_curAngVel = totalVel;

      if (AtAngTarget()) {
        m_state = AT_TARGET;
      }
      break;
    }
    case AT_TARGET:
    {
      m_curAngVel = 0;
      if (!AtAngTarget()) {
        m_state = EXECUTING_TARGET;
      }

      break;
    }
  }
  m_prevTime = curTime;
}

/**
 * Gets current speed in one of rotational/translational motion
 * 
 * @returns Speed
*/
double AutoAngLineup::GetSpeed() {
  double curT = Utils::GetCurTimeS();
  if (curT < m_angTimes.startT) {
    // shouldn't be here
    return 0;
  }
  if (curT > m_angTimes.endT) {
    return 0;
  }

  double speed;
  if (curT < m_angTimes.maxSpeedT) {
    speed = m_maxAccel * (curT - m_angTimes.startT);
  } else if (curT < m_angTimes.descentT) {
    speed = m_maxSpeed;
  } else {
    speed = m_maxAccel * (m_angTimes.endT - curT);
  }

  return speed;
}

/**
 * Returns whether robot angular position is at the target
 * 
 * Assumes autoconstant error tolerance
 * 
 * @returns Whether robot is where it is supposed to be angularly at the end of profile
*/
bool AutoAngLineup::AtAngTarget() const {
  return AtAngTarget(AutoLineupConstants::ANG_TOL, std::numeric_limits<double>::infinity());
}

/**
 * Returns wheter robot angular position is at target
 * 
 * @param posErrTol position error tolerance
 * @param velErrTol velocity error tolerance
 * 
 * @returns Wheter robot is where it is supposed to be angularly at the end of profile
*/
bool AutoAngLineup::AtAngTarget(double posErrTol, double velErrTol) const {
  double curAng = m_odom.GetAngNorm();
  return Utils::NearZero(m_targetAng - curAng, posErrTol) && Utils::NearZero(m_curAngVel, velErrTol);
}

/**
 * Shuffleboard init
*/
void AutoAngLineup::ShuffleboardInit() {
  if (!m_shuffleboard) {
    return;
  }

  frc::SmartDashboard::PutNumber("lineup max speed", m_maxSpeed);
  frc::SmartDashboard::PutNumber("lineup max accel", m_maxAccel);
  frc::SmartDashboard::PutNumber("lineup ang kp", m_kP);
  frc::SmartDashboard::PutNumber("lineup ang ki", m_kI);
  frc::SmartDashboard::PutNumber("lineup ang kd", m_kD);
  frc::SmartDashboard::PutNumber("targ ang", m_targetAng);
  frc::SmartDashboard::PutNumber("pos error", 0);
  frc::SmartDashboard::PutNumber("vel error", 0);
  frc::SmartDashboard::PutString("state", GetStateString());
}

/**
 * Shuffleboard Periodic
*/
void AutoAngLineup::ShuffleboardPeriodic() {
  if (!m_shuffleboard) {
    return;
  }

  double maxSpeed = frc::SmartDashboard::GetNumber("lineup max speed", m_maxSpeed);
  double maxAccel = frc::SmartDashboard::GetNumber("lineup max accel", m_maxAccel);
  double kP = frc::SmartDashboard::GetNumber("lineup ang kp", m_kP);
  double kI = frc::SmartDashboard::GetNumber("lineup ang ki", m_kI);
  double kD = frc::SmartDashboard::GetNumber("lineup ang kd", m_kD);
  double targAng = frc::SmartDashboard::GetNumber("targ ang", m_targetAng);

  SetProfileConfig(maxSpeed, maxAccel);
  SetPID(kP, kI, kD);
  SetTarget(targAng);

  frc::SmartDashboard::PutNumber("pos error", CalcError());
  frc::SmartDashboard::PutNumber("cur ang", m_odom.GetAngNorm());
  frc::SmartDashboard::PutNumber("exp ang", m_curExpectedAng);
  frc::SmartDashboard::PutNumber("cur vel", m_curAngVel);
  frc::SmartDashboard::PutString("state", GetStateString());
}

/**
 * Gets angular velocity
*/
double AutoAngLineup::GetAngVel() const {
  return m_curAngVel;
}

/**
 * Gets state
*/
AutoAngLineup::ExecuteState AutoAngLineup::GetState() const {
  return m_state;
}

/**
 * Gets state string
*/
std::string AutoAngLineup::GetStateString() const {
  switch (m_state) {
    case NOT_EXECUTING:
      return "Not executing";
    case EXECUTING_TARGET:
      return "Executing target";
    case AT_TARGET:
      return "At Target";
  }
  return "Error";
}