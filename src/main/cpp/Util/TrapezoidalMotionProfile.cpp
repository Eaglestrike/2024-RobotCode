#include "Util/TrapezoidalMotionProfile.h"

/**
 * Constructor
 * 
 * @param MAX_VEL maximum velocity
 * @param MAX_ACC max acceleration
 * @param curPos current position
 * @param setPt setpoint
 * 
 * @note If MAX_VEL or MAX_ACC is 0, then sets the value to 1 to avoid calculation errors,
 * and if either is negative, uses absolute value
*/
TrapezoidalMotionProfile::TrapezoidalMotionProfile(double MAX_VEL, double MAX_ACC, double curPos, double setPt): 
m_maxVel{std::abs(MAX_VEL)}, m_maxAcc{std::abs(MAX_ACC)}, m_curTime{-1}{
        SetSetpoint(curPos, setPt);
}

/**
 * Constructor
 * 
 * @param MAX_VEL maximum velocity
 * @param MAX_ACC max acceleration
 * 
 * @note If MAX_VEL or MAX_ACC is 0, then sets the value to 1 to avoid calculation errors,
 * and if either is negative, uses absolute value
*/
TrapezoidalMotionProfile::TrapezoidalMotionProfile(double MAX_VEL, double MAX_ACC): 
m_maxVel{std::abs(MAX_VEL)}, m_maxAcc{std::abs(MAX_ACC)}, m_curTime{-1}{}

/**
 * Sets setpoint and starts profile
 * 
 * @param curPos current position
 * @param setPt setpoint
*/
void TrapezoidalMotionProfile::SetSetpoint(double curPos, double setPt){
    m_setPt = setPt;
    m_targetPos = curPos;
    m_targetVel = 0.0;
    m_targetAcc = m_maxAcc;
    m_curTime = Utils::GetCurTimeS();
    if (setPt < curPos)m_targetAcc *= -1;
    CalcSpeedDecreasePos(curPos, setPt);
}

/**
 * Determines whether profile is at setpoint
 * 
 * @returns Whether profile is at setpoint
*/
bool TrapezoidalMotionProfile::AtSetPoint() const {
    if (m_targetVel == 0.0 && m_curTime != -1) return true;
    return false;
}

/**
 * Calculaates position where velocity should start deceleraating
 * 
 * @param curPos current position
 * @param setPt setpoint
*/
void TrapezoidalMotionProfile::CalcSpeedDecreasePos(double curPos, double setPt){
    if (m_maxAcc <= 0 || m_maxVel <= 0) {
        return;
    }

    if(fabs(setPt - curPos) < m_maxVel*m_maxVel/m_maxAcc){ // for triangle motion profile
        m_speedDecreasePos = (m_setPt+curPos)/2;
    } else if (m_setPt > curPos)
        m_speedDecreasePos = m_setPt - m_maxVel*m_maxVel/(m_maxAcc*2);
    else 
        m_speedDecreasePos = m_setPt + m_maxVel*m_maxVel/(m_maxAcc*2);
}

/**
 * Periodic
*/
void TrapezoidalMotionProfile::Periodic(){
    if (m_maxAcc <= 0 || m_maxVel <= 0) {
        return;
    }

    double newP = m_targetPos, newV = m_targetVel, newA = m_targetAcc;
    
    newP += m_targetVel;
    double newTime = Utils::GetCurTimeS(), timePassed = newTime- m_curTime;
    m_curTime = newTime;

    if (m_speedDecreasePos < m_setPt){ // if trapezoid is pos
        if (m_targetPos > m_speedDecreasePos) // if after turn pt
            newV = std::max(0.0, m_targetVel - m_maxAcc * timePassed);
        else 
            newV = std::min(m_maxVel, m_targetVel + m_maxAcc * timePassed);
    } else {
        if (m_targetPos > m_speedDecreasePos) // if before the turn pt
            newV = std::max(-m_maxVel, m_targetVel - m_maxAcc * timePassed);
        else 
            newV = std::min(0.0, m_targetVel + m_maxAcc * timePassed);
    }

    if (newV-m_targetVel == 0) newA = 0;
    else if (newV > m_targetVel) newA = m_maxAcc;
    else newA = -m_maxAcc;

    m_targetPos = newP;
    m_targetVel = newV;
    m_targetAcc = newA;
}

/**
 * Gets position
 * 
 * @returns Position
*/
double TrapezoidalMotionProfile::GetPosition() const{
    return m_targetPos;
}

/**
 * Gets velocity
 * 
 * @returns velocity
*/
double TrapezoidalMotionProfile::GetVelocity() const{
    return m_targetVel;
}

/**
 * Gets acceleraiton
 * 
 * @returns acel
*/
double TrapezoidalMotionProfile::GetAcceleration() const{
    return m_targetAcc;
}

/**
 * Gets max acceleraiton
 * 
 * @returns maxaccel
*/
double TrapezoidalMotionProfile::GetMaxAcc() const {
    return m_maxAcc;
}

/**
 * Gets max velocity
 * 
 * @returns maxvel
*/
double TrapezoidalMotionProfile::GetMaxVel() const {
    return m_maxVel;
}

/**
 * Sets max velocity
 * 
 * @param maxVel max velocity
*/
void TrapezoidalMotionProfile::SetMaxVel(double maxVel) {
    if (maxVel == 0) {
        return;
    }

    m_maxVel = std::abs(maxVel);
}

/**
 * Sets max accel
 * 
 * @param maxAcc max accel
*/
void TrapezoidalMotionProfile::SetMaxAcc(double maxAcc) {
    if (maxAcc == 0) {
        return;
    }

    m_maxAcc = std::abs(maxAcc);
}