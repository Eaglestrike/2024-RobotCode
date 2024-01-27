#pragma once

#include "Util/Odometry.h"
#include "Util/simplevectors.hpp"

/**
 * Auto angle lineup motion profile, for drivebase only
 * 
 * Shooter auto lineup should be handled outside of this class
*/
class AutoAngLineup{
public:
    enum ExecuteState {
        NOT_EXECUTING,
        EXECUTING_TARGET,
        AT_TARGET
    };

    struct Times {
        double startT;
        double maxSpeedT;
        double descentT;
        double endT;
    };

    AutoAngLineup(bool shuffleboard, Odometry &odom);

    double GetAngVel() const;
    ExecuteState GetState() const;

    void Start();
    void Stop();

    void SetTarget(double targAng);
    void SetProfileConfig(double maxSpeed, double maxAccel);
    void SetPID(double kP, double kI, double kD);

    void Periodic();

    void ShuffleboardInit();
    void ShuffleboardPeriodic();

private:
    bool m_shuffleboard;
    Odometry &m_odom;

    Times m_angTimes;
    ExecuteState m_state;

    double m_maxSpeed, m_maxAccel;
    double m_kP, m_kI, m_kD;

    double m_targetAng;
    double m_curExpectedAng, m_curExpectedAngVel;
    double m_prevAngErr, m_totalAngErr;
    double m_angVecDir;
    double m_prevTime;
    double m_curAngVel;

    void CalcTimes(double dist);
    double CalcPID(double deltaT);
    double GetSpeed();
    bool AtAngTarget() const;
    bool AtAngTarget(double posErrTol, double velErrTol) const;
};