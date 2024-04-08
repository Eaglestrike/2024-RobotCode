#pragma once

#include <string>

#include "Util/Odometry.h"
#include "Util/Utils.h"
#include "Util/simplevectors.hpp"
#include "Util/TrapezoidalProfile.h"
#include "ShuffleboardSender/ShuffleboardSender.h"

/**
 * Auto angle lineup motion profile, for drivebase only
 * 
 * Shooter auto lineup should be handled outside of this class
*/
class AutoAngLineup{
    public:
        AutoAngLineup(bool shuffleboard, Odometry &odom);

        double GetAngVel();
        bool AtTarget();

        void SetTarget(double targAng);

        void SetProfileConfig(double maxSpeed, double maxAccel);
        void SetPID(double kP, double kI, double kD);

        void ShuffleboardInit();
        void ShuffleboardPeriodic();

        double GetTargAng();
        double GetExpAng();

    private:
        Odometry &m_odom;
        TrapezoidalProfile m_profile;

        double m_prevTime;
        double m_kP, m_kI, m_kD;
        double m_totalAngErr;

        double m_posTol, m_velTol;
        
        ShuffleboardSender m_shuff;
};