#include <cmath>
#include "Util/Utils.h"

class TrapezoidalMotionProfile {
    public:
    TrapezoidalMotionProfile(double MAX_VEL, double MAX_ACC, double curPos, double setPt);
    TrapezoidalMotionProfile(double MAX_VEL, double MAX_ACC);
    double GetVelocity() const;
    double GetPosition() const;
    double GetAcceleration() const;
    bool AtSetPoint() const;
    double GetMaxVel() const;
    double GetMaxAcc() const;

    void SetSetpoint(double curPos, double setPoint);
    void SetMaxVel(double maxVel);
    void SetMaxAcc(double maxAcc);
    void Periodic();
    private:
        // void CalcTurnTime(double curPos, double setPt);
        void CalcSpeedDecreasePos(double curPos, double setPt);
        double m_targetPos, m_targetVel, m_targetAcc;
        double m_maxVel, m_maxAcc;
        double m_setPt;
        double m_speedDecreasePos;
        double m_curTime;
};