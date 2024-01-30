#pragma once

#include "Util/hermite.hpp"
#include "Util/Odometry.h"
#include "Util/Poses.h"

using Poses::Pose1D;

class AutoLineup{
    public:
        AutoLineup(Odometry &odom);
        vec::Vector2D GetVel() const;
        double GetAngVel() const;
        double GetAng() const;
        void Periodic();
        void SetTarget(vec::Vector2D newTarget);

    private:
         struct profileInfo{
            Pose1D targPose;
            double MAX_VEL;
            double MAX_ACC;
            double speedDecreasePos;
            double setPt;
        };

        void createProfile(profileInfo p, double vi, double targPos, double curPos = 0);
        void UpdateProfile(profileInfo p);

        profileInfo m_ang, m_pos;
        Odometry &m_odom;

        


};