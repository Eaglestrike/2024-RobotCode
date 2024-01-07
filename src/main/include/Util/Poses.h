#pragma once

namespace Poses{
    struct Pose1D{
        double pos;
        double vel;
        double acc;
    };

    Pose1D extrapolate(const Pose1D& pose, double time);

    Pose1D abs(const Pose1D& pose);

    Pose1D operator+(const Pose1D& pose1, const Pose1D& pose2);
    Pose1D& operator+=(Pose1D& pose1, const Pose1D& pose2);
    Pose1D operator-(const Pose1D& pose1, const Pose1D& pose2);
    Pose1D operator*(const Pose1D& pose, double k);
}