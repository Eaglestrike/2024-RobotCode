#include "Util/Poses.h"

#include <cmath>

namespace Poses{
    Pose1D extrapolate(const Pose1D& pose, double time){
        Pose1D newPose = pose;
        newPose.vel += time * pose.acc; // v = at
        newPose.pos += (newPose.vel + pose.vel)/2.0 * time; // x = (avg v) * t
        return newPose;
    }

    Pose1D abs(const Pose1D& pose){
        return {
            .pos = std::abs(pose.pos),
            .vel = std::abs(pose.vel),
            .acc = std::abs(pose.acc)
        };
    }

    Pose1D operator+(const Pose1D& pose1, const Pose1D& pose2){
        return {
            .pos = pose1.pos + pose2.pos,
            .vel = pose1.vel + pose2.vel,
            .acc = pose1.acc + pose2.acc
        };
    }

    Pose1D& operator+=(Pose1D& pose1, const Pose1D& pose2){
        pose1.pos += pose2.pos;
        pose1.vel += pose2.vel;
        pose1.acc += pose2.acc;
        return pose1;
    }

    Pose1D operator-(const Pose1D& pose1, const Pose1D& pose2){
        return {
            .pos = pose1.pos - pose2.pos,
            .vel = pose1.vel - pose2.vel,
            .acc = pose1.acc - pose2.acc
        };
    }

    Pose1D operator*(const Pose1D& pose, double k){
        return {
            .pos = pose.pos*k,
            .vel = pose.vel*k,
            .acc = pose.acc*k
        };
    }
}