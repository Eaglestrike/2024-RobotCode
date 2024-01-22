#pragma once

#include "Poses.h"

/**
 * PID controller class
*/
class PID{
    public:
        struct PIDConfig{
            double kp;
            double ki;
            double kd;
        };

        PID(PIDConfig config, double posTol = 0.0, double velTol = 0.0);
        double Calculate(Poses::Pose1D pose, Poses::Pose1D target);
        void Reset();

        bool AtTarget() const;

        void SetPID(PIDConfig config);
        PIDConfig GetPID();
        
        void SetTolerance(double posTol, double velTol);
        double GetPosTolerance();
        double GetVelTolerance();

    private:
        //Config
        PIDConfig config_;
        double posTol_;
        double velTol_;
        //Calculation members
        double accum_;
        Poses::Pose1D prevPose_;
        double prevT_;
        bool atTarget_;
};