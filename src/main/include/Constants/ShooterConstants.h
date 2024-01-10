#pragma once

#include <map>

namespace ShooterConstants{
    struct PID {
        double kp;
        double ki;
        double kd;
    };

    struct Feedforward{
        double ks;
        double kv;
        double ka;
        double kg = 0.0;
    };

    const std::string SHOOTER_CANBUS = "";

    //Flywheel Constants
    const int FLYWHEEL_ID = 0;

    const double FLYWHEEL_MAX_A = 0.0; //Max Acceleration
    const double FLYWHEEL_MAX_VOLTS = 0.0;

    const PID FLYWHEEL_PID = {
        .kp = 0.0,
        .ki = 0.0,
        .kd = 0.0
    };

    const Feedforward FLYWHEEL_FF = {
        .ks = 0.0,
        .kv = 0.0,
        .ka = 0.0
    };

    //Pivot Constants
    const int PIVOT_ID = 0;
    
    const double PIVOT_MIN = 0.0;
    const double PIVOT_MAX = 0.0;

    const double PIVOT_MAX_VOLTS = 0.0;

    const double PIVOT_OFFSET = 0.0;

    const PID PIVOT_PID = {
        .kp = 0.0,
        .ki = 0.0,
        .kd = 0.0
    };

    const Feedforward PIVOT_FF = {
        .ks = 0.0,
        .kv = 0.0,
        .ka = 0.0,
        .kg = 0.0
    };

    const double PIVOT_MAX_V = 0.0;
    const double PIVOT_MAX_A = 0.0;

    //Shooter data
    struct ShootConfig{
        double ang;
        double vel;
    };

    std::map<double, ShootConfig> SHOOT_DATA = { //Distance -> ang, vel
        {0.0, {0.0, 0.0}}
    };

    std::map<double, double> SHOOT_SPIN = { // Angle -> Spin
        {0.0, 0.0}
    };

    const double STROLL_SPEED = 0.0;
}