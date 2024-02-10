#pragma once

#include <map>
#include "Util/simplevectors.hpp"

namespace vec = svector;

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
    struct FlywheelConfig{
        std::string name;
        int id;
        bool inverted;
    };

    const FlywheelConfig LEFT_FLYWHEEL{
        .name = "Bottom Flywheel",
        .id = 30,
        .inverted = true
    };

    const FlywheelConfig RIGHT_FLYWHEEL{
        .name = "Top Flywheel",
        .id = 3,
        .inverted = true
    };
    
    const double FLYWHEEL_GEARING = 24.0/36.0;

    const double FLYWHEEL_R = 0.0508;
    const double FLYWHEEL_MAX_A = 0.0; //Max Acceleration
    const double FLYWHEEL_MAX_VOLTS = 0.0;

    const PID FLYWHEEL_PID = {
        .kp = 0.0,
        .ki = 0.0,
        .kd = 0.0
    };

    const Feedforward FLYWHEEL_FF = {
        .ks = 0.297,
        .kv = 0.506,
        .ka = 0.01142
    };

    const double FLYWHEEL_VEL_TOL = 0.0;

    //Pivot Constants
    const int PIVOT_ID = 37;
    const int PIVOT_CHILD_ID = 35;

    const int PIVOT_ENCODER_ID = 15;

    const double PIVOT_GEARING = 12.0/196.0;
    
    const double PIVOT_MIN = 0.312;
    const double PIVOT_MAX = PIVOT_MIN + 0.816988623;

    const double PIVOT_MAX_VOLTS = 10.0;

    const double PIVOT_OFFSET = 0.34535;

    const PID PIVOT_PID = {
        .kp = 0.0,
        .ki = 0.0,
        .kd = 0.0
    };

    const Feedforward PIVOT_FF = {
        .ks = 1.0,
        .kv = 0.663,
        .ka = 0.1579,
        .kg = 0.297
    };

    const double PIVOT_MAX_V = 0.0;
    const double PIVOT_MAX_A = 0.0;

    const double PIVOT_POS_TOL = 0.0;
    const double PIVOT_VEL_TOL = 0.0;

    //Shooter data
    struct ShootConfig{
        double ang;
        double vel;
    };

    const std::map<double, ShootConfig> SHOOT_DATA = { //Distance -> ang, vel
        //{0.0, {0.0, 0.0}}
    };

    const double K_SPIN = 0.0; //Constant of how much the robot spins the note

    const double STROLL_SPEED = 0.0; //Voltage of strolling

    const double PIVOT_INTAKE = 0.0; //Angle for pivot to intake piece into shooter
    const double SHOOT_TIME = 1.0; //Time for piece to exit shooter

    //Tolerances
    const double SHOOT_POS_TOL = 0.3;
    const double SHOOT_VEL_TOL = 0.3;
    const double SHOOT_YAW_TOL = 0.05;

    //Kinematics calc (Field Data + robot stats)
    const vec::Vector2D RED_SPEAKER = {17.0, 5.74};
    const vec::Vector2D BLUE_SPEAKER = {0.0, 5.74};

    const double SPEAKER_MIN = 1.98; //height; m
    const double SPEAKER_MAX = 2.11;
    const double SPEAKER_CENTER = (SPEAKER_MIN + SPEAKER_MAX)/2.0;
    const double SPEAKER_HEIGHT = SPEAKER_MAX - SPEAKER_MIN;
    const double SPEAKER_WIDTH = 1.05;

    const double SHOOTER_HEIGHT = 0.0;

    const vec::Vector2D ABSOLUTE_MISS = {10000000.0, 10000000.0}; //Forward kinematic miss
}