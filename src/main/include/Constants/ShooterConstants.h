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
        .name = "Left Flywheel",
        .id = 0,
        .inverted = true
    };

    const FlywheelConfig RIGHT_FLYWHEEL{
        .name = "Right Flywheel",
        .id = 0,
        .inverted = false
    };

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
    const int PIVOT_CHILD_ID = 0;
    
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

    const std::map<double, ShootConfig> SHOOT_DATA = { //Distance -> ang, vel
        //{0.0, {0.0, 0.0}}
    };

    const double K_SPIN = 0.0; //Constant of how much the robot spins the note

    const double STROLL_SPEED = 0.0; //Voltage of strolling

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