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
        .id = 25,
        .inverted = true
    };
    
    const double FLYWHEEL_GEARING = 24.0/36.0;

    const double FLYWHEEL_R = 0.0508;
    const double FLYWHEEL_MAX_A = 40.0; //Max Acceleration
    const double FLYWHEEL_MAX_VOLTS = 10.0; //Max velocity -> ~20.0

    const PID FLYWHEEL_PID = {
        .kp = 0.1,
        .ki = 0.1,
        .kd = 0.0
    };

    const Feedforward FLYWHEEL_FF = {
        .ks = 0.297,
        .kv = 0.506,
        .ka = 0.01142
    };

    const double FLYWHEEL_VEL_TOL = 0.5;

    //Pivot Constants
    const int PIVOT_ID = 37;
    const int PIVOT_CHILD_ID = 35;

    const int PIVOT_ENCODER_ID = 15;

    const double PIVOT_GEARING = 12.0/196.0;
    
    const double PIVOT_MIN = 17.7 * M_PI/180.0;
    const double PIVOT_MAX = 67.2 * M_PI/180.0;
   
    const double PIVOT_MAX_VOLTS = 3.0;

    const double PIVOT_OFFSET = 5.587;
 
    const PID PIVOT_PID = {
        .kp = 6.0,
        .ki = 0.8,
        .kd = 0.2
    };

    const Feedforward PIVOT_FF = {
        .ks = 0.08,
        .kv = 0.356,
        .ka = 0.0352,
        .kg = 0.36
    };

    const double PIVOT_MAX_V = 2.0;
    const double PIVOT_MAX_A = 3.0;

    const double PIVOT_POS_TOL = 0.02;
    const double PIVOT_VEL_TOL = 0.05;

    //Shooter data
    struct ShootConfig{
        double ang;
        double vel;
    };

    const std::map<double, ShootConfig> SHOOT_DATA = {
    //distance-> ang, vel
        {0.0,   {1.1,  17.0}}, //0 distance shot (used just for interpolation)
        {1.32,  {1.05,  17.0}},
        {1.55,  {1.02,   17.0}},
        {1.7,   {0.90,  17.0}},
        {1.857, {0.84,  17.0}},
        {1.97,  {0.82,  17.0}},
        {2.06,  {0.8,   17.0}},
        {2.65,  {0.7,   17.0}},
        {2.86,  {0.64,  18.0}},
        {3.32,  {0.62,  18.5}},
        {3.39,  {0.6,   19.0}},
        {4.14,  {0.54,  19.0}},
        {4.40,  {0.52,  19.0}},
        {4.47,  {0.51,  19.0}},
        {5.461, {0.43,  19.0}}
    };

    const double K_SPIN = 0.0; //Constant of how much the robot spins the note

    const double STROLL_SPEED = 0.3; //Voltage of strolling

    const double PIVOT_INTAKE = PIVOT_MIN + 0.4; //Angle for pivot to intake piece into shooter
    const double SHOOT_TIME = 1.0; //Time for piece to exit shooter
 
    //Tolerances
    const double SHOOT_POS_TOL = 0.3;
    const double SHOOT_VEL_TOL = 0.3;
    //const double SHOOT_YAW_TOL = 0.05;
    const double SHOOT_YAW_PERCENT = 0.6;
    const double LINEUP_YAW_PERCENT = 0.5;
    const double PIVOT_ANG_PERCENT = 0.8;

    //Field Data

    //Speaker center positions (x, y)
    const vec::Vector2D RED_SPEAKER = {17.0, 5.58};
    const vec::Vector2D BLUE_SPEAKER = {0.0, 5.58};

    //Dimensions of the shootable area
    const double SPEAKER_MIN = 1.98; //height; m
    const double SPEAKER_MAX = 2.11;
    const double SPEAKER_CENTER = (SPEAKER_MIN + SPEAKER_MAX)/2.0;
    const double SPEAKER_HEIGHT = SPEAKER_MAX - SPEAKER_MIN;
    const double SPEAKER_WIDTH = 1.05; // y
    const double SPEAKER_DEPTH = 0.424; // x

    //Birds Eye Box
    const std::vector<vec::Vector2D> RED_SPEAKER_BOX = {
        RED_SPEAKER + vec::Vector2D{-SPEAKER_DEPTH, SPEAKER_WIDTH/2.0},
        RED_SPEAKER + vec::Vector2D{-SPEAKER_DEPTH, -SPEAKER_WIDTH/2.0},
        RED_SPEAKER + vec::Vector2D{0.0, SPEAKER_WIDTH/2.0},
        RED_SPEAKER + vec::Vector2D{0.0, -SPEAKER_WIDTH/2.0},
    };

    const std::vector<vec::Vector2D> BLUE_SPEAKER_BOX = {
        BLUE_SPEAKER + vec::Vector2D{SPEAKER_DEPTH, SPEAKER_WIDTH/2.0},
        BLUE_SPEAKER + vec::Vector2D{SPEAKER_DEPTH, -SPEAKER_WIDTH/2.0},
        BLUE_SPEAKER + vec::Vector2D{0.0, SPEAKER_WIDTH/2.0},
        BLUE_SPEAKER + vec::Vector2D{0.0, -SPEAKER_WIDTH/2.0},
    };

    const double SHOOTER_HEIGHT = 0.0;

    const vec::Vector2D ABSOLUTE_MISS = {10000000.0, 10000000.0}; //Forward kinematic miss


    //Shooter amp : ang:1.0, vel:3.9
    //Constants for time calculation (t = kD * d + cT)
    const double kD = 0.0424441;
    const double cT = 0.0834987;
    const double prepareT = 0.1;
}