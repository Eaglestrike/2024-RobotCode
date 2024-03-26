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

    struct Incher{
        double volts;
        int onCycles;
        int numCycles;
    };

    const std::string SHOOTER_CANBUS = "";

    //Flywheel Constants
    struct FlywheelConfig{
        std::string name;
        int id;
        bool inverted;
    };

    const FlywheelConfig BOTTOM_FLYWHEEL{
        .name = "Bottom Flywheel",
        .id = 30,
        .inverted = true
    };

    const FlywheelConfig TOP_FLYWHEEL{
        .name = "Top Flywheel",
        .id = 25,
        .inverted = true
    };
    
    const double FLYWHEEL_GEARING = 24.0/36.0;

    const double FLYWHEEL_R = 0.0508;
    const double FLYWHEEL_MAX_A = 40.0; //Max Acceleration
    const double FLYWHEEL_MAX_VOLTS = 10.0; //Max velocity -> ~20.0

    const PID FLYWHEEL_PID = {
        .kp = 0.0,
        .ki = 0.07,
        .kd = 0.0
    };

    const Feedforward FLYWHEEL_FF = {
        .ks = 0.197,
        .kv = 0.506,
        .ka = 0.01142
    };

    const double FLYWHEEL_VEL_TOL = 0.5;

    const uint FLYWHEEL_FILTER_SIZE = 10;

    //Pivot Constants
    const int PIVOT_ID = 37;
    const int PIVOT_CHILD_ID = 35;

    const int PIVOT_ENCODER_ID = 15;

    const double PIVOT_GEARING = 12.0/196.0;
    
    const double PIVOT_MIN = 17.7 * M_PI/180.0;
    const double PIVOT_MAX = 69.9 * M_PI/180.0;

    const double PIVOT_UNHOOK = PIVOT_MAX - 0.01;
   
    const double PIVOT_MAX_VOLTS = 4.0;

    const double PIVOT_OFFSET = 5.587;
 
    const PID PIVOT_PID = {
        .kp = 10.0, //11.0
        .ki = 0.4,
        .kd = 0.2
    };

    const Feedforward PIVOT_FF = {
        .ks = 0.01,
        .kv = 0.366,
        .ka = 0.0152,
        .kg = 0.475
    };

    const Incher PIVOT_INCH = {
        .volts = 0.7,
        .onCycles = 2,
        .numCycles = 20
    };
    const double PIVOT_INCH_TOL = 0.01;
    const double PIVOT_INCH_DEADBAND = 0.007;

    const double PIVOT_MAX_V = 5.5;
    const double PIVOT_MAX_A = 5.0;

    const double PIVOT_POS_TOL = 0.02;
    const double PIVOT_VEL_TOL = 0.2;
    const double PIVOT_REGEN_TOL = 0.1;

    //Shooter data
    struct ShootConfig{
        double ang;
        double vel;
    };

    const std::map<double, ShootConfig> SHOOT_DATA = {
    //distance-> ang, vel
        {0.0,   {1.075,  15.0}}, //0 distance shot (used just for interpolation)
        {1.32,  {1.075,  15.0}},
        {1.51,  {0.97,  17.0}},
        {1.68,  {0.93,  17.0}},
        {1.88,  {0.87,  17.0}},
        {2.10,  {0.81,  17.0}},
        {2.37,  {0.74,  17.0}},
        {2.64,  {0.69,  17.0}},
        {3.08,  {0.625, 18.0}},
        {3.46,  {0.585, 18.0}},
        {3.89,  {0.54,  18.0}},
        {4.37,  {0.51,  18.0}}
    };

    //const double K_SPIN = 0.0; //Constant of how much the robot spins the note

    const double STROLL_SPEED = 0.7; //Voltage of strolling

    const double SHOOT_TIME = 0.5; //Time for piece to exit shooter
 
    //Tolerances
    const double SHOOT_POS_TOL = 0.3;
    const double SHOOT_VEL_TOL = 0.3;
    //const double SHOOT_YAW_TOL = 0.05;
    const double SHOOT_YAW_PERCENT = 0.5;
    const double LINEUP_YAW_PERCENT = 0.37; //0.45
    const double PIVOT_ANG_PERCENT = 0.7; // 0.8

    //Field Data

    //Speaker center positions (x, y)
    const vec::Vector2D RED_SPEAKER = {16.54-0.30-0.075, 5.58};
    const vec::Vector2D BLUE_SPEAKER = {0.0+0.15+0.225, 5.58};

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

    //Constants for time calculation (t = kD * d + cT)
    const double kD = 0.269123;
    const double cT = 0.087579;
    const double prepareT = 0.1;

    // amp
    const double PIVOT_AMP = 1.15; //1.1
    const double FLYWHEEL_SPEED_AMP = 3.3; //4.25
    const double FLYWHEEL_SPIN_AMP = 0.0; //-0.25
}