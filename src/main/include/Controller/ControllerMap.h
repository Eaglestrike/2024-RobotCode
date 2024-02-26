#pragma once

#include <vector>

#include "Constants/ControllerConstants.h"
#include "Util/simplevectors.hpp"

namespace vec = svector;

namespace Actions{
    enum Action{
        NONE = -1,
        SWERVE_STRAFEX,
        SWERVE_STRAFEY,
        SWERVE_ROTATION,
        ZERO_DRIVE_PID,
        ZERO_YAW,
        SLOW_MODE,
        INTAKE,
        INTAKE_TO_AMP,
        INTAKE_TO_CHANNEL,
        SHOOT,
        MANUAL_CLIMB,
        MANUAL_INTAKE_WRIST,
        BRAKE,
        UNBRAKE,
        MANUAL_1,
        MANUAL_2,
        MANUAL_EJECT_IN,
        MANUAL_EJECT_OUT,
        ZERO_1,
        ZERO_2,
        ZERO_CLIMB,
        ZERO_INTAKE,
        SHOOT_AUTO,
        ACTION_COUNT //Just the number of actions, as it is at the end of a enum
    };

    //Different enum for POV actions because logic is different
    enum POVAction{
        NO_POV_ACTION = -1,
        TEMP,
        CLIMB,
        AMP_AUTO_LINEUP,
        HALF_STOW,
        EXTEND,
        ACTION_COUNT_POV //Just the number of actions, as it is at the end of a enum
    };
}

namespace ControllerMapData{
    using namespace ControllerConstants;
    using namespace Actions;

    struct ControlMapElement{
        Button button;
        Action action;
    };

    //Maps Buttons -> Actions
    //Buttons are structs in the form of {Joystick, ButtonData}
    //There are already some named ButtonData and Buttons
    const std::vector<ControlMapElement> ButtonMap = {
        {{LJOY, X_AXIS},        SWERVE_STRAFEX},
        {{LJOY, Y_AXIS},        SWERVE_STRAFEY},
        {{LJOY, TRIGGER},       SHOOT},
        {{LJOY, B_4},           NONE},
        {{LJOY, B_2},           NONE},
        {{RJOY, B_3},           NONE},
        {{RJOY, X_AXIS},        SWERVE_ROTATION},
        {{RJOY, Y_AXIS},        NONE},
        {{RJOY, TRIGGER},       INTAKE},
        {{RJOY, B_2},           SLOW_MODE},

        {XBOX_LJOY_X,           NONE},
        {XBOX_LJOY_Y,           MANUAL_CLIMB}, 
        {XBOX_RJOY_X,           NONE},
        {XBOX_RJOY_Y,           MANUAL_INTAKE_WRIST},
        {XBOX_A_BUTTON ,        INTAKE_TO_AMP},
        {XBOX_B_BUTTON ,        MANUAL_EJECT_OUT}, // moved amp autolineup to pov right
        {XBOX_X_BUTTON ,        MANUAL_EJECT_IN}, // moved half stow to pov left
        {XBOX_Y_BUTTON ,        INTAKE_TO_CHANNEL},
        {XBOX_L_BUMPER ,        ZERO_1},
        {XBOX_LTRIGGER ,        MANUAL_1},
        {XBOX_R_BUMPER ,        ZERO_2},
        {{XBOX, B_7} ,          ZERO_DRIVE_PID},
        {{XBOX, B_8} ,          ZERO_YAW},
        {XBOX_RTRIGGER ,        MANUAL_2},
        {BB_Y_TRIM_DOWN ,       UNBRAKE},
        {BB_Y_TRIM_UP ,         BRAKE},
        {BB_LEFT ,              ZERO_CLIMB},
        {BB_RIGHT ,             ZERO_INTAKE},
        {BB_UP,                 SHOOT_AUTO}
    };

    //Allows for maps of buttons to values, such as the index of the buttonboard
    //Only for buttons and triggers currently
    //No need for default val because it's now in the controller method
    template <typename T>
    struct ValueMapElement{
        Button button;
        T value;
    };

     const std::vector<ValueMapElement<int>> SCORING_POS = {
        {{BUTTONBOARD, B_1}, 1},
        {{BUTTONBOARD, B_2}, 2},
        {{BUTTONBOARD, B_3}, 3},
        // {{BUTTONBOARD, B_4}, 4},
        // {{BUTTONBOARD, B_5}, 5},
        // {{BUTTONBOARD, B_6}, 6},
        // {{BUTTONBOARD, B_7}, 7},
        // {{BUTTONBOARD, B_8}, 8},
        // {{BUTTONBOARD, B_9}, 9},
    };

     const std::vector<ValueMapElement<double>> SHOOT_MANUAL = {
        {BB_L1, 0.31},
        {BB_L2, 0.65},
        {BB_L3, 1.00}
    };

    const double TRIM_SIZE = 0.075; // shooter units
    const std::vector<ValueMapElement<vec::Vector2D>> GET_SHOOTER_TRIM = {
        {BB_X_TRIM_LEFT,    {-TRIM_SIZE, 0.0}},
        {BB_X_TRIM_RIGHT,   {TRIM_SIZE, 0.0}},
        {BB_Y_TRIM_UP,      {0.0, TRIM_SIZE}},
        {BB_Y_TRIM_DOWN,    {0.0,-TRIM_SIZE}}
    };

    //Takes the range from min to max
    //if range covers over 0, like from 350->10, the larger number comes first
    struct POVRange{
        int min;
        int max;
    };

    const POVRange POV_UP = {350, 10};
    const POVRange POV_RIGHT = {80, 100};
    const POVRange POV_DOWN = {170, 190};
    const POVRange POV_LEFT = {260, 280};

    struct POVMapElement{
        Button pov;
        POVRange range;
        POVAction action;
    };

    const std::vector<POVMapElement> POVMap = {
        {XBOX_POV, POV_UP, EXTEND},
        {XBOX_POV, POV_DOWN, CLIMB},
        {XBOX_POV, POV_LEFT, HALF_STOW},
        {XBOX_POV, POV_RIGHT, AMP_AUTO_LINEUP},
    };
};