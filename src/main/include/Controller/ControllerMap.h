#pragma once

#include <vector>

#include "Constants/ControllerConstants.h"

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
        ACTION_COUNT //Just the number of actions, as it is at the end of a enum
    };

    //Different enum for POV actions because logic is different
    enum POVAction{
        NO_POV_ACTION = -1,
        TEMP,
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
        {XBOX_LJOY_Y,           NONE}, 
        {XBOX_RJOY_X,           NONE},
        {XBOX_RJOY_Y,           NONE},
        {XBOX_A_BUTTON ,        INTAKE_TO_AMP},
        {XBOX_B_BUTTON ,        NONE},
        {XBOX_X_BUTTON ,        NONE},
        {XBOX_Y_BUTTON ,        INTAKE_TO_CHANNEL},
        {XBOX_L_BUMPER ,        NONE},
        {XBOX_LTRIGGER ,        NONE},
        {XBOX_R_BUMPER ,        NONE},
        {{XBOX, B_7} ,          ZERO_DRIVE_PID},
        {{XBOX, B_8} ,          ZERO_YAW},
        {XBOX_RTRIGGER ,        NONE}
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
        {{BUTTONBOARD, B_4}, 4},
        {{BUTTONBOARD, B_5}, 5},
        {{BUTTONBOARD, B_6}, 6},
        {{BUTTONBOARD, B_7}, 7},
        {{BUTTONBOARD, B_8}, 8},
        {{BUTTONBOARD, B_9}, 9},
    };

     const std::vector<ValueMapElement<int>> GET_LEVEL = {
        {BB_L1, 1},
        {BB_L2, 2},
        {BB_L3, 3}
    };

    const double TRIM_SIZE = 0.05; // in m
    const std::vector<ValueMapElement<double>> GET_TRIM_X = {
        {BB_X_TRIM_UP, TRIM_SIZE},
        {BB_X_TRIM_DOWN, -TRIM_SIZE}
    };

    const std::vector<ValueMapElement<double>> GET_TRIM_Y = {
        {BB_Y_TRIM_UP, TRIM_SIZE},
        {BB_Y_TRIM_DOWN, -TRIM_SIZE}
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
    };
};