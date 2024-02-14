#pragma once

#include <string>
#include <vector>
#include <map>

#include "Util/Utils.h"
#include "Util/simplevectors.hpp"

namespace vec = svector;


namespace AutoConstants {
  struct StartPose {
    vec::Vector2D pos;
    double ang;
  };

  // Translational PID after FF
  const double DRIVE_P = 15;
  const double DRIVE_I = 0.001;
  const double DRIVE_D = 0.02;

  // Angular velocity PID after FF
  const double ANG_P = 15;
  const double ANG_I = 0.001;
  const double ANG_D = 0.08;

  // tolerance for being at target, in m and degrees
  const double POS_TOL = 0.005;
  const double POS_TOL_BIG = 0.1;
  const double ANG_TOL = Utils::DegToRad(2);
  const double ANG_TOL_BIG = Utils::DegToRad(5);

  // const double JITTER_FILTER = 0.005;

  enum AutoType{
    AT_START,
    BEFORE_END,
    AFTER
  };

  enum AutoAction{
    DRIVE,
    SHOOT,
    INTAKE,
    STOW
  };

  /**
   * Paths will be arrays of auto elements
  */
  struct AutoElement{
    AutoAction action;
    AutoType type;
    std::string data = ""; //String to select files or other data
    double offset = 0.0;
  };

  using AutoPath = std::vector<AutoElement>;
  const AutoPath FOUR_PIECE = {
    {SHOOT, AFTER},
    {DRIVE, AFTER, "RightScore_to_RightIntake.csv"},
    {INTAKE, BEFORE_END},
    {DRIVE, AFTER, "RightIntake_to_MidScore.csv"},
    {SHOOT, AFTER},
    {DRIVE, AFTER, "MidScore_to_MidIntake.csv"},
    {INTAKE, BEFORE_END},
    {DRIVE, AFTER, "MidIntake_to_MidScore.csv"},
    {SHOOT, AFTER},
    {DRIVE, AFTER, "MidScore_to_LeftIntake.csv"},
    {INTAKE, BEFORE_END},
    {DRIVE, AFTER, "LeftIntake_to_MidScore.csv"},
    {SHOOT, AFTER}
  };

  const AutoPath NOTHING = {
  };

  //Shuffleboard paths
  const std::map<std::string, AutoPath> PATHS = {
      {"Four Piece", FOUR_PIECE},
      {"Nothing", NOTHING}
  };

  const double CHANNEL_TIME = 0.5;
  const double SHOOT_TIME = 1.5;
  const double INTAKE_TIME = 1.0;
  const double STOW_TIME = 1.0;

  //Time padding
  const double DRIVE_PADDING = 0.5;
  const double INTAKE_PADDING = 0.5;
  const double STOW_PADDING = 0.5;
  const double SHOOT_PADDING = 0.5; //Shoot earlier bc channel

  //Pos tol for shoot aiming
  const double SHOOT_POS_TOL = 0.1;

  // starting positions
  const StartPose BLUE_L = {{0.666, 6.721}, 4.2309};
  const StartPose BLUE_M = {{1.265, 5.526}, 3.141592};
  const StartPose BLUE_R = {{0.597, 4.41}, 2.0375};

  // auto positions array size
  const int POS_ARR_SIZE = 8;

  // chooser names
  const std::string L_NAME = "Left";
  const std::string M_NAME = "Mid";
  const std::string R_NAME = "Right";
  const std::string S_NAME = "Skip";

  const std::string L_START = "LeftStart";
  const std::string M_START = "MidStart";
  const std::string R_START = "RightStart";

  const std::string L_NEAR = "LeftNear";
  const std::string M_NEAR = "MidNear";
  const std::string R_NEAR = "RightNear";
  const std::string L_FAR = "LeftFar";
  const std::string ML_FAR = "MidLeftFar";
  const std::string M_FAR = "MidFar";
  const std::string MR_FAR = "MidRightFar";
  const std::string R_FAR = "RightFar";

  const std::string L_SCORE = "LeftScore";
  const std::string M_SCORE = "MidScore";
  const std::string R_SCORE = "RightScore";
}