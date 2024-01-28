#pragma once

#include <string>
#include <vector>
#include <map>

#include "Util/Utils.h"

namespace AutoConstants {
  // Translational PID after FF
  const double DRIVE_P = 15;
  const double DRIVE_I = 0.001;
  const double DRIVE_D = 0.02;

  // Angular velocity PID after FF
  const double ANG_P = 15;
  const double ANG_I = 0.001;
  const double ANG_D = 0.08;

  // tolerance for being at target, in m and degrees
  const double POS_TOL = 0.05;
  const double ANG_TOL = Utils::DegToRad(3);

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

  const double SHOOT_TIME = 1.0;
  const double INTAKE_TIME = 1.0;
  const double STOW_TIME = 1.0;

  //Time padding
  const double DRIVE_PADDING = 0.5;
  const double INTAKE_PADDING = 0.5;
  const double STOW_PADDING = 0.5;
  const double SHOOT_PADDING = 0.0;
}