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
  const AutoPath INTAKE1 = {
    {DRIVE, AFTER, "testIntake1.csv"},
    {INTAKE, BEFORE_END},
    {DRIVE, AFTER, "testBackFromIntake.csv"},
    {SHOOT, AFTER, "", 1.0}
  };
  const AutoPath INTAKE2 = {
    {DRIVE, AFTER, "testIntake2.csv"},
    {INTAKE, BEFORE_END},
    {DRIVE, AFTER, "testBackFromIntake.csv"},
    {SHOOT, AFTER, "", 1.0}
  };
  const AutoPath INTAKE3 = {
    {DRIVE, AFTER, "testIntake3.csv"},
    {INTAKE, BEFORE_END},
    {DRIVE, AFTER, "testBackFromIntake.csv"},
    {SHOOT, AFTER, "", 1.0}
  };

  const AutoPath NOTHING = {
  };

  //Shuffleboard paths
  const std::map<std::string, AutoPath> PATHS = {
      {"intake 1", INTAKE1},
      {"intake 2", INTAKE2},
      {"intake 3", INTAKE3},
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

  // starting positions
  const StartPose BLUE_L = {{0.666, 6.721}, 4.126271};
  const StartPose BLUE_M = {{1.265, 5.526}, 3.141592};
  const StartPose BLUE_R = {{0.597, 4.41}, 2.062718};
}