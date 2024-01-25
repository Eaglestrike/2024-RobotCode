#pragma once

#include <string>
#include <vector>
#include <map>

#include "Util/Utils.h"

namespace AutoConstants {
  // Translational PID after FF
  // TODO change
  const double DRIVE_P = 15;
  const double DRIVE_I = 0.001;
  const double DRIVE_D = 0.02;

  // Angular velocity PID after FF
  // TODO change
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
  const AutoPath TEST = {
    {DRIVE, AFTER, "testIntake"},
    {INTAKE, BEFORE_END},
    {DRIVE, AFTER, "backFromIntake"},
    {STOW, AT_START, "", 1.0},
    {SHOOT, AFTER, "", 1.0}
  };

  const AutoPath TEST2 = {
    {DRIVE, AFTER, "testIntake"},
    {INTAKE, BEFORE_END, "", 1.0},
    {DRIVE, AFTER, "backFromIntake", 4.0},
    {STOW, AT_START, "", 1.0},
    {SHOOT, AFTER, "", 1.0}
  };

  const AutoPath NOTHING = {
  };

  //Shuffleboard paths
  const std::map<std::string, AutoPath> PATHS = {
      {"drive and intake", TEST},
      {"drive and intake wait", TEST2},
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