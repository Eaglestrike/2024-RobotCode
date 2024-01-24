#pragma once

#include <string>
#include <vector>

#include "Util/Utils.h"

namespace AutoConstants {
  const std::vector<std::string> DEPLOY_FILES = {
    "hehe",
    "4pieceswooo",
    "aprilTagOdomTest",
    "aprilTagOdomTest7s",
    "aprilTagOdomTest4s",
    "aprilTagOdomTest3s",
    "aprilTagOdomTest2s",
    "odometryTest",
    "odometryTestCurve"
  };

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
    {DRIVE, AFTER, "zero"},
    {STOW, AT_START, "", 1.0},
    {SHOOT, AFTER, "", 1.0}
  };

  const double SHOOT_TIME = 1.0;
  const double INTAKE_TIME = 1.0;
  const double STOW_TIME = 1.0;
}