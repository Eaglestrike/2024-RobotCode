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
    "odometryTest",
    "odometryTestCurve"
  };

  // Translational PID after FF
  // TODO change
  const double DRIVE_P = 15;
  const double DRIVE_I = 0;
  const double DRIVE_D = 0;

  // Angular velocity PID after FF
  // TODO change
  const double ANG_P = 3;
  const double ANG_I = 0.6;
  const double ANG_D = 0;

  // tolerance for being at target, in m and degrees
  const double POS_TOL = 0.05;
  const double ANG_TOL = Utils::DegToRad(3);
}