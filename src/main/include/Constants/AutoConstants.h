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
}