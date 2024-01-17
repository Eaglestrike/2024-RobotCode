#pragma once

#include <string>
#include <vector>

#include "Util/Utils.h"

namespace AutoConstants {
  const std::vector<std::string> DEPLOY_FILES = {
    "4pieceswooo",
    "Blue_LeftIntake_to_LeftScore",
    "Blue_LeftIntake_to_MidScore",
    "Blue_LeftScore_to_LeftIntake",
    "Blue_LeftScore_to_MidIntake",
    "Blue_MidIntake_to_LeftScore",
    "Blue_MidIntake_to_MidScore",
    "Blue_MidIntake_to_RightScore",
    "Blue_MidScore_to_LeftIntake",
    "Blue_MidScore_to_MidIntake",
    "Blue_MidScore_to_RightIntake",
    "Blue_RightIntake_to_MidScore",
    "Blue_RightIntake_to_RightScore",
    "Blue_RightScore_to_MidIntake",
    "Blue_RightScore_to_RightIntake"
  };

  // Translational PID after FF
  // TODO change
  const double DRIVE_P = 0;
  const double DRIVE_I = 0;
  const double DRIVE_D = 0;

  // Angular velocity PID after FF
  // TODO change
  const double ANG_P = 0;
  const double ANG_I = 0;
  const double ANG_D = 0;

  // tolerance for being at target, in m and degrees
  const double POS_TOL = 0.1;
  const double ANG_TOL = Utils::DegToRad(5);
}