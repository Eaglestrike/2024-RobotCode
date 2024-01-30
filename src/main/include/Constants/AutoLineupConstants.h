#pragma once

#include <cmath>
#include <vector>

#include "Util/Utils.h"

namespace vec = svector;

namespace AutoLineupConstants {
  const double ANG_P = 11;
  const double ANG_I = 0.001;
  const double ANG_D = 0.02;

  // const double ANG_VEL_MULT = 4;

  const double ANG_TOL = Utils::DegToRad(3);

  const double MAX_SPEED = 7;
  const double MAX_ACCEL = 20;

  const double AMP_LINEUP_ANG = -M_PI / 2;

  const std::vector<vec::Vector2D> BLUE_SHOOT_LOCATIONS = 
  {
    {0.722, 4.706}, // R under speaker
    {1.313, 5.733}, // M under speaker
    {0.636, 6.921}, // L under speaker
    {2.798, 4.295}  // safe zone
  };
}