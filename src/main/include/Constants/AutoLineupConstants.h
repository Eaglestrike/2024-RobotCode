#pragma once

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
}