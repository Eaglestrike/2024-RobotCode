#pragma once

#include "Util/simplevectors.hpp"
#include "Util/Utils.h"

namespace vec = svector;

namespace AutoLineupConstants {
  vec::Vector2D SPEAKER_POS_RED{};
  vec::Vector2D SPEAKER_POS_BLUE{};

  const double ANG_P = 15;
  const double ANG_I = 0.001;
  const double ANG_D = 0.08;

  const double ANG_TOL = Utils::DegToRad(3);
}