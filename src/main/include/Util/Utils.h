#pragma once

#include <cstddef>
#include <vector>

#include "simplevectors.hpp"

namespace vec = svector; //!< vector namespace alias

/**
 * Utility class with static methods
*/
namespace Utils {
  namespace {
    const double NEAR_ZERO_TOLERANCE = 0.000001;    
  }

  double AbsMin(const double a, const double b);

  vec::Vector2D GetVecAverage(const std::vector<vec::Vector2D>);

  bool NearZero(const double num, const double tolerance = NEAR_ZERO_TOLERANCE);
  bool NearZero(const vec::Vector2D vec, const double tolerance = NEAR_ZERO_TOLERANCE);

  double NormalizeAng(const double ang);
  double NormalizeAngDeg(const double ang);

  std::size_t GetCurTimeMs();
  double GetCurTimeS();

  double DegToRad(const double deg);
  double RadToDeg(const double rad);

  vec::Vector2D GetUnitVecDir(const double ang);
  vec::Vector2D GetProjection(const vec::Vector2D v, const vec::Vector2D w);
  double GetAngBetweenVec(const vec::Vector2D v1, const vec::Vector2D v2);
  vec::Vector2D MultiplyComps(const vec::Vector2D v1, const vec::Vector2D v2);
};