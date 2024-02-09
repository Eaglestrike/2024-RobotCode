#pragma once

#include <string>

#include "Constants/AutoConstants.h"
#include "Util/hermite.hpp"
#include "Util/simplevectors.hpp"

namespace vec = svector;
namespace hm = hermite;

/**
 * Helps with getting position based on which side we are on
*/
namespace SideHelper {
  bool IsBlue();
  vec::Vector2D GetPos(vec::Vector2D bluePos);
  vec::Vector2D GetVel(vec::Vector2D blueVel);
  double GetAng(double blueAng);
  double GetAngVel(double blueAngVel);
  AutoConstants::StartPose GetStartingPose(int idx);
  AutoConstants::StartPose GetStartingPose(std::string pos);
  double GetJoystickAng();
  double GetManualLineupAng(int idx);

  std::string GetPath(std::string path);

  hm::Hermite<2> GetSplinePos(hm::Hermite<2> inp);
  hm::Hermite<1> GetSplineAng(hm::Hermite<1> inp);
}