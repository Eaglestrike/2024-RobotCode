#pragma once

#include <map>

#include "Util/simplevectors.hpp"

namespace vec = svector;

/**
 * Mechanical Advantage 6328 pose estimator, based on WPILib's pose estimtaor
 * 
 * https://github.com/Mechanical-Advantage/RobotCode2023/blob/main/src/main/java/org/littletonrobotics/frc2023/util/PoseEstimator.java
*/
class PoseEstimator {
public:

private:
  vec::Vector2D m_q;
  vec::Vector2D m_basePose;
  vec::Vector2D m_curPose;
};