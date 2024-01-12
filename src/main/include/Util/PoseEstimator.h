#pragma once

#include <map>
#include <vector>

#include "Util/simplevectors.hpp"

namespace vec = svector;

/**
 * Mechanical Advantage 6328 pose estimator, based on WPILib's pose estimtaor
 * 
 * https://github.com/Mechanical-Advantage/RobotCode2023/blob/main/src/main/java/org/littletonrobotics/frc2023/util/PoseEstimator.java
*/
class PoseEstimator {
public:
  struct VisionUpdate {
    vec::Vector2D pos;
    vec::Vector2D stdDev;
  };

  struct PoseUpdate {
    vec::Vector2D deltaPos;
    std::vector<VisionUpdate> visUpdates;

    vec::Vector2D Apply(const vec::Vector2D &lastPose, const vec::Vector2D &q);
  };

  PoseEstimator(const vec::Vector2D &stdDevs);

  void UpdateDrivebase(const double &timestamp, const vec::Vector2D &deltaPos);
  void UpdateCams(const double &timestamp, const vec::Vector2D &camPos, const vec::Vector2D &stdDevs);

private:
  vec::Vector2D m_q;
  vec::Vector2D m_basePose;
  vec::Vector2D m_curPose;
  std::map<double, PoseUpdate> m_updates;

  const double MAX_HISTORY_TIME = 0.3;

  void Update();
};