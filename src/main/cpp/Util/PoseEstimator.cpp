#include "Util/PoseEstimator.h"

#include <algorithm>
#include <cmath>

#include "Util/Utils.h"

/**
 * Max time before rejecting cameras
*/
const double PoseEstimator::MAX_HISTORY_TIME = 0.3;

/**
 * Constructor
 * 
 * @param stdDevs system standard deviation in x and y directions
*/
PoseEstimator::PoseEstimator(const vec::Vector2D &stdDevs)
  : m_q{Utils::MultiplyComps(stdDevs, stdDevs)} {}

/**
 * Update drivebase
 * 
 * @param timestamp Timestamp
 * @param deltaPos delta position between measurements (speed * deltaT)
*/
void PoseEstimator::UpdateDrivebase(const double &timestamp, const vec::Vector2D &deltaPos) {
  PoseUpdate update{deltaPos, {}};
  m_updates[timestamp] = update; 
  Update();
}

/**
 * Compare method
 * 
 * @param v1 vision update 1
 * @param v2 vision update 2
 * 
 * @returns If v1 < v2 (if sums of std devs of v1 < sums of std devs of v2)
*/
bool cmp(const PoseEstimator::VisionUpdate &v1, const PoseEstimator::VisionUpdate &v2) {
  return v1.stdDev.x() + v1.stdDev.y() < v2.stdDev.x() + v2.stdDev.y();
}

/**
 * Updates cameras
 * 
 * @param timestamp Timestamp
 * @param camPos Camera pos
 * @param stdDevs Standard deviations for each direction
*/
void PoseEstimator::UpdateCams(const double &timestamp, const vec::Vector2D &camPos, const vec::Vector2D &stdDevs) {
  VisionUpdate curVis{camPos, stdDevs};

  if (m_updates.find(timestamp) != m_updates.end()) {
    // if already have timestamp, then add to it
    m_updates[timestamp].visUpdates.push_back(curVis);
    std::sort(m_updates[timestamp].visUpdates.begin(), m_updates[timestamp].visUpdates.end(), cmp);
  } else {
    auto ub = m_updates.upper_bound(timestamp);
    if (ub == m_updates.begin() || ub == m_updates.end()) {
      return;
    }
    auto lb = ub;
    lb--;
    if (lb == m_updates.end()) {
      return;
    }

    // create two partial twists, interpolating between drivebase measurements
    const double time0 = lb->first;    
    const double time1 = ub->first;
    const vec::Vector2D deltaPos = ub->second.deltaPos;

    const vec::Vector2D deltaPos0 = deltaPos * ((timestamp - time0) / (time1 - time0));
    const vec::Vector2D deltaPos1 = deltaPos * ((time1 - timestamp) / (time1 - time0));

    // add vision updates as a new pose update
    std::vector<VisionUpdate> newVisionUpdates;
    newVisionUpdates.push_back(curVis);
    std::sort(newVisionUpdates.begin(), newVisionUpdates.end(), cmp);

    m_updates[timestamp] = {deltaPos0, newVisionUpdates};
    ub->second.deltaPos = deltaPos1;
  }

  Update();
}

/**
 * Updates pose estimator
*/
void PoseEstimator::Update() {
  // clear old data
  const double curTimeS = Utils::GetCurTimeS();
  while (m_updates.size() > 1 && m_updates.begin()->first < curTimeS - MAX_HISTORY_TIME) {
    PoseUpdate update = m_updates.begin()->second;
    m_basePose = update.Apply(m_basePose, m_q);
    m_updates.erase(m_updates.begin());
  }

  // update cur pose
  m_curPose = m_basePose;
  for (auto it = m_updates.begin(); it != m_updates.end(); it++) {
    m_curPose = it->second.Apply(m_curPose, m_q);
  }
}

/**
 * Applies pose update
 * 
 * @param lastPose Last pose
 * @param q System std dev
 * 
 * @returns applies pose update
*/
vec::Vector2D PoseEstimator::PoseUpdate::Apply(const vec::Vector2D &lastPose, const vec::Vector2D &q) {
  vec::Vector2D pose = lastPose + deltaPos;
  for (VisionUpdate update : visUpdates) {
    vec::Vector2D k;
    for (int i = 0; i < 2; i++) {
      if (q[i] == 0) {
        k[i] = 0;
      } else {
        k[i] = q[i] / (q[i] + std::sqrt(q[i] * update.stdDev[i] * update.stdDev[i]));
      }
    }
    vec::Vector2D deltaVision = update.pos - pose;
    deltaVision = Utils::MultiplyComps(deltaVision, k);
    pose += deltaVision;
  }

  return pose;
}

/**
 * Sets cur pose
 * 
 * @param pose Pose
*/
void PoseEstimator::SetPos(const vec::Vector2D &pose) {
  m_basePose = pose;
  m_updates.clear();
  Update();
}

/**
 * Sets Standard Deviation value
 * 
 * @param stdDev std dev value
*/
void PoseEstimator::SetQ(const vec::Vector2D &stdDev) {
  m_q = Utils::MultiplyComps(stdDev, stdDev);
}

/**
 * Gets latest position
 * 
 * @returns Latest pos
*/
vec::Vector2D PoseEstimator::GetCurPos() const {
  return m_curPose;
}