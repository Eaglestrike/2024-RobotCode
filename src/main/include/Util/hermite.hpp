/**
 * @file
 *
 * The main hermite spline class
 */

#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <map>
#include <vector>

#include "Util/hermite/base_spline.hpp"
#include "Util/hermite/hermite_sub.hpp"
#include "Util/hermite/pose.hpp"
#include "Util/simplevectors.hpp"

namespace hermite {
using svector::magn;

/**
 * @brief Hermite spline class
 *
 * Given a set of poses (i.e. a desired position and velocity at a certain
 * time), this class interpolates a path between these poses by using Hermite
 * splines. It does this by interpolating a Hermite function between each pose
 * using HermiteSub, and matching the final position and velocity of one pose
 * with the initial position and velocity of the next.
 *
 * This class can work with multiple dimensions, specified in the template
 * argument. For example, if the path you want to interpolate is 2-dimensional,
 * then specify "2" in the template argument.
 *
 * This class acts like a std::map, where the key is the time. This is to ensure
 * maximum efficiency inserting, removing, changing, and searching for the
 * interpoloation equations. However, because time is a floating point number,
 * and they are always imprecise, it needs to be converted into an integer. It
 * does this by multiplying the time by a certain multiplier and truncating the
 * rest of the digits. This is fine for most cases, as the time does not need to
 * be super precise.
 *
 * Hermite curves allow for fast local control, meaning that inserting and
 * removing points is quick. However, it only provides C1 continuity, which
 * means that there may be high jerk at knot points (as acceleration is
 * discontinuous).
 */
template <std::size_t D> class Hermite : public BaseSpline<D> {
public:
  /**
   * @brief Default constructor
   *
   * Sets multiplier to 10, meaning that it shifts the decimal place of the
   * given time by 1 digit before truncating the rest of the number whenever it
   * is stored as a waypoint.
   */
  Hermite() : m_multiplier{10LL} {}

  /**
   * @brief Constructor
   *
   * @param multiplier Multiplies the time given in the pose by the multiplier,
   * then truncates the rest of the digits when storing the waypoint.
   */
  Hermite(const double multiplier) : m_multiplier{multiplier} {}

  /**
   * @brief Copy constructor
   */
  Hermite(const Hermite<D> &other)
      : m_multiplier{other.m_multiplier}, m_waypoints{other.m_waypoints} {}

  /**
   * @brief Assignment operator
   *
   * @note All data stored in current Hermite object will be lost.
   */
  Hermite<D> &operator=(const Hermite<D> &other) {
    // check if assigning to self
    if (this == &other) {
      return *this;
    }

    m_multiplier = other.m_multiplier;
    m_waypoints = other.m_waypoints;

    return *this;
  }

  /**
   * @brief Destructor
   */
  ~Hermite() override = default;

  /**
   * @brief Inserts a waypoint
   *
   * @param waypoint Waypoint to insert
   *
   * @note If the waypoint's time exists (rounded according to the multiplier),
   * then this method does nothing.
   */
  void insert(const Pose<D> &waypoint) {
    if (exists(waypoint)) {
      return;
    }

    auto tRounded = roundTime(waypoint.getTime());
    m_waypoints[tRounded] = waypoint;
  }

  /**
   * @brief Replaces a waypoint
   *
   * @param waypoint Waypoint to replace
   *
   * @note Gets the time from the waypoint.
   * @note If the waypoint does not exist, then this method does not change
   * anything.
   */
  void replace(const Pose<D> &waypoint) {
    if (!exists(waypoint)) {
      return;
    }

    auto tRounded = roundTime(waypoint.getTime());
    m_waypoints[tRounded] = waypoint;
  }

  /**
   * @brief Inserts a waypoint if it doesn't exist, otherwise replaces the
   * waypoint.
   *
   * @param waypoint Waypoint to insert or replace
   */
  void insertOrReplace(const Pose<D> &waypoint) {
    auto tRounded = roundTime(waypoint.getTime());
    m_waypoints[tRounded] = waypoint;
  }

  /**
   * @brief Checks if a waypoint exists
   *
   * @param waypoint Checks if this waypoint exists
   *
   * @note Only uses the time argument of the waypoint.
   *
   * @returns If waypoint exists.
   */
  bool exists(const Pose<D> &waypoint) const {
    return exists(waypoint.getTime());
  }

  /**
   * @brief Checks if a waypoint at a certain time exists
   *
   * @param time Checks if a waypoint exists at this time
   *
   * @returns If waypoint exists.
   */
  bool exists(const double time) const {
    const auto tRounded = roundTime(time);
    return m_waypoints.find(tRounded) != m_waypoints.end();
  }

  /**
   * @brief Removes a waypoint
   *
   * @param waypoint Waypoint to remove
   *
   * @note Only the waypoint's time is used in this method.
   * @note The waypoint's time is rounded by using the multplier
   * @note If the waypoint's time is not precise enough, then this may erase a
   * waypoint even though it might not be intended.
   */
  void erase(const Pose<D> &waypoint) { erase(waypoint.getTime()); }

  /**
   * @brief Removes a waypoint
   *
   * @param time Time of waypoint to remove
   *
   * @note The waypoint's time is rounded by using the multplier
   * @note If the waypoint's time is not precise enough, then this may erase a
   * waypoint even though it might not be intended.
   */
  void erase(const double time) {
    const auto tRounded = roundTime(time);

    // check if exists
    auto it = m_waypoints.find(tRounded);
    if (it == m_waypoints.end()) {
      return;
    }

    m_waypoints.erase(it);
  }

  /**
   * @brief Gets a list of all waypoints
   *
   * @returns A list of all waypoints, sorted in order of time.
   */
  std::vector<Pose<D>> getAllWaypoints() const {
    std::vector<Pose<D>> res;
    for (const auto &it : m_waypoints) {
      res.push_back(it.second);
    }

    return res;
  }

  /**
   * @brief Gets the lower bound of the domain of the piecewise spline function,
   * which is the first time (lowest t-value) listed in the waypoints.
   *
   * @note If there are no waypoints, then returns 0
   *
   * @returns The first time measurement
   */
  double getLowestTime() const override {
    if (m_waypoints.size() == 0) {
      return 0;
    }

    auto firstIt = m_waypoints.begin();
    return firstIt->second.getTime();
  }

  /**
   * @brief Gets the upper bound of the domain of the piecewise spline function,
   * which is the last time (highest t-value) listed in the waypoints.
   *
   * @note If there are no waypoints, then returns 0
   *
   * @returns The last time measurement
   */
  double getHighestTime() const override {
    if (m_waypoints.size() == 0) {
      return 0;
    }

    auto lastIt = m_waypoints.rbegin();
    return lastIt->second.getTime();
  }

  /**
   * @brief Gets position at a certain time
   *
   * Same as calling operator()()
   *
   * @note If time is outside the domain of time from the given points, then it
   * calculates the value for the function whose domain is nearest to t.
   * @note If number of waypoints is less than or equal to 1, then returns a
   * zero vector.
   *
   * @param t Time
   *
   * @returns Position
   */
  Vector<D> getPos(const double t) const override {
    Vector<D> res;
    if (m_waypoints.size() < 2) {
      return res;
    }

    auto func = getSub(t);
    return func.getPos(t);
  }

  /**
   * @brief Gets velocity at a certain time
   *
   * @note If time is outside the domain of time from the given points, then it
   * calculates the value for the function whose domain is nearest to t.
   * @note If number of waypoints is less than or equal to 1, then returns a
   * zero vector.
   *
   * @param t Time
   *
   * @returns Velocity
   */
  Vector<D> getVel(const double t) const override {
    Vector<D> res;
    if (m_waypoints.size() < 2) {
      return res;
    }

    auto func = getSub(t);
    return func.getVel(t);
  }

  /**
   * @brief Gets acceleration of the function at a certain time
   *
   * @note If time is outside the domain of time from the given points, then it
   * calculates the value for the function whose domain is nearest to t.
   * @note If number of waypoints is less than or equal to 1, then returns a
   * zero vector.
   * @note If called on a border between two functions, will take the value of
   * the latter function.
   *
   * @param t Time
   *
   * @returns Acceleration
   */
  Vector<D> getAcc(const double t) const override {
    Vector<D> res;
    if (m_waypoints.size() < 2) {
      return res;
    }

    auto func = getSub(t);
    return func.getAcc(t);
  }

  /**
   * @brief Gets arc length
   *
   * @param timeStep The time step to try for the arc length
   *
   * @note This function will take much longer for smaller timesteps.
   * Recommended is between 0.001 and 0.1, but this also depends on the domain
   * of your function.
   * @note If zero or one poses, returns 0.
   *
   * @returns Arc length
   */
  double getLength(const double timeStep) const override {
    double res = 0.0;

    if (m_waypoints.size() < 2) {
      return res;
    }

    double time = getLowestTime() + timeStep;
    const double timeEnd = getHighestTime();
    while (time <= timeEnd) {
      auto vel = getVel(time);
      auto speed = magn(vel);
      res += speed * timeStep;

      time += timeStep;
    }

    return res;
  }

private:
  double m_multiplier;
  std::map<std::int64_t, Pose<D>> m_waypoints;

  /**
   * @brief Rounds time to int
   *
   * @param t Time to round
   *
   * @returns Rounded time
   */
  std::int64_t roundTime(double t) const {
    t *= m_multiplier;
    return static_cast<std::int64_t>(t);
  }

  /**
   * Gets hermite subinterval given a certain time
   *
   * @note Assumes that number of waypoints is greater than or equal to 2. If
   * not, results in undefined behavior.
   *
   * @returns Hermite subinterval
   */
  HermiteSub<D> getSub(const double t) const {
    const auto tRound = roundTime(t);
    auto itLower = m_waypoints.upper_bound(tRound);

    if (itLower == m_waypoints.begin()) {
      itLower++;
    }

    if (itLower == m_waypoints.end()) {
      itLower--;
    }

    auto itUpper = itLower;
    itLower--;

    const auto objUpper = itUpper->second;
    const auto objLower = itLower->second;

    const auto p0 = objLower.getPos();
    const auto pf = objUpper.getPos();
    const auto v0 = objLower.getVel();
    const auto vf = objUpper.getVel();
    const auto lowerT = objLower.getTime();
    const auto upperT = objUpper.getTime();

    HermiteSub<D> res{p0, pf, v0, vf, lowerT, upperT};
    return res;
  }
};
} // namespace hermite
