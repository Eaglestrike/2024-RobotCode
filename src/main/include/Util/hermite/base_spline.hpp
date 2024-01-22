/**
 * @file
 *
 * A base class for interpolating splines
 */

#pragma once

#include <cstddef>

#include "Util/hermite/base_interpol.hpp"
#include "Util/simplevectors.hpp"

namespace hermite {
using svector::Vector;

/**
 * @brief Abstract base class for interpolating splines
 */
template <std::size_t D> class BaseSpline : public BaseInterpol<D> {
public:
  /**
   * @brief Destructor
   */
  virtual ~BaseSpline() = default;

  /**
   * @brief Gets the lower bound of the domain of the piecewise spline function,
   * which is the first time (lowest t-value) listed in the waypoints.
   *
   * @note If there are no waypoints, then returns 0
   *
   * @returns The first time measurement
   */
  virtual double getLowestTime() const = 0;

  /**
   * @brief Gets the upper bound of the domain of the piecewise spline function,
   * which is the last time (highest t-value) listed in the waypoints.
   *
   * @note If there are no waypoints, then returns 0
   *
   * @returns The last time measurement
   */
  virtual double getHighestTime() const = 0;

  /**
   * @brief Gets value of the interpolation function at a certain point
   *
   * Same as calling operator()()
   *
   * @param t Input to get value from
   *
   * @returns Output from function
   */
  virtual Vector<D> getPos(const double t) const = 0;

  /**
   * @brief Gets derivative of the function at a certain point
   *
   * This will usually be the velocity.
   *
   * @param t Input to get value from
   *
   * @returns Output from function's derivative
   */
  virtual Vector<D> getVel(const double t) const = 0;

  /**
   * @brief Gets second derivative of the function at a certain point
   *
   * This will usually be the acceleration.
   *
   * @param t Input to get value from
   *
   * @returns Output from function's second derivative
   */
  virtual Vector<D> getAcc(const double t) const = 0;

  /**
   * @brief Gets maximum distance from origin
   *
   * @param timeStep The time step to try for the absolute maximum
   *
   * @note This function will take much longer for smaller timesteps.
   * Recommended is between 0.001 and 0.1, but this also depends on the domain
   * of your function.
   * @note If no waypoints, returns 0.
   *
   * @returns Maximum distance from the origin.
   */
  double getMaxDistance(const double timeStep) const {
    double res = 0.0;
    double time = getLowestTime();
    const double timeEnd = getHighestTime();

    while (time <= timeEnd) {
      auto pos = getPos(time);
      double dist = magn(pos);
      res = std::max(res, dist);

      time += timeStep;
    }

    return res;
  }

  /**
   * @brief Gets maximum speed
   *
   * @param timeStep The time step to try for the absolute maximum
   *
   * @note This function will take much longer for smaller timesteps.
   * Recommended is between 0.001 and 0.1, but this also depends on the domain
   * of your function.
   * @note If no poses, returns 0.
   *
   * @returns Maximum speed.
   */
  double getMaxSpeed(const double timeStep) const {
    double res = 0.0;
    double time = getLowestTime();
    const double timeEnd = getHighestTime();

    while (time <= timeEnd) {
      auto vel = getVel(time);
      double dist = magn(vel);
      res = std::max(res, dist);

      time += timeStep;
    }

    return res;
  }

  /**
   * @brief Gets maximum magnitude of acceleration
   *
   * @param timeStep The time step to try for the absolute maximum
   *
   * @note This function will take much longer for smaller timesteps.
   * Recommended is between 0.001 and 0.1, but this also depends on the domain
   * of your function.
   * @note If no poses, returns 0.
   *
   * @returns Magnitude of maximum acceleration.
   */
  double getMaxAcceleration(const double timeStep) const {
    double res = 0.0;
    double time = getLowestTime();
    const double timeEnd = getHighestTime();

    while (time <= timeEnd) {
      auto acc = getAcc(time);
      double dist = magn(acc);
      res = std::max(res, dist);

      time += timeStep;
    }

    return res;
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
  virtual double getLength(const double timeStep) const = 0;
};
} // namespace hermite
