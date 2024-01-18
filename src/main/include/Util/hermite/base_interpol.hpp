/**
 * @file
 *
 * Base class for all interpolation classes
 */

#pragma once

#include <cstddef>

#include "Util/simplevectors.hpp"

namespace hermite {
using svector::Vector;

/**
 * @brief Base class for interpolation functions
 *
 * In the class, you must specify the number of dimensions that you want to
 * interpolate. For example, if you want to calculate a 2D position over time,
 * then speficy "2" in the template.
 */
template <std::size_t D> class BaseInterpol {
public:
  /**
   * @brief Destructor
   */
  virtual ~BaseInterpol() = default;

  /**
   * @brief Gets value of the interpolation function at a certain point
   *
   * Same as calling getPos()
   *
   * @param t Input to get value from
   *
   * @returns Output from function
   */
  virtual Vector<D> operator()(const double t) const { return getPos(t); }

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
};
} // namespace hermite
