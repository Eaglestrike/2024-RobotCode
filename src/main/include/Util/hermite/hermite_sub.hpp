/**
 * @file
 *
 * A hermite spline on a subinterval
 */

#pragma once

#include <algorithm>
#include <cstddef>

#include "Util/hermite/base_interpol.hpp"
#include "Util/hermite/constants.hpp"
#include "Util/hermite/hermite_unit.hpp"

namespace hermite {
/**
 * @brief A hermite spline on a subinterval
 *
 * Allows for two points on an arbitrary interval to be interpolated, not just
 * at 0 and 1. It does this through an affline transformation.
 */
template <std::size_t D> class HermiteSub : public BaseInterpol<D> {
public:
  /**
   * @brief Default constructor
   *
   * Initializes lower bound to 0 and upper bound to 1, and the function is
   * going to be zero for all input values.
   */
  HermiteSub() : m_lower{0}, m_upper{1} {};

  /**
   * @brief Constructor
   *
   * @param p0 Initial position vector
   * @param pf Final position vector
   * @param v0 Initial velocity vector
   * @param vf Final velocity vector
   * @param lower Lower bound
   * @param upper Upper bound
   *
   * @note If lower >= upper, undefined behavior occurs.
   */
  HermiteSub(const Vector<D> p0, const Vector<D> pf, const Vector<D> v0,
             const Vector<D> vf, const double lower, const double upper)
      : m_lower{lower}, m_upper{upper}, m_unit{p0, pf, v0 * (upper - lower),
                                               vf * (upper - lower)} {};

  /**
   * @brief Copy constructor
   */
  HermiteSub(const HermiteSub<D> &other)
      : m_lower{other.m_lower}, m_upper{other.m_upper}, m_unit{other.m_unit} {}

  /**
   * @brief Assignment operator
   */
  HermiteSub<D> &operator=(const HermiteSub<D> &other) {
    // check if assigning to self
    if (this == &other) {
      return *this;
    }

    m_unit = other.m_unit;
    m_lower = other.m_lower;
    m_upper = other.m_upper;

    return *this;
  }

  /**
   * @brief Destructor
   */
  ~HermiteSub() override = default;

  /**
   * @brief Gets position at a certain time
   *
   * Same as calling operator()()
   *
   * @note If t is outside of the given interval, then it still calculates
   * vector.
   *
   * @param t Time
   *
   * @returns Position
   */
  Vector<D> getPos(const double t) const override {
    Vector<D> res;
    const double tNew = (t - m_lower) / (m_upper - m_lower);
    res = m_unit.getPos(tNew);
    return res;
  }

  /**
   * @brief Gets velocity at a certain time
   *
   * @note If t is outside of the given interval, then it still calculates
   * vector.
   *
   * @param t Time
   *
   * @returns Velocity
   */
  Vector<D> getVel(const double t) const override {
    Vector<D> res;
    const double tNew = (t - m_lower) / (m_upper - m_lower);
    res = m_unit.getVel(tNew) * (1 / (m_upper - m_lower)); // multiply by chain
    return res;
  }

  /**
   * @brief Gets acceleration of the function at a certain time
   *
   * @note If t is outside of the given interval, then it still calculates
   * vector.
   *
   * @param t Time
   *
   * @returns Acceleration
   */
  Vector<D> getAcc(const double t) const override {
    Vector<D> res;
    const double tNew = (t - m_lower) / (m_upper - m_lower);
    res = m_unit.getAcc(tNew) * (1 / (m_upper - m_lower)) *
          (1 / (m_upper -
                m_lower)); // multiply by chain; don't need to add product rule
                           // because second derivative of tNew(t) = 0
    return res;
  }

private:
  double m_lower;
  double m_upper;
  HermiteUnit<D> m_unit;
};
} // namespace hermite
