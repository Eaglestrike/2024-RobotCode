/**
 * @file
 *
 * Interpolation on the unit interval
 */

#pragma once

#include <cstddef>

#include "Util/hermite/base_interpol.hpp"
#include "Util/hermite/constants.hpp"
#include "Util/simplevectors.hpp"

namespace hermite {
using svector::Vector;

/**
 * @brief Interpolates on the unit interval
 *
 * Calculates one Hermite spline section on the unit interval [0, 1] given a
 * starting point and velocity at time t=0 and an ending point and velocity at
 * t=tf.
 *
 * The template represents the number of dimensions to calculate in. For
 * example, for 2 dimensions, the position and velocity functions will output a
 * 2D vector.
 */
template <std::size_t D> class HermiteUnit : public BaseInterpol<D> {
public:
  /**
   * @brief Default constructor
   *
   * Initializing a unit Hermite interpolation curve with the default
   * constructor creates a curve that is a constant 0.
   */
  HermiteUnit() = default;

  /**
   * @brief Constructor
   *
   * @param p0 Initial position vector
   * @param p1 Final position vector
   * @param v0 Initial velocity vector
   * @param v1 Final velocity vector
   */
  HermiteUnit(const Vector<D> p0, const Vector<D> p1, const Vector<D> v0,
              const Vector<D> v1)
      : m_p0{p0}, m_p1{p1}, m_v0{v0}, m_v1{v1} {}

  /**
   * @brief Copy constructor
   */
  HermiteUnit(const HermiteUnit<D> &other)
      : m_p0{other.m_p0}, m_p1{other.m_p1}, m_v0{other.m_v0}, m_v1{other.m_v1} {
  }

  /**
   * @brief Assignment operator
   */
  HermiteUnit<D> &operator=(const HermiteUnit<D> &other) {
    // check if assigning to self
    if (this == &other) {
      return *this;
    }

    m_p0 = other.m_p0;
    m_p1 = other.m_p1;
    m_v0 = other.m_v0;
    m_v1 = other.m_v1;

    return *this;
  }

  /**
   * @brief Destructor
   */
  ~HermiteUnit() override = default;

  /**
   * @brief Gets position at a certain time
   *
   * Same as calling operator()()
   *
   * @note If t is outside of [0, 1], then it still calculates the vector at
   * that time.
   *
   * @param t Time
   *
   * @returns Position
   */
  Vector<D> getPos(const double t) const override {
    Vector<D> res;
    res = m_p0 * h00(t) + m_v0 * h10(t) + m_p1 * h01(t) + m_v1 * h11(t);
    return res;
  }

  /**
   * @brief Gets velocity at a certain time
   *
   * @note If t is outside of [0, 1], then it still calculates the vector at
   * that time.
   *
   * @param t Time
   *
   * @returns Velocity
   */
  Vector<D> getVel(const double t) const override {
    Vector<D> res;
    res = m_p0 * h00d(t) + m_v0 * h10d(t) + m_p1 * h01d(t) + m_v1 * h11d(t);
    return res;
  }

  /**
   * @brief Gets acceleration of the function at a certain time
   *
   * @note If t is outside of [0, 1], then it still calculates the vector at
   * that time.
   *
   * @param t Time
   *
   * @returns Acceleration
   */
  Vector<D> getAcc(const double t) const override {
    Vector<D> res;
    res = m_p0 * h00dd(t) + m_v0 * h10dd(t) + m_p1 * h01dd(t) + m_v1 * h11dd(t);
    return res;
  }

private:
  Vector<D> m_p0;
  Vector<D> m_p1;
  Vector<D> m_v0;
  Vector<D> m_v1;
};
} // namespace hermite
