/**
 * @file
 *
 * Containes pose data structure
 */

#pragma once

#include <cstddef>

#include "Util/simplevectors.hpp"

namespace hermite {
using svector::Vector;

/**
 * @brief A pose object
 *
 * Poses in multiple dimensions can use multiple Pose objects.
 *
 * It is recommended to create a pose using makePose().
 */
template <std::size_t D> class Pose {
public:
  /**
   * @brief Default constructor
   *
   * Initializes all values to zero
   */
  Pose() : m_time{0} {}

  /**
   * @brief Constructor
   *
   * @param time Time to reach this pose
   * @param pos Position vector
   * @param vel Velocity vector
   */
  Pose(const double time, const Vector<D> pos, const Vector<D> vel)
      : m_time{time}, m_pos{pos}, m_vel{vel} {}

  /**
   * @brief Copy constructor
   */
  Pose(const Pose<D> &other)
      : m_time{other.m_time}, m_pos{other.m_pos}, m_vel{other.m_vel} {}

  /**
   * @brief Assignment operator
   */
  Pose<D> &operator=(const Pose<D> &other) {
    // check if assigning to self
    if (this == &other) {
      return *this;
    }

    m_pos = other.m_pos;
    m_vel = other.m_vel;
    m_time = other.m_time;

    return *this;
  }

  /**
   * @brief Destructor
   */
  ~Pose() = default;

  /**
   * @brief Gets position vector
   *
   * @returns Position vector
   */
  Vector<D> getPos() const { return m_pos; }

  /**
   * @brief Gets velocity vector
   *
   * @returns Velocity vector
   */
  Vector<D> getVel() const { return m_vel; }

  /**
   * @brief Gets time
   *
   * @returns time
   */
  double getTime() const { return m_time; }

  /**
   * @brief Sets position vector
   *
   * @param pos Position vector
   */
  void setPos(const Vector<D> &pos) { m_pos = pos; }

  /**
   * @brief Sets velocity vector
   *
   * @param vel Velocity vector
   */
  void setVel(const Vector<D> &vel) { m_vel = vel; }

  /**
   * @brief Sets time
   *
   * @param time Time
   */
  void setTime(const double &time) { m_time = time; }

private:
  double m_time;
  Vector<D> m_pos;
  Vector<D> m_vel;
};
} // namespace hermite
