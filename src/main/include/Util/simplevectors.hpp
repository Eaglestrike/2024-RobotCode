/**
 * @file simplevectors.hpp
 *
 * @internal
 * The MIT License (MIT)
 *
 * Copyright (c) 2023 Jonathan Liu
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * @endinternal
 */

#ifndef INCLUDE_SVECTORS_SIMPLEVECTORS_HPP_
#define INCLUDE_SVECTORS_SIMPLEVECTORS_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <string>
#include <type_traits>
#include <vector>

namespace svector {
/**
 * @brief Angle enumerator
 *
 * An enum representing the angle to use for a 3D vector.
 *
 * This is only used in svector::Vector3D::angle() and
 * svector::Vector3D::rotate().
 */
enum AngleDir {
  ALPHA, //!< Angle between positive x-axis and vector
  BETA,  //!< Angle between positive y-axis and vector
  GAMMA  //!< Angle between positive z-axis and vector
};
/**
 * @brief A base vector representation.
 *
 * @note The binary +, -, *, /, ==, and != operators are by default implemented
 * in functions.hpp. To use the class implementation rather than the one in
 * functions.hpp, define the variable SVECTOR_USE_CLASS_OPERATORS.
 *
 * @tparam D The number of dimensions.
 * @tparam T Vector type.
 */
template <std::size_t D, typename T = double> class Vector {
public:
  // makes sure that type is numeric
  static_assert(std::is_arithmetic<T>::value, "Vector type must be numeric");

#ifdef SVECTOR_EXPERIMENTAL_COMPARE
  template <std::size_t D1, std::size_t D2, typename T1, typename T2>
  friend bool operator<(const Vector<D1, T1> &, const Vector<D2, T2> &);

  template <std::size_t D1, std::size_t D2, typename T1, typename T2>
  friend bool operator<=(const Vector<D1, T1> &, const Vector<D2, T2> &);

  template <std::size_t D1, std::size_t D2, typename T1, typename T2>
  friend bool operator>(const Vector<D1, T1> &, const Vector<D2, T2> &);

  template <std::size_t D1, std::size_t D2, typename T1, typename T2>
  friend bool operator>=(const Vector<D1, T1> &, const Vector<D2, T2> &);
#endif

  typedef
      typename std::array<T, D>::iterator iterator; //!< An std::array::iterator
  typedef typename std::array<T, D>::const_iterator
      const_iterator; //!< An std::array::const_iterator
  typedef typename std::array<T, D>::reverse_iterator
      reverse_iterator; //!< An std::array::reverse_iterator
  typedef typename std::array<T, D>::const_reverse_iterator
      const_reverse_iterator; //!< An std::array::const_reverse_iterator

  /**
   * @brief No-argument constructor
   *
   * Initializes a zero vector (all components are 0).
   */
  Vector() { this->m_components.fill(0); }

  /**
   * @brief Initializes a vector given initializer list
   *
   * The initializer list should represent the components of the vector in each
   * dimension. If the size of the initializer list is greater than the number
   * of dimensions given, then the vector only initializes with the first D
   * elements in the list, where D is the number of dimensions. If the size of
   * the initializer list is less than the number of dimensions given, then the
   * vector fills the rest of the dimensions with the value 0.
   *
   * @param args the initializer list.
   */
  Vector(const std::initializer_list<T> args) {
    // in case length of args < dimensions
    this->m_components.fill(0);

    std::size_t counter = 0;
    for (const auto &num : args) {
      if (counter >= D) {
        break;
      }

      this->m_components[counter] = num;
      counter++;
    }
  }

  /**
   * @brief Copy constructor
   *
   * Copies from another vector to an uninitialized vector.
   */
  Vector(const Vector<D, T> &other) {
    for (std::size_t i = 0; i < D; i++) {
      this->m_components[i] = other[i];
    }
  }

  /**
   * @brief Move constructor
   *
   * Uses C++ default move constructor.
   */
  Vector(Vector<D, T> &&) noexcept = default;

  /**
   * @brief Assignment operator
   *
   * Copies from another vector to a vector whose values already exist.
   */
  Vector<D, T> &operator=(const Vector<D, T> &other) {
    // check if assigning to self
    if (this == &other) {
      return *this;
    }

    for (std::size_t i = 0; i < D; i++) {
      this->m_components[i] = other[i];
    }

    return *this;
  }

  /**
   * @brief Move assignment operator
   *
   * Uses C++ default move assignment operator.
   */
  Vector<D, T> &operator=(Vector<D, T> &&) noexcept = default;

  /**
   * @brief Destructor
   *
   * Uses C++ default destructor.
   */
  virtual ~Vector() = default;

  /**
   * @brief Returns string form of vector
   *
   * This string form can be used for printing.
   *
   * @returns The string form of the vector.
   */
  virtual std::string toString() const {
    std::string str = "<";
    for (std::size_t i = 0; i < D - 1; i++) {
      str += std::to_string(this->m_components[i]);
      str += ", ";
    }

    str += std::to_string(this->m_components[D - 1]);
    str += ">";

    return str;
  }

#ifdef SVECTOR_USE_CLASS_OPERATORS
  /**
   * @brief Vector addition
   *
   * Performs vector addition and returns a new vector representing the sum of
   * the two vectors.
   *
   * @note This method is only used if SVECTOR_USE_CLASS_OPERATORS is
   * defined. Otherwise, the binary operators in functions.hpp are used.
   *
   * @note The dimensions of the other vector must be the same
   * as the current one.
   *
   * @param other The other vector.
   *
   * @returns A new vector representing the vector sum.
   */
  Vector<D, T> operator+(const Vector<D, T> &other) const {
    Vector<D, T> tmp;
    for (std::size_t i = 0; i < D; i++) {
      tmp[i] = this->m_components[i] + other[i];
    }

    return tmp;
  }

  /**
   * @brief Vector subtraction
   *
   * Performs vector subtraction and returns a new vector representing the
   * difference of the two vectors.
   *
   * @note This method is only used if SVECTOR_USE_CLASS_OPERATORS is
   * defined. Otherwise, the binary operators in functions.hpp are used.
   *
   * @note The dimensions of the other vector must be the same
   * as the current one.
   *
   * @param other The other vector.
   *
   * @returns A new vector representing the vector difference.
   */
  Vector<D, T> operator-(const Vector<D, T> &other) const {
    Vector<D, T> tmp;
    for (std::size_t i = 0; i < D; i++) {
      tmp[i] = this->m_components[i] - other[i];
    }

    return tmp;
  }

  /**
   * @brief Scalar multiplication
   *
   * Performs scalar multiplication and returns a new vector representing the
   * product.
   *
   * @note This method is only used if SVECTOR_USE_CLASS_OPERATORS is
   * defined. Otherwise, the binary operators in functions.hpp are used.
   *
   * @param other The other vector.
   *
   * @returns A new vector representing the scalar product.
   */
  Vector<D, T> operator*(const T other) const {
    Vector<D, T> tmp;
    for (std::size_t i = 0; i < D; i++) {
      tmp[i] = this->m_components[i] * other;
    }

    return tmp;
  }

  /**
   * @brief Scalar division
   *
   * Performs scalar division and returns a new vector representing the
   * quotient.
   *
   * @note This method is only used if SVECTOR_USE_CLASS_OPERATORS is
   * defined. Otherwise, the binary operators in functions.hpp are used.
   *
   * @param other The other vector.
   *
   * @returns A new vector representing the scalar quotient.
   */
  Vector<D, T> operator/(const T other) const {
    Vector<D, T> tmp;
    for (std::size_t i = 0; i < D; i++) {
      tmp[i] = this->m_components[i] / other;
    }

    return tmp;
  }

  /**
   * @brief Compares equality of two vectors.
   *
   * @note This method is only used if SVECTOR_USE_CLASS_OPERATORS is
   * defined. Otherwise, the binary operators in functions.hpp are used.
   *
   * @note The dimensions of the other vector must be the same
   * as the current one.
   *
   * @param other The other vector.
   *
   * @returns A boolean representing whether the vectors compare equal.
   */
  bool operator==(const Vector<D, T> &other) const {
    for (std::size_t i = 0; i < D; i++) {
      if (this->m_components[i] != other[i]) {
        return false;
      }
    }

    return true;
  }

  /**
   * @brief Compares inequality of two vectors.
   *
   * @note This method is only used if SVECTOR_USE_CLASS_OPERATORS is
   * defined. Otherwise, the binary operators in functions.hpp are used.
   *
   * @note The dimensions of the other vector must be the same
   * as the current one.
   *
   * @param other The other vector.
   *
   * @returns A boolean representing whether the vectors do not compare equal.
   */
  bool operator!=(const Vector<D, T> &other) const {
    return !((*this) == other);
  }
#endif

  /**
   * @brief Negative of a vector
   *
   * Makes all components of the vector negative of what they currently are.
   *
   * This can also be thought of flipping the direction of the vector.
   *
   * @returns A new vector representing the negative of the current vector.
   */
  Vector<D, T> operator-() const {
    Vector<D, T> tmp;
    for (std::size_t i = 0; i < D; i++) {
      tmp[i] = -this->m_components[i];
    }

    return tmp;
  }

  /**
   * @brief Positive of a vector
   *
   * Creates new vector where the unary plus operator is applied to each
   * component. In almost all cases, this returns the original vector.
   *
   * @returns The current vector.
   */
  Vector<D, T> operator+() const {
    Vector<D, T> tmp;
    for (std::size_t i = 0; i < D; i++) {
      tmp[i] = +this->m_components[i];
    }

    return tmp;
  }

  /**
   * @brief In-place addition
   *
   * Adds another vector object to the current object.
   *
   * @param other The other vector to add.
   */
  Vector<D, T> &operator+=(const Vector<D, T> &other) {
    for (std::size_t i = 0; i < D; i++) {
      this->m_components[i] += other[i];
    }

    return *this;
  }

  /**
   * @brief In-place subtraction
   *
   * Subtracts another vector object from the current object.
   *
   * @param other The other vector to subtract.
   */
  Vector<D, T> &operator-=(const Vector<D, T> &other) {
    for (std::size_t i = 0; i < D; i++) {
      this->m_components[i] -= other[i];
    }

    return *this;
  }

  /**
   * @brief In-place scalar multiplication
   *
   * Performs scalar multiplication on the current object.
   *
   * @param other The number to multiply by.
   */
  Vector<D, T> &operator*=(const T other) {
    for (std::size_t i = 0; i < D; i++) {
      this->m_components[i] *= other;
    }

    return *this;
  }

  /**
   * @brief In-place scalar division
   *
   * Performs scalar division on the current object.
   *
   * @param other The number to divide by.
   */
  Vector<D, T> &operator/=(const T other) {
    for (std::size_t i = 0; i < D; i++) {
      this->m_components[i] /= other;
    }

    return *this;
  }

  /**
   * @brief Dot product
   *
   * Calculates the dot product of two vectors.
   *
   * @note The dimensions of the other vector must be the same
   * as the current one.
   *
   * @param other The other vector to dot the current vector with.
   *
   * @returns A new vector representing the dot product of the two vectors.
   */
  T dot(const Vector<D, T> &other) const {
    T result = 0;

    for (std::size_t i = 0; i < D; i++) {
      result += this->m_components[i] * other[i];
    }

    return result;
  }

  /**
   * @brief Magnitude
   *
   * Gets the magnitude of the vector.
   *
   * @returns The magnitude of the vector.
   */
  T magn() const {
    T sum_of_squares = 0;

    for (const auto &i : this->m_components) {
      sum_of_squares += i * i;
    }

    return std::sqrt(sum_of_squares);
  };

  /**
   * @brief Normalizes a vector.
   *
   * Finds the unit vector with the same direction angle as the current vector.
   *
   * @note This method will result in undefined behavior if the vector is a zero
   * vector (if the magnitude equals zero).
   *
   * @returns A new vector representing the normalized vector.
   */
  Vector<D, T> normalize() const { return (*this) / this->magn(); }

  /**
   * @brief Gets the number of dimensions.
   *
   * @returns Number of dimensions.
   */
  constexpr std::size_t numDimensions() const { return D; }

  /**
   * @brief Determines whether the current vector is a zero vector.
   *
   * @returns Whether the current vector is a zero vector.
   */
  bool isZero() const { return this->magn() == 0; }

  /**
   * @brief Value of a certain component of a vector
   *
   * Gets a reference to a specific component of the vector given the dimension
   * number.
   *
   * @param index The dimension number.
   *
   * @returns A constant reference to that dimension's component of the vector.
   */
  const T &operator[](const std::size_t index) const {
    return this->m_components[index];
  }

  /**
   * @brief Sets value of a certain component
   *
   * Sets a certain component of the vector given the dimension number.
   *
   * @param index The dimension number.
   */
  T &operator[](const std::size_t index) { return this->m_components[index]; }

  /**
   * @brief Value of a certain component of a vector
   *
   * Gets a reference to a specific component of the vector given the dimension
   * number.
   *
   * Throws an out_of_range exception if the given number is out of bounds.
   *
   * @param index The dimension number.
   *
   * @returns A constant reference to that dimension's component of the vector.
   */
  const T &at(const std::size_t index) const {
    return this->m_components.at(index);
  }

  /**
   * @brief Sets value of a certain component
   *
   * Sets a certain component of the vector given the dimension number.
   *
   * Throws an out_of_range exception if the given number is out of bounds.
   *
   * @param index The dimension number.
   */
  T &at(const std::size_t index) { return this->m_components.at(index); }

  /**
   * @brief Iterator of first element
   *
   * Returns an iterator to the first dimension of the vector. This iterator
   * will be equal to end() for a zero-dimension vector.
   *
   * This can be used for looping through the dimensions of a vector.
   *
   * @returns An iterator to the first dimension of the vector.
   */
  iterator begin() noexcept { return iterator{this->m_components.begin()}; }

  /**
   * @brief Const interator of first element
   *
   * Returns a constant iterator to the first dimension of the vector. This
   * iterator will be equal to end() for a zero-dimension vector.
   *
   * @returns A constant iterator to the first dimension of the vector.
   */
  const_iterator begin() const noexcept {
    return const_iterator{this->m_components.begin()};
  }

  /**
   * @brief Interator of last element + 1
   *
   * Returns an iterator to the element following the last dimension of the
   * vector.
   *
   * This iterator is a placeholder and attempting to access it will result in
   * undefined behavior.
   *
   * This can be used for looping through the dimensions of a vector.
   *
   * @returns An iterator to the element following the last dimension.
   */
  iterator end() noexcept { return iterator{this->m_components.end()}; }

  /**
   * @brief Const interator of last element + 1
   *
   * Returns a constant iterator to the element following the last dimension of
   * the vector.
   *
   * This iterator is a placeholder and attempting to access it will result in
   * undefined behavior.
   *
   * @returns A constant iterator to the element following the last dimension.
   */
  const_iterator end() const noexcept {
    return const_iterator{this->m_components.end()};
  }

  /**
   * @brief Reverse iterator to last element
   *
   * Returns a reverse iterator to the first dimension of the reversed vector.
   * It corresponds to the last dimension of the original vector. The iterator
   * will be equal to rend() for a zero-dimension vector.
   *
   * This can be used for looping through the dimensions of a vector.
   *
   * @returns A reverse iterator to the first dimension.
   */
  reverse_iterator rbegin() noexcept {
    return reverse_iterator{this->m_components.rbegin()};
  }

  /**
   * @brief Const reverse iterator to last element
   *
   * Returns a constant reverse iterator to the first dimension of the reversed
   * vector. It corresponds to the last dimension of the original vector. The
   * iterator will be equal to rend() for a zero-dimension vector.
   *
   * @returns A constant reverse iterator to the first dimension.
   */
  const_reverse_iterator rbegin() const noexcept {
    return const_reverse_iterator{this->m_components.rbegin()};
  }

  /**
   * @brief Reverse iterator to first element - 1
   *
   * Returns a reverse iterator to the element following the last dimension of
   * the reversed vector. It corresponds to the element preceding the first
   * dimension of the original vector.
   *
   * This iterator is a placeholder and attempting to access it will result in
   * undefined behavior.
   *
   * This can be used for looping through the dimensions of a vector.
   *
   * @returns A reverse iterator to the element following the last dimension.
   */
  reverse_iterator rend() noexcept {
    return reverse_iterator{this->m_components.rend()};
  }

  /**
   * @brief Const reverse iterator to first element - 1
   *
   * Returns a constant reverse iterator to the element following the last
   * dimension of the reversed vector. It corresponds to the element preceding
   * the first dimension of the original vector.
   *
   * This iterator is a placeholder and attempting to access it will result in
   * undefined behavior.
   *
   * @returns A constant reverse iterator to the element following the last
   * dimension.
   */
  const_reverse_iterator rend() const noexcept {
    return const_reverse_iterator{this->m_components.rend()};
  }

protected:
  std::array<T, D> m_components; //!< An array of components for the vector.

#ifdef SVECTOR_EXPERIMENTAL_COMPARE
private:
  /**
   * @brief Compares elements between vectors lexographically (EXPERIMENTAL).
   *
   * Loops through components one by one from left to right, and at any
   * component, if the component of this vector is less than the component of
   * the other vector, then returns -1. If the component of this vector is
   * greater than the component of the other vector, then returns 1. If all
   * components are equal, the returns 0.
   *
   * If this vector has fewer components, then returns -1, and if other vector
   * has fewer components, returns 1.
   *
   * @tparam D2 The number of dimensions of the other vector.
   * @tparam T2 Vector type of the other vector.
   *
   * @param other The other vector to compare to.
   *
   * @returns -1 if the current vector compares less, 0 if the current vector
   * compares equal, and 1 if the current vector compares greater.
   */
  template <std::size_t D2, typename T2>
  std::int8_t compare(const Vector<D2, T2> &other) const noexcept {
    std::size_t min_dim = std::min(D, D2);
    std::size_t counter = 0;

// to suppress MSVC warning
#ifdef _MSC_VER
    if constexpr (D != D2) {
#else
    if (D != D2) {
#endif
      // check dimensions first
      return D < D2 ? -1 : 1;
    }

    // compare one by one
    for (std::size_t i = 0; i < min_dim; i++) {
      if (this->m_components[i] == other[i]) {
        counter++;
      } else if (this->m_components[i] < other[i]) {
        return -1;
      } else {
        return 1;
      }
    }

    if (counter != D || counter != D2) {
      return -1;
    }

    // means two vectors are equal
    return 0;
  }
#endif
};

typedef Vector<2> Vec2_; //!< An alias to Vector<2>.

/**
 * @brief A simple 2D vector representation.
 */
class Vector2D : public Vec2_ {
public:
  using Vec2_::Vector;

  /**
   * @brief Initializes a vector given xy components.
   *
   * @param x The x-component.
   * @param y The y-component.
   */
  Vector2D(const double x, const double y) {
    this->m_components[0] = x;
    this->m_components[1] = y;
  }

  /**
   * @brief Copy constructor for base class.
   */
  Vector2D(const Vec2_ &other) {
    this->m_components[0] = other[0];
    this->m_components[1] = other[1];
  }

  /**
   * @brief Gets x-component
   *
   * Gets the x-component of the vector.
   *
   * @returns x-component of vector.
   */
  double x() const { return this->m_components[0]; }

  /**
   * @brief Sets x-component
   *
   * Sets the x-component of the vector.
   *
   * @param newX x-value to set
   */
  void x(const double &newX) { this->m_components[0] = newX; }

  /**
   * @brief Gets y-component
   *
   * Gets the y-component of the vector.
   *
   * @returns y-component of vector.
   */
  double y() const { return this->m_components[1]; }

  /**
   * @brief Sets y-component
   *
   * Sets the y-component of the vector.
   *
   * @param newY y-value to set
   */
  void y(const double &newY) { this->m_components[1] = newY; }

  /**
   * @brief Angle of vector
   *
   * Gets the angle of the vector in radians.
   *
   * The angle will be in the range (-π, π].
   *
   * @returns The angle of the vector.
   */
  double angle() const { return std::atan2(this->y(), this->x()); }

  /**
   * @brief Rotates vector by a certain angle.
   *
   * The angle should be given in radians. The vector rotates
   * counterclockwise when the angle is positive and clockwise
   * when the angle is negative.
   *
   * @param ang the angle to rotate the vector, in radians.
   *
   * @returns A new, rotated vector.
   */
  Vector2D rotate(const double ang) const {
    //
    // Rotation matrix:
    //
    // | cos(ang)   -sin(ang) | |x|
    // | sin(ang)    cos(ang) | |y|
    //

    const double xPrime = this->x() * std::cos(ang) - this->y() * std::sin(ang);
    const double yPrime = this->x() * std::sin(ang) + this->y() * std::cos(ang);

    return Vector2D{xPrime, yPrime};
  }

  /**
   * @brief Converts vector to another object
   *
   * Converts the components of the vector to an object with a constructor
   * that has two parameters.
   *
   * For example, this method can be used to convert the components of
   * a 2D vector into a `pair<double, double>`, or a struct with two
   * variables and a constructor for those two variables.
   *
   * @returns The converted value.
   */
  template <typename T> T componentsAs() const {
    return T{this->x(), this->y()};
  }
};

typedef Vector<3> Vec3_; //!< An alias to Vector<3>.

/**
 * @brief A simple 3D vector representation.
 */
class Vector3D : public Vec3_ {
public:
  using Vec3_::Vector;

  /**
   * @brief Initializes a vector given xyz components.
   *
   * @param x The x-component.
   * @param y The y-component.
   * @param z The z-component.
   */
  Vector3D(const double x, const double y, const double z) {
    this->m_components[0] = x;
    this->m_components[1] = y;
    this->m_components[2] = z;
  }

  /**
   * @brief Copy constructor for the base class.
   */
  Vector3D(const Vec3_ &other) {
    this->m_components[0] = other[0];
    this->m_components[1] = other[1];
    this->m_components[2] = other[2];
  }

  /**
   * @brief Gets x-component
   *
   * Gets the x-component of the vector.
   *
   * @returns x-component of vector.
   */
  double x() const { return this->m_components[0]; }

  /**
   * @brief Sets x-component
   *
   * Sets the x-component of the vector.
   *
   * @param newX x-value to set
   */
  void x(const double &newX) { this->m_components[0] = newX; }

  /**
   * @brief Gets y-component
   *
   * Gets the y-component of the vector.
   *
   * @returns y-component of vector.
   */
  double y() const { return this->m_components[1]; }

  /**
   * @brief Sets y-component
   *
   * Sets the y-component of the vector.
   *
   * @param newY y-value to set
   */
  void y(const double &newY) { this->m_components[1] = newY; }

  /**
   * @brief Gets z-component
   *
   * Gets the z-component of the vector.
   *
   * @returns z-component of vector.
   */
  double z() const { return this->m_components[2]; }

  /**
   * @brief Sets z-component
   *
   * Sets the z-component of the vector.
   *
   * @param newZ z-value to set
   */
  void z(const double &newZ) { this->m_components[2] = newZ; }

  /**
   * @brief Cross product of two vectors.
   *
   * @param other The other vector to cross current vector with.
   *
   * @returns The cross product of the two vectors.
   */
  Vector3D cross(const Vector3D &other) const {
    const double newx = this->y() * other.z() - this->z() * other.y();
    const double newy = this->z() * other.x() - this->x() * other.z();
    const double newz = this->x() * other.y() - this->y() * other.x();

    return Vector3D{newx, newy, newz};
  }

  /**
   * @brief Converts vector to another object
   *
   * Converts components of vector to an object with a constructor
   * that has three parameters.
   *
   * For example, this method can be used to convert the components of
   * a 3D vector into a struct with three
   * variables and a constructor for those three variables.
   *
   * @returns The converted value.
   */
  template <typename T> T componentsAs() const {
    return T{this->x(), this->y(), this->z()};
  }

  /**
   * @brief Converts angles to another object
   *
   * Converts the angles of vector to an object with a constructor that
   * has three parameters.
   *
   * For example, this method can be used to convert the angles
   * of a 3D vector into a struct with three variables and a
   * constructor for those three variables.
   *
   * @returns Converted value.
   */
  template <typename T> T anglesAs() const {
    return T{this->getAlpha(), this->getBeta(), this->getGamma()};
  }

  /**
   * @brief Gets a specific angle of the vector.
   *
   * Each angle is in the range [0, π]. Angle will be in radians.
   *
   * Specify whether you want the angle from the positive x-axis, the
   * positive y-axis, or the positive z-axis in the template argument.
   *
   * @see svector::AngleDir
   *
   * @note This method will result in undefined behavior if the vector is a zero
   * vector (if the magnitude equals zero).
   *
   * @returns An angle representing the angle you specified.
   */
  template <AngleDir D> double angle() const {
    switch (D) {
    case ALPHA:
      return this->getAlpha();
    case BETA:
      return this->getBeta();
    default:
      return this->getGamma();
    }
  }

  /**
   * @brief Rotates vector around a certain axis by a certain angle.
   *
   * Uses the basic gimbal-like 3D rotation matrices for the
   * x-axis, y-axis, and the z-axis.
   *
   * Specify your rotation in the template argument. When the given template is
   * ALPHA, the vector rotates around the x-axis, when the given template is
   * BETA, the vector rotates around y-axis, and when the given template is
   * GAMMA, the vector rotates around z-axis.
   *
   * @see svector::AngleDir
   *
   * @param ang the angle to rotate the vector, in radians.
   *
   * @returns A new, rotated vector.
   */
  template <AngleDir D> Vector3D rotate(const double &ang) const {
    switch (D) {
    case ALPHA:
      return this->rotateAlpha(ang);
    case BETA:
      return this->rotateBeta(ang);
    default:
      return this->rotateGamma(ang);
    }
  }

private:
  /**
   * Gets α angle.
   *
   * α is the angle between the vector and the x-axis.
   *
   * @returns α
   */
  double getAlpha() const { return std::acos(this->x() / this->magn()); }

  /**
   * Gets β angle.
   *
   * β is the angle between the vector and the y-axis.
   *
   * @returns β
   */
  double getBeta() const { return std::acos(this->y() / this->magn()); }

  /**
   * Gets γ angle.
   *
   * γ is the angle between the vector and the z-axis.
   *
   * @returns γ
   */
  double getGamma() const { return std::acos(this->z() / this->magn()); }

  /**
   * Rotates around x-axis.
   */
  Vector3D rotateAlpha(const double &ang) const {
    /**
     * Rotation matrix:
     *
     * |1   0           0     | |x|
     * |0  cos(ang)  −sin(ang)| |y|
     * |0  sin(ang)   cos(ang)| |z|
     */

    const double xPrime = this->x();
    const double yPrime = this->y() * std::cos(ang) - this->z() * std::sin(ang);
    const double zPrime = this->y() * std::sin(ang) + this->z() * std::cos(ang);

    return Vector3D{xPrime, yPrime, zPrime};
  }

  /**
   * Rotates around y-axis.
   */
  Vector3D rotateBeta(const double &ang) const {
    /**
     * Rotation matrix:
     *
     * | cos(ang)  0  sin(ang)| |x|
     * |   0       1      0   | |y|
     * |−sin(ang)  0  cos(ang)| |z|
     */

    const double xPrime = this->x() * std::cos(ang) + this->z() * std::sin(ang);
    const double yPrime = this->y();
    const double zPrime =
        -this->x() * std::sin(ang) + this->z() * std::cos(ang);

    return Vector3D{xPrime, yPrime, zPrime};
  }

  /**
   * Rotates around z-axis.
   */
  Vector3D rotateGamma(const double &ang) const {
    /**
     * Rotation matrix:
     *
     * |cos(ang)  −sin(ang)  0| |x|
     * |sin(ang)  cos(ang)   0| |y|
     * |  0         0        1| |z|
     */

    const double xPrime = this->x() * std::cos(ang) - this->y() * std::sin(ang);
    const double yPrime = this->x() * std::sin(ang) + this->y() * std::cos(ang);
    const double zPrime = this->z();

    return Vector3D{xPrime, yPrime, zPrime};
  }
};

/**
 * @brief Creates a vector from an std::array.
 *
 * @tparam D The number of dimensions.
 * @tparam T Vector type.
 * @param array An array.
 *
 * @returns A vector whose dimensions reflect the elements in the array.
 */
template <std::size_t D, typename T>
Vector<D, T> makeVector(std::array<T, D> array) {
  Vector<D, T> vec;
  for (std::size_t i = 0; i < D; i++) {
    vec[i] = array[i];
  }

  return vec;
}

/**
 * @brief Creates a vector from a std::vetor.
 *
 * If the given std::vector has fewer elements than the specified dimensions,
 * then this function will fill up the first elements of the vector with those
 * in the given std::vector. The rest of the elements would be 0.
 *
 * If the given std::vector has more elements than the specified dimensions,
 * then the resulting vector would ignore the numbers in those dimensions.
 *
 * @tparam D The number of dimensions.
 * @tparam T Vector type.
 * @param vector A std::vector.
 *
 * @returns A vector whose dimensions reflect the elements in the std::vector.
 */
template <std::size_t D, typename T>
Vector<D, T> makeVector(std::vector<T> vector) {
  Vector<D, T> vec;
  for (std::size_t i = 0; i < std::min(D, vector.size()); i++) {
    vec[i] = vector[i];
  }

  return vec;
}

/**
 * @brief Creates a vector from an initializer list.
 *
 * The initializer list should represent the components of the vector in each
 * dimension. If the size of the initializer list is greater than the number
 * of dimensions given, then the vector only initializes with the first D
 * elements in the list, where D is the number of dimensions. If the size of
 * the initializer list is less than the number of dimensions given, then the
 * vector fills the rest of the dimensions with the value 0.
 *
 * @tparam D The number of dimensions.
 * @tparam T Vector type.
 * @param args the initializer list.
 *
 * @returns A vector whose dimensions reflect the elements in the initializer
 * list.
 */
template <std::size_t D, typename T>
Vector<D, T> makeVector(const std::initializer_list<T> args) {
  Vector<D, T> vec(args);
  return vec;
}

/**
 * @brief Gets the x-component of a 2D vector.
 *
 * @param v A 2D Vector.
 *
 * @returns x-component of the vector.
 */
inline double x(const Vector2D &v) { return v[0]; }

/**
 * @brief Sets the x-component of a 2D vector.
 *
 * @param v A 2D Vector.
 * @param xValue The x-value to set to the vector.
 */
inline void x(Vector2D &v, const double xValue) { v[0] = xValue; }

/**
 * @brief Gets the x-component of a 3D vector.
 *
 * @param v A 3D Vector.
 *
 * @returns x-component of the vector.
 */
inline double x(const Vector3D &v) { return v[0]; }

/**
 * @brief Sets the x-component of a 3D vector.
 *
 * @param v A 3D Vector.
 * @param xValue The x-value to set to the vector.
 */
inline void x(Vector3D &v, const double xValue) { v[0] = xValue; }

/**
 * @brief Gets the y-component of a 2D vector.
 *
 * @param v A 2D Vector.
 *
 * @returns y-component of the vector.
 */
inline double y(const Vector2D &v) { return v[1]; }

/**
 * @brief Sets the y-component of a 2D vector.
 *
 * @param v A 2D Vector.
 * @param yValue The y-value to set to the vector.
 */
inline void y(Vector2D &v, const double yValue) { v[1] = yValue; }

/**
 * @brief Gets the y-component of a 3D vector.
 *
 * @param v A 3D Vector.
 *
 * @returns y-component of the vector.
 */
inline double y(const Vector3D &v) { return v[1]; }

/**
 * @brief Sets the y-component of a 3D vector.
 *
 * @param v A 3D Vector.
 * @param yValue The y value to set to the vector.
 */
inline void y(Vector3D &v, const double yValue) { v[1] = yValue; }

/**
 * @brief Gets the z-component of a 3D vector.
 *
 * @param v A 3D Vector.
 *
 * @returns z-component of the vector.
 */
inline double z(const Vector3D &v) { return v[2]; }

/**
 * @brief Sets the z-component of a 3D vector.
 *
 * @param v A 3D Vector.
 * @param zValue The z value to set to the vector.
 */
inline void z(Vector3D &v, const double zValue) { v[2] = zValue; }

/**
 * @brief Calculates the dot product of two vectors.
 *
 * @note The dimensions of the two vectors must be the same.
 *
 * @tparam D The number of dimensions.
 * @tparam T Vector type.
 *
 * @param lhs First vector.
 * @param rhs Second vector.
 *
 * @returns The dot product of lhs and rhs.
 */
template <typename T, std::size_t D>
inline T dot(const Vector<D, T> &lhs, const Vector<D, T> &rhs) {
  T result = 0;

  for (std::size_t i = 0; i < D; i++) {
    result += lhs[i] * rhs[i];
  }

  return result;
}

/**
 * @brief Gets the magnitude of the vector.
 *
 * @tparam D The number of dimensions.
 * @tparam T Vector type.
 *
 * @param v The vector to get magnitude of.
 *
 * @returns magnitude of vector.
 */
template <typename T, std::size_t D> inline T magn(const Vector<D, T> &v) {
  T sum_of_squares = 0;

  for (std::size_t i = 0; i < D; i++) {
    sum_of_squares += v[i] * v[i];
  }

  return std::sqrt(sum_of_squares);
}

/**
 * @brief Normalizes a vector.
 *
 * Finds the unit vector with the same direction angle as the current vector.
 *
 * @note This method will result in undefined behavior if the vector is a zero
 * vector (if the magnitude equals zero).
 *
 * @tparam D The number of dimensions.
 * @tparam T Vector type.
 *
 * @param v The vector to normalize.
 *
 * @returns Normalized vector.
 */
template <typename T, std::size_t D>
inline Vector<D, T> normalize(const Vector<D, T> &v) {
  return v / magn(v);
}

/**
 * @brief Determines whether a vector is a zero vector.
 *
 * @tparam D The number of dimensions.
 * @tparam T Vector type.
 *
 * @returns Whether the given vector is a zero vector.
 */
template <typename T, std::size_t D> inline bool isZero(const Vector<D, T> &v) {
  return magn(v) == 0;
}

/**
 * @brief Gets the angle of a 2D vector in radians.
 *
 * The angle will be in the range (-π, π].
 *
 * @param v A 2D vector.
 *
 * @returns angle of the vector.
 */
inline double angle(const Vector2D &v) { return std::atan2(y(v), x(v)); }

/**
 * @brief Rotates a 2D vector by a certain angle.
 *
 * The angle should be given in radians. The vector rotates
 * counterclockwise when the angle is positive and clockwise
 * when the angle is negative.
 *
 * @param v A 2D vector.
 * @param ang the angle to rotate the vector, in radians.
 *
 * @returns a new, rotated vector.
 */
inline Vector2D rotate(const Vector2D &v, const double ang) {
  //
  // Rotation matrix:
  //
  // | cos(ang)   -sin(ang) | |x|
  // | sin(ang)    cos(ang) | |y|
  //

  const double xPrime = x(v) * std::cos(ang) - y(v) * std::sin(ang);
  const double yPrime = x(v) * std::sin(ang) + y(v) * std::cos(ang);

  return Vector2D{xPrime, yPrime};
}

/**
 * @brief Cross product of two vectors.
 *
 * @param lhs The first vector.
 * @param rhs The second vector, crossed with the first vector.
 *
 * @returns The cross product of the two vectors.
 */
inline Vector3D cross(const Vector3D &lhs, const Vector3D &rhs) {
  const double newx = y(lhs) * z(rhs) - z(lhs) * y(rhs);
  const double newy = z(lhs) * x(rhs) - x(lhs) * z(rhs);
  const double newz = x(lhs) * y(rhs) - y(lhs) * x(rhs);

  return Vector3D{newx, newy, newz};
}

/**
 * @brief Gets α angle.
 *
 * α is the angle between the vector and the x-axis.
 *
 * @note This method will result in undefined behavior if the vector is a zero
 * vector (if the magnitude equals zero).
 *
 * @param v A 3D vector.
 *
 * @returns α
 */
inline double alpha(const Vector3D &v) { return std::acos(x(v) / magn(v)); }

/**
 * @brief Gets β angle.
 *
 * β is the angle between the vector and the y-axis.
 *
 * @note This method will result in undefined behavior if the vector is a zero
 * vector (if the magnitude equals zero).
 *
 * @param v A 3D vector.
 *
 * @returns β
 */
inline double beta(const Vector3D &v) { return std::acos(y(v) / magn(v)); }

/**
 * @brief Gets γ angle.
 *
 * γ is the angle between the vector and the z-axis.
 *
 * @note This method will result in undefined behavior if the vector is a zero
 * vector (if the magnitude equals zero).
 *
 * @param v A 3D vector.
 *
 * @returns γ
 */
inline double gamma(const Vector3D &v) { return std::acos(z(v) / magn(v)); }

/**
 * @brief Rotates around x-axis.
 *
 * Uses the basic gimbal-like 3D rotation matrices for rotation.
 *
 * @param v A 3D vector.
 * @param ang The angle to rotate the vector, in radians.
 *
 * @returns A new, rotated vector.
 */
inline Vector3D rotateAlpha(const Vector3D &v, const double &ang) {
  //
  // Rotation matrix:
  //
  // |1   0           0     | |x|
  // |0  cos(ang)  −sin(ang)| |y|
  // |0  sin(ang)   cos(ang)| |z|
  //

  const double xPrime = x(v);
  const double yPrime = y(v) * std::cos(ang) - z(v) * std::sin(ang);
  const double zPrime = y(v) * std::sin(ang) + z(v) * std::cos(ang);

  return Vector3D{xPrime, yPrime, zPrime};
}

/**
 * @brief Rotates around y-axis.
 *
 * Uses the basic gimbal-like 3D rotation matrices for rotation.
 *
 * @param v A 3D vector.
 * @param ang The angle to rotate the vector, in radians.
 *
 * @returns A new, rotated vector.
 */
inline Vector3D rotateBeta(const Vector3D &v, const double &ang) {
  //
  // Rotation matrix:
  //
  // | cos(ang)  0  sin(ang)| |x|
  // |   0       1      0   | |y|
  // |−sin(ang)  0  cos(ang)| |z|
  //

  const double xPrime = x(v) * std::cos(ang) + z(v) * std::sin(ang);
  const double yPrime = y(v);
  const double zPrime = -x(v) * std::sin(ang) + z(v) * std::cos(ang);

  return Vector3D{xPrime, yPrime, zPrime};
}

/**
 * @brief Rotates around z-axis.
 *
 * Uses the basic gimbal-like 3D rotation matrices for rotation.
 *
 * @param v A 3D vector.
 * @param ang The angle to rotate the vector, in radians.
 *
 * @returns A new, rotated vector.
 */
inline Vector3D rotateGamma(const Vector3D &v, const double &ang) {
  //
  // Rotation matrix:
  //
  // |cos(ang)  −sin(ang)  0| |x|
  // |sin(ang)  cos(ang)   0| |y|
  // |  0         0        1| |z|
  //

  const double xPrime = x(v) * std::cos(ang) - y(v) * std::sin(ang);
  const double yPrime = x(v) * std::sin(ang) + y(v) * std::cos(ang);
  const double zPrime = z(v);

  return Vector3D{xPrime, yPrime, zPrime};
}

#ifndef SVECTOR_USE_CLASS_OPERATORS
/**
 * @brief Vector addition
 *
 * Performs vector addition and returns a new vector representing the sum of
 * the two vectors.
 *
 * @note This method is only used if SVECTOR_USE_CLASS_OPERATORS is not
 * defined. Otherwise, the operators in svector::Vector are used.
 *
 * @note The dimensions of the two vectors must be the same.
 *
 * @tparam D The number of dimensions.
 * @tparam T Vector type.
 *
 * @param lhs The first vector.
 * @param rhs The second vector.
 *
 * @returns A new vector representing the vector sum.
 */
template <typename T, std::size_t D>
inline Vector<D, T> operator+(const Vector<D, T> &lhs,
                              const Vector<D, T> &rhs) {
  Vector<D, T> tmp;
  for (std::size_t i = 0; i < D; i++) {
    tmp[i] = lhs[i] + rhs[i];
  }

  return tmp;
}

/**
 * @brief Vector subtraction
 *
 * Performs vector subtraction and returns a new vector representing the
 * difference of the two vectors.
 *
 * @note This method is only used if SVECTOR_USE_CLASS_OPERATORS is not
 * defined. Otherwise, the operators in svector::Vector are used.
 *
 * @note The dimensions of the two vectors must be the same.
 *
 * @tparam D The number of dimensions.
 * @tparam T Vector type.
 *
 * @param lhs The first vector.
 * @param rhs The second vector.
 *
 * @returns A new vector representing the vector sum.
 */
template <typename T, std::size_t D>
inline Vector<D, T> operator-(const Vector<D, T> &lhs,
                              const Vector<D, T> &rhs) {
  Vector<D, T> tmp;
  for (std::size_t i = 0; i < D; i++) {
    tmp[i] = lhs[i] - rhs[i];
  }

  return tmp;
}

/**
 * @brief Scalar multiplication
 *
 * Performs scalar multiplication and returns a new vector representing the
 * product.
 *
 * @note This method is only used if SVECTOR_USE_CLASS_OPERATORS is not
 * defined. Otherwise, the operators in svector::Vector are used.
 *
 * @tparam D The number of dimensions.
 * @tparam T Vector type.
 * @tparam T2 Scalar multiplication type.
 *
 * @param lhs The first vector.
 * @param rhs The second vector.
 *
 * @returns A new vector representing the scalar product.
 */
template <typename T, typename T2, std::size_t D>
inline Vector<D, T> operator*(const Vector<D, T> &lhs, const T2 rhs) {
  Vector<D, T> tmp;
  for (std::size_t i = 0; i < D; i++) {
    tmp[i] = lhs[i] * rhs;
  }

  return tmp;
}

/**
 * @brief Scalar division
 *
 * Performs scalar division and returns a new vector representing the
 * quotient.
 *
 * @note This method is only used if SVECTOR_USE_CLASS_OPERATORS is not
 * defined. Otherwise, the operators in svector::Vector are used.
 *
 * @tparam D The number of dimensions.
 * @tparam T Vector type.
 * @tparam T2 Scalar division type.
 *
 * @param lhs The first vector.
 * @param rhs The second vector.
 *
 * @returns A new vector representing the scalar product.
 */
template <typename T, typename T2, std::size_t D>
inline Vector<D, T> operator/(const Vector<D, T> &lhs, const T2 rhs) {
  Vector<D, T> tmp;
  for (std::size_t i = 0; i < D; i++) {
    tmp[i] = lhs[i] / rhs;
  }

  return tmp;
}

/**
 * @brief Compares equality of two vectors.
 *
 * @note This method is only used if SVECTOR_USE_CLASS_OPERATORS is not
 * defined. Otherwise, the operators in svector::Vector are used.
 *
 * @note The dimensions of the two vectors must be the same.
 *
 * @tparam D The number of dimensions.
 * @tparam T Vector type.
 *
 * @param lhs The first vector.
 * @param rhs The second vector.
 *
 * @returns A boolean representing whether the two vectors compare equal.
 */
template <typename T, std::size_t D>
inline bool operator==(const Vector<D, T> &lhs, const Vector<D, T> &rhs) {
  for (std::size_t i = 0; i < D; i++) {
    if (lhs[i] != rhs[i]) {
      return false;
    }
  }

  return true;
}

/**
 * @brief Compares inequality of two vectors.
 *
 * @note This method is only used if SVECTOR_USE_CLASS_OPERATORS is not
 * defined. Otherwise, the operators in svector::Vector are used.
 *
 * @note The dimensions of the two vectors must be the same.
 *
 * @tparam D The number of dimensions.
 * @tparam T Vector type.
 *
 * @param lhs The first vector.
 * @param rhs The second vector.
 *
 * @returns A boolean representing whether the two vectors do not compare equal.
 */
template <typename T, std::size_t D>
inline bool operator!=(const Vector<D, T> &lhs, const Vector<D, T> &rhs) {
  return !(lhs == rhs);
}
#endif

#ifdef SVECTOR_EXPERIMENTAL_COMPARE
template <std::size_t D1, std::size_t D2, typename T1, typename T2>
bool operator<(const Vector<D1, T1> &lhs, const Vector<D2, T2> &rhs) {
  return lhs.compare(rhs) < 0;
}

template <std::size_t D1, std::size_t D2, typename T1, typename T2>
bool operator>(const Vector<D1, T1> &lhs, const Vector<D2, T2> &rhs) {
  return lhs.compare(rhs) > 0;
}

template <std::size_t D1, std::size_t D2, typename T1, typename T2>
bool operator<=(const Vector<D1, T1> &lhs, const Vector<D2, T2> &rhs) {
  return lhs.compare(rhs) <= 0;
}

template <std::size_t D1, std::size_t D2, typename T1, typename T2>
bool operator>=(const Vector<D1, T1> &lhs, const Vector<D2, T2> &rhs) {
  return lhs.compare(rhs) >= 0;
}
#endif
} // namespace svector

#endif

