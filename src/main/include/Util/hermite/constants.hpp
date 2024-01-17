/**
 * @file
 *
 * Constants and constant expressions
 */

#pragma once

namespace hermite {
/**
 * @brief First Hermite basis function
 */
constexpr double h00(const double t) { return 2 * t * t * t - 3 * t * t + 1; }

/**
 * @brief Second Hermite basis function
 */
constexpr double h10(const double t) { return t * t * t - 2 * t * t + t; }

/**
 * @brief Third Hermite basis function
 */
constexpr double h01(const double t) { return -2 * t * t * t + 3 * t * t; }

/**
 * @brief Fourth Hermite basis function
 */
constexpr double h11(const double t) { return t * t * t - t * t; }

/**
 * @brief First Hermite basis function first derivative
 */
constexpr double h00d(const double t) { return 6 * t * t - 6 * t; }

/**
 * @brief Second Hermite basis function first derivative
 */
constexpr double h10d(const double t) { return 3 * t * t - 4 * t + 1; }

/**
 * @brief Third Hermite basis function first derivative
 */
constexpr double h01d(const double t) { return 6 * t - 6 * t * t; }

/**
 * @brief Fourth Hermite basis function first derivative
 */
constexpr double h11d(const double t) { return 3 * t * t - 2 * t; }

/**
 * @brief First Hermite basis function second derivative
 */
constexpr double h00dd(const double t) { return 12 * t - 6; }

/**
 * @brief Second Hermite basis function second derivative
 */
constexpr double h10dd(const double t) { return 6 * t - 4; }

/**
 * @brief Third Hermite basis function second derivative
 */
constexpr double h01dd(const double t) { return 6 - 12 * t; }

/**
 * @brief Fourth Hermite basis function second derivative
 */
constexpr double h11dd(const double t) { return 6 * t - 2; }
} // namespace hermite
