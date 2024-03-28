#pragma once

#include <set>

#include "Util/simplevectors.hpp"


namespace vec = svector;

namespace FieldConstants {  
  // tags, units are in INCHES
  const vec::Vector2D TAGS[16] {
    {593.68, 9.68},
    {637.21, 34.79},
    {652.73, 196.17},
    {652.73, 218.42},
    {578.77, 323.00},
    {72.5, 323.00},
    {-1.5, 218.42},
    {-1.5, 196.17},
    {14.02, 34.79},
    {57.54, 9.68},
    {468.69, 146.19},
    {468.69, 177.10},
    {441.74, 161.62},
    {209.48, 161.62},
    {182.73, 177.10},
    {182.73, 146.19}
  };

  // units are in INCHES
  const double FIELD_WIDTH = 651.23;
  const double FIELD_HEIGHT = 323.00;
  const double FIELD_MARGIN = 50.0;

  // speaker location
  const vec::Vector2D BLUE_SPEAKER = {-0.1, 5.79};

  const std::set<int> BLUE_TAGS = {7, 8, 14};
  const std::set<int> RED_TAGS = {3, 4, 13};

  // amp location
  const vec::Vector2D BLUE_AMP = {1.815, 7.888};
  const vec::Vector2D RED_AMP = {14.743, 7.888};
}