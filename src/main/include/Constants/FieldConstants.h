#pragma once

#include <set>
#include <utility>

#include "Util/simplevectors.hpp"


namespace vec = svector;

namespace FieldConstants {  
  typedef std::pair<vec::Vector2D, double> tag_t;

  // tags, units are in INCHES and DEGREES
  const tag_t TAGS[16] {
    {{593.68, 9.68}, 120},
    {{637.21, 34.79}, 120},
    {{652.73, 196.17}, 180},
    {{652.73, 218.42}, 180},
    {{578.77, 323.00}, 270},
    {{72.5, 323.00}, 270},
    {{-1.5, 218.42}, 0},
    {{-1.5, 196.17}, 0},
    {{14.02, 34.79}, 60},
    {{57.54, 9.68}, 60},
    {{468.69, 146.19}, 300},
    {{468.69, 177.10}, 60},
    {{441.74, 161.62}, 180},
    {{209.48, 161.62}, 0},
    {{182.73, 177.10}, 120},
    {{182.73, 146.19}, 240}
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