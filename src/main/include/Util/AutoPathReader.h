#pragma once

#include <string>

#include "Util/hermite.hpp"

namespace hm = hermite;

/**
 * Reads auto path CSV file and forms Hermite spline object
*/
class AutoPathReader {
public:
  AutoPathReader(const std::string name);

  hm::Hermite<2> GetSplinePos() const; 
  hm::Hermite<1> GetSplineAng() const;
  bool FileExists() const;
private:
  void ParseLine(const std::string line);

  std::string m_name;
  bool m_exists;
  hm::Hermite<2> m_splinePos;
  hm::Hermite<1> m_splineAng;
};