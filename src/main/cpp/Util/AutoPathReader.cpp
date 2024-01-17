#include "Util/AutoPathReader.h"

#include <fstream>
#include <sstream>
#include <vector>

/**
 * Constructor
 * 
 * @param name Name of file
 * 
 * @note If file doesn't exist, GetSpline() returns empty spline object
*/
AutoPathReader::AutoPathReader(const std::string name)
: m_name{name}, m_splinePos{1000}, m_splineAng{1000} {
  std::ifstream fin{name};
  if (!fin.is_open()) {
    return;
  }

  std::string line;
  bool notfirst = false;
  while (std::getline(fin, line)) {
    if (!notfirst) {
      notfirst = true;
      continue;
    }
    ParseLine(line);
  } 
}

/**
 * Gets spline for translational motion
 * 
 * @return Spline for translational motion
*/
hm::Hermite<2> AutoPathReader::GetSplinePos() const {
  return m_splinePos;
}

/**
 * Gets spline for angular motion
 * 
 * @returns Spline for angular motion
*/
hm::Hermite<1> AutoPathReader::GetSplineAng() const {
  return m_splineAng;
}

/**
 * Parses line
 * 
 * @param line Line to parse
*/
void AutoPathReader::ParseLine(const std::string line) {
  std::stringstream lineStream(line);
  std::string cell;
  std::vector<double> result;

  while (std::getline(lineStream, cell, ',')) {
    result.push_back(std::stod(cell));
  }
  if (!lineStream && cell.empty()) {
    result.push_back(0.0);
  }

  if (result.size() != 7) {
    return;
  }

  const double x = result[0];
  const double y = result[1];
  const double vx = result[2];
  const double vy = result[3];
  const double ang = result[4];
  const double angVel = result[5];
  const double t = result[6];

  m_splinePos.insert(hm::Pose<2>{t, {x, y}, {vx, vy}});
  m_splineAng.insert(hm::Pose<1>{t, {ang}, {angVel}});
}
