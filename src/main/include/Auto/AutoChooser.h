#pragma once

#include <string>
#include <vector>

#include "Auto/Auto.h"

/**
 * Auto Chooser
*/
class AutoChooser {
public:
  AutoChooser(bool shuffleboard, Auto &auton);

  void ProcessChoosers(std::vector<std::string> positions);

  void ShuffleboardInit();
  void ShuffleboardPeriodic();
private:
  bool ValidatePositions(std::vector<std::string> positions);

  bool m_shuffleboard;
  Auto &m_auto;
};