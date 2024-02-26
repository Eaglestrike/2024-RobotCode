#pragma once

#include <string>
#include <vector>
#include <utility>

#include "Auto/Auto.h"

/**
 * Auto Chooser
*/
class AutoChooser {
public:
  AutoChooser(bool shuffleboard, Auto &auton);

  void SetPosition(int idx, std::string primary, std::string secondary);
  void ProcessChoosers(bool dryRun);

  void ShuffleboardInit();
  void ShuffleboardPeriodic();
private:
  bool m_shuffleboard;
  Auto &m_auto;

  std::vector<std::pair<std::string, std::string>> m_positions;
};