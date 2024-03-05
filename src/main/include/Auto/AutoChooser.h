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
  bool ProcessChoosers(bool dryRun);

  void ShuffleboardInit();
  void ShuffleboardPeriodic();
private:
  bool m_shuffleboard;
  Auto &m_auto;

  bool m_edited;
  std::vector<std::pair<std::string, std::string>> m_positions;
};