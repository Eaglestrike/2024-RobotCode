#include "Auto/AutoChooser.h"

#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>

#include "Constants/AutoConstants.h"
#include "Util/SideHelper.h"

/**
 * Constructor
 * 
 * @param shuffleboard Shuffleboard toggle
 * 
 * @param auton Auto object
*/
AutoChooser::AutoChooser(bool shuffleboard, Auto &auton) :
  m_shuffleboard{shuffleboard}, m_auto{auton} {
  m_positions.resize(AutoConstants::POS_ARR_SIZE);
}

/**
 * Sets auto path at a position
 * 
 * @param idx Index to set auto path to, 0 <= idx < AutoConstants::POS_ARR_SIZE
 * @param primary Primary path, "" to skip
 * @param secondary Secondary path, "" to skip
*/
void AutoChooser::SetPosition(int idx, std::string primary, std::string secondary) {
  if (idx < 0 || idx >= m_positions.size()) {
    return;
  }

  m_positions[idx] = {primary, secondary};
}

/**
 * Processes choosers by interpreting positions array and inserting them into auto obj
 * 
 * @param dryRun Does not actually insert paths and prints paths instead, use for debug
*/
void AutoChooser::ProcessChoosers(bool dryRun) {
  if (dryRun) {
    std::cout << std::endl;
  }

  m_auto.Clear();

  std::string prevName = m_positions[0].first;
  for (int i = 1; i < m_positions.size(); i++) {
    std::string primary = m_positions[i].first;
    std::string secondary = m_positions[i].second;

    if (primary == "") {
      continue;
    }

    if (primary == AutoConstants::M_FAR && secondary != "") {
      if (prevName == AutoConstants::R_SCORE || prevName == AutoConstants::R_NEAR || prevName == AutoConstants::R_START) {
        secondary = AutoConstants::R_SCORE;
      } else {
        secondary = AutoConstants::M_SCORE;
      }
    }

    std::string path1 = prevName + "_to_" + primary + ".csv";
    path1 = SideHelper::GetPath(path1);
    if (dryRun) {
      std::cout << path1 << std::endl; 
    } else if (i < m_positions.size() - 1) {
      // insert path 1
      m_auto.SetSegment(i, path1);
    } else {
      m_auto.SetDrive(i, path1);
    }
    prevName = primary;

    if (secondary == "") {
      continue;
    }

    std::string path2 = prevName + "_to_" + secondary + ".csv";
    path2 = SideHelper::GetPath(path2);
    if (dryRun) {
      std::cout << path2 << std::endl; 
    } else {
      // insert path 2
      m_auto.SetSegment(i, path1, path2);
    }
    prevName = secondary;
  }
}

/**
 * Shuffleboard init
*/
void AutoChooser::ShuffleboardInit() {
  if (!m_shuffleboard) {
    return;
  }

  frc::SmartDashboard::PutBoolean("Print Paths", false);
}

/**
 * Shuffleboard periodic
*/
void AutoChooser::ShuffleboardPeriodic() {
  if (!m_shuffleboard) {
    return;
  }

  bool printPath = frc::SmartDashboard::GetBoolean("Print Paths", false);
  if (printPath) {
    ProcessChoosers(true);
    frc::SmartDashboard::PutBoolean("Print Paths", false);
  }
}