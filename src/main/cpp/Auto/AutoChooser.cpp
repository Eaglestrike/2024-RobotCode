#include "Auto/AutoChooser.h"

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
}

/**
 * Processes choosers
 * 
 * @param positions Vector of positions, must be "Left", "Mid", "Right", or "Skip"
 * 
 * @note If paths has an element that is not "Left", "Mid", "Right", or "Skip"
 * then will return
*/
void AutoChooser::ProcessChoosers(std::vector<std::string> positions) {
  if (!ValidatePositions(positions)) {
    return;
  } 

  std::string prevPos = positions[0];
  for (unsigned int i = 1; i <= 3; i++) {
    std::string rPos = positions[i]; // ring pos
    if (rPos == AutoConstants::S_NAME) {
      // SetSegment(i, "", "");
      continue;
    }

    std::string path1 = SideHelper::GetPath(prevPos + "Score_to_" + rPos + "Intake.csv"); // get path from prevPos -> rPos
    std::string path2 = SideHelper::GetPath(rPos + "Intake_to_" + rPos + "Score.csv"); // get path from rPos -> sPos
    // setSegment(i, path1, path2);
    prevPos = rPos;
  }

  std::string endPositions = positions[AutoConstants::POS_ARR_SIZE - 1];
  if (endPositions == AutoConstants::S_NAME) {
    return;
  }
  // std::string endPath = // get path from prevPos -> endPos
  // SetSegment(AutoConstants::POS_ARR_SIZE - 1, endPath)
}

void AutoChooser::ShuffleboardInit() {
  if (!m_shuffleboard) {
    return;
  }
}

void AutoChooser::ShuffleboardPeriodic() {
  if (!m_shuffleboard) {
    return;
  }
}

/**
 * Validates an array of positions
 * 
 * Positions must be "Left", "Mid", "Right", "Skip". Size of positions array must be 5
 * 
 * @param positions Array of positions
 * 
 * @returns If array of positions is valid
*/
bool AutoChooser::ValidatePositions(std::vector<std::string> positions) {
  if (positions.size() != AutoConstants::POS_ARR_SIZE) {
    return false;
  }

  for (const auto &str : positions) {
    if (str != AutoConstants::L_NAME && str != AutoConstants::M_NAME && str != AutoConstants::R_NAME && str != AutoConstants::S_NAME) {
      return false;
    }
  }

  return true;
}