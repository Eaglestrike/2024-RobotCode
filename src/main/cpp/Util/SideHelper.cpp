#include "Util/SideHelper.h"

#include <cmath>
#include <regex>

#include <frc/DriverStation.h>

#include "Constants/AutoConstants.h"
#include "Constants/AutoLineupConstants.h"
#include "Constants/FieldConstants.h"
#include "Util/Utils.h"

/**
 * Determines whether if blue
 * 
 * @returns if blue
*/
bool SideHelper::IsBlue() {
  auto alliance = frc::DriverStation::GetAlliance();

  if (alliance) {
    if (alliance.value() == frc::DriverStation::Alliance::kBlue) {
      return true;
    } else {
      return false;
    }
  } 

  return true;
}

/**
 * Gets position based on side. All position constants are on blue side.
 * Use this to get positions on red side by mirroring.
 * 
 * @param bluePos position on blue, in m
 * 
 * @return Position in m on red, if we are on red, otherwise position on blue
*/
vec::Vector2D SideHelper::GetPos(vec::Vector2D bluePos) {
  if (IsBlue()) {
    return bluePos;
  }
  
  double fieldWidthM = Utils::InToM(FieldConstants::FIELD_WIDTH);
  double redX = fieldWidthM - bluePos.x();
  return {redX, bluePos.y()};
}

/**
 * Gets velocity based on side. Velocity waypoints aare on blue side.
 * Use this to get velocity on red side by mirroring
 * 
 * @param blueVel veloicty of blue, in m/s
 * 
 * @returns Velocity in m/s on red, if we are on red, otherwise velocity on blue
*/
vec::Vector2D SideHelper::GetVel(vec::Vector2D blueVel) {
  if (IsBlue()) {
    return blueVel;
  }

  return {-blueVel.x(), blueVel.y()};
}

/**
 * Gets angle based on side. Angle constants are on blue side.
 * Use this to get angles on red side by mirroring.
 * 
 * @param blueAng Blue angle, in rad
 * 
 * @returns Angle in rad on red, if we are on red, otherwise angle on blue
*/
double SideHelper::GetAng(double blueAng) {
  if (IsBlue()) {
    return blueAng;
  } 

  return M_PI - blueAng;
}

/**
 * Gets ang vel based on side. Ang Velocity waypoints aare on blue side.
 * Use this to get ang velocities on red side by mirroring
 * 
 * @param blueAngVel ang veloicty of blue, in rad/s
 * 
 * @returns Ang Velocity in rad/s on red, if we are on red, otherwise ang velocity on blue
*/
double SideHelper::GetAngVel(double blueAngVel) {
  if (IsBlue()) {
    return blueAngVel;
  }

  return -blueAngVel;
}

/**
 * Gets starting positions
 * 
 * @param idx index if out of bounds, chooses 1
 * 
 * @note On blue, [0, 1, 2] are [L, M, R] respectively, but on red, [0, 1, 2] are [R, M, L] respectively
 * 
 * @returns Start pose
*/
AutoConstants::StartPose SideHelper::GetStartingPose(int idx) {
  if (idx < 0 || idx > 2) {
    idx = 1;
  }

  switch (idx) {
    case 0:
      return {SideHelper::GetPos(AutoConstants::BLUE_L.pos), SideHelper::GetAng(AutoConstants::BLUE_L.ang)};
    case 1:
      return {SideHelper::GetPos(AutoConstants::BLUE_M.pos), SideHelper::GetAng(AutoConstants::BLUE_M.ang)};
    case 2:
      return {SideHelper::GetPos(AutoConstants::BLUE_R.pos), SideHelper::GetAng(AutoConstants::BLUE_R.ang)};
  } 

  return {SideHelper::GetPos(AutoConstants::BLUE_M.pos), SideHelper::GetAng(AutoConstants::BLUE_M.ang)};
}

/**
 * Gets starting positions from string
 * 
 * @param pos Starting position, either "Left", "Middle", "Right," converts to correct starting position no matter which side
 * 
 * @returns Start pose
*/
AutoConstants::StartPose SideHelper::GetStartingPose(std::string pos) {
  int idx = 1;

  if (pos == AutoConstants::L_START) {
    idx = 0;
  } else if (pos == AutoConstants::R_START) {
    idx = 2;
  }

  if (!IsBlue()) {
    idx = 2 - idx;
  }

  return GetStartingPose(idx);
}

/**
 * Gets joystick ang based on side
 * 
 * @returns Joystick ang, in rad
*/
double SideHelper::GetJoystickAng() {
  if (IsBlue()) {
    return 0;
  }

  return M_PI;
}

/**
 * Gets position spline, and mirror it to red if we are red
 * 
 * @param inp Input spline
 * 
 * @returns res position spline
*/
hm::Hermite<2> SideHelper::GetSplinePos(hm::Hermite<2> inp) {
  hm::Hermite<2> res;

  for (auto waypoint : inp.getAllWaypoints()) {
    double time = waypoint.getTime();
    vec::Vector2D pos = SideHelper::GetPos((vec::Vector2D) waypoint.getPos());
    vec::Vector2D vel = SideHelper::GetVel((vec::Vector2D) waypoint.getVel());

    res.insert({time, pos, vel});
  }

  return res;
}

/**
 * Gets path name depending on side
 * 
 * @param path Path
 * 
 * @returns Path name
*/
std::string SideHelper::GetPath(std::string path) {
  if (IsBlue()) {
    return path;
  }

  path = std::regex_replace(path, std::regex{"Left"}, "^^^UNKNOWN&&&");
  path = std::regex_replace(path, std::regex{"Right"}, "Left");
  path = std::regex_replace(path, std::regex{"\\^\\^\\^UNKNOWN&&&"}, "Right");

  return path;
}

/**
 * Gets ang spline, and mirror it to red if we are red
 * 
 * @param inp Input spline
 * 
 * @returns res ang spline
*/
hm::Hermite<1> SideHelper::GetSplineAng(hm::Hermite<1> inp) {
  hm::Hermite<1> res;

  for (auto waypoint : inp.getAllWaypoints()) {
    double time = waypoint.getTime();
    double pos = SideHelper::GetAng(waypoint.getPos()[0]);
    double vel = SideHelper::GetAngVel(waypoint.getVel()[0]);

    res.insert({time, {pos}, {vel}});
  }

  return res;
}

/**
 * Gets manual lineup angle
 * 
 * @param idx Index of lineup
 * 
 * @returns Manual lineup angle
*/
double SideHelper::GetManualLineupAng(int idx) {
  if (idx < 0 || idx >= static_cast<int>(AutoLineupConstants::BLUE_SHOOT_LOCATIONS.size())) {
    return IsBlue() ? M_PI : 0;
  }
  
  if (!IsBlue()) {
    idx = static_cast<int>(AutoLineupConstants::BLUE_SHOOT_LOCATIONS.size()) - idx - 1;
  }

  vec::Vector2D shootPos = GetPos(AutoLineupConstants::BLUE_SHOOT_LOCATIONS[idx]);
  vec::Vector2D speakerPos = GetPos(FieldConstants::BLUE_SPEAKER);

  vec::Vector2D shootToSpeaker = speakerPos - shootPos;
  
  return shootToSpeaker.angle();
}