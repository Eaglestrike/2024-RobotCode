/**
 * A file of constants for the drivebase
 */

#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "Util/simplevectors.hpp"
#include "Util/Utils.h"

namespace vec = svector;

namespace SwerveConstants
{
  const double MAG_ENCODER_COUNTS_PER_REV = 4096;
  const double TALON_FX_COUNTS_PER_REV = 2048;
  const double WHEEL_RADIUS = 0.049; // in meters
  const double WHEEL_GEAR_RATIO = 6.12; // stolen from Alex, 6.12 motor spins = 1 wheel spin

  // meters
  const double CENTER_TO_EDGE = 0.288;

  const std::size_t FR_DRIVE_ID = 14; // 17, 14
  const std::size_t BR_DRIVE_ID = 11; // 21, 11
  const std::size_t FL_DRIVE_ID = 21; // 11, 21
  const std::size_t BL_DRIVE_ID = 17; // 14, 17

  const std::size_t FR_TURN_ID = 13; // 18, 13
  const std::size_t BR_TURN_ID = 12; // 15, 12
  const std::size_t FL_TURN_ID = 15; // 12, 15
  const std::size_t BL_TURN_ID = 18; // 13, 18

  const std::size_t FR_ENCODER_ID = 62; // 10, 62
  const std::size_t BR_ENCODER_ID = 42; // 8,  42
  const std::size_t FL_ENCODER_ID = 8; // 42, 8
  const std::size_t BL_ENCODER_ID = 10;  // 62, 10

  // If positive drive motor does not move swerve module forward when angle is 0
  const bool FR_DRIVE_INVERTED = true;
  const bool BR_DRIVE_INVERTED = false;
  const bool FL_DRIVE_INVERTED = false;
  const bool BL_DRIVE_INVERTED = true;

  // if positive encoder != CCW movement
  const bool FR_ENCODER_INVERTED = false;
  const bool BR_ENCODER_INVERTED = false;
  const bool FL_ENCODER_INVERTED = false;
  const bool BL_ENCODER_INVERTED = false;

  // if positive angle motor != CCW movement
  const bool FR_ANG_INVERTED = true;
  const bool BR_ANG_INVERTED = true;
  const bool FL_ANG_INVERTED = true;
  const bool BL_ANG_INVERTED = true;

  // encoder offset degrees, subtracted from reading
  const double FR_OFFSET = 16.2; // -73.97
  const double BR_OFFSET = 6.328; // 96.503, not oscilatimg
  const double FL_OFFSET = 100.57;    // 9.3
  const double BL_OFFSET = 104.45;   // 4.53

  // individual module wheel PID
  const double TURN_P = 6.8;
  const double TURN_I = 0;
  const double TURN_D = 0.075;

  // angle correction when translationally driving
  const double ANG_CORRECT_P = 5;
  const double ANG_CORRECT_I = 0;
  const double ANG_CORRECT_D = 0.5;
  const double ANG_CORRECT_TOL = Utils::DegToRad(3.5);
  const double SPEED_NO_ANG_CORRECT = 0.25;
  const bool ANG_CORRECT_INVERTED = false;

  // whether navx is upside down or not
  const bool NAVX_UPSIDE_DOWN = false;

  const double MAX_VOLTS = 12.0; 
  const double kS = 0.1833;
  const double kV = 1.455;
  const double kA = 0.1410;

  const double NORMAL_SWERVE_MULT = 12.0;
  const double SLOW_SWERVE_MULT = 3.0;

  const double PITCH_OFFSET = -0.02;
  const double ROLL_OFFSET = M_PI-0.02;

  struct SwerveConfig{
    std::string name;
    std::size_t driveMotorId;
    std::size_t angleMotorId;
    std::size_t encoderId;
    bool driveInverted;
    bool encoderInverted;
    bool angMotorInverted;
    double offset;
    vec::Vector2D position;
    double kP = TURN_P;
    double kI = TURN_I;
    double kD = TURN_D;
  };

  const SwerveConfig FR_CONFIG{
     .name = "Front Right",
     .driveMotorId = FR_DRIVE_ID, 
     .angleMotorId = FR_TURN_ID,
     .encoderId = FR_ENCODER_ID,
     .driveInverted = FR_DRIVE_INVERTED,
     .encoderInverted = FR_ENCODER_INVERTED,
     .angMotorInverted = FR_ANG_INVERTED,
     .offset = FR_OFFSET,
     .position = {SwerveConstants::CENTER_TO_EDGE, -SwerveConstants::CENTER_TO_EDGE}
  };

  const SwerveConfig BR_CONFIG{
     .name = "Back Right",
     .driveMotorId = BR_DRIVE_ID, 
     .angleMotorId = BR_TURN_ID,
     .encoderId = BR_ENCODER_ID,
     .driveInverted = BR_DRIVE_INVERTED,
     .encoderInverted = BR_ENCODER_INVERTED,
     .angMotorInverted = BR_ANG_INVERTED,
     .offset = BR_OFFSET,
     .position = {-SwerveConstants::CENTER_TO_EDGE, -SwerveConstants::CENTER_TO_EDGE}
  };

  const SwerveConfig FL_CONFIG{
     .name = "Front Left",
     .driveMotorId = FL_DRIVE_ID, 
     .angleMotorId = FL_TURN_ID,
     .encoderId = FL_ENCODER_ID,
     .driveInverted = FL_DRIVE_INVERTED,
     .encoderInverted = FL_ENCODER_INVERTED,
     .angMotorInverted = FL_ANG_INVERTED,
     .offset = FL_OFFSET,
     .position = {SwerveConstants::CENTER_TO_EDGE, SwerveConstants::CENTER_TO_EDGE}
  };

  const SwerveConfig BL_CONFIG{
     .name = "Back Left",
     .driveMotorId = BL_DRIVE_ID, 
     .angleMotorId = BL_TURN_ID,
     .encoderId = BL_ENCODER_ID,
     .driveInverted = BL_DRIVE_INVERTED,
     .encoderInverted = BL_ENCODER_INVERTED,
     .angMotorInverted = BL_ANG_INVERTED,
     .offset = BL_OFFSET,
     .position = {-SwerveConstants::CENTER_TO_EDGE, SwerveConstants::CENTER_TO_EDGE}
  };
}