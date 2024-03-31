/**
 * Constants for odometry
*/

#pragma once

#include "Util/simplevectors.hpp"
#include "Util/Utils.h"

#include <frc/geometry/Transform3d.h>

namespace vec = svector;

namespace OdometryConstants {
  // const double P_INITIAL = 1.0;
  // const double POS_STD_DEV = 0.1;
  // const double MEASURE_STD_DEV = 0.1;
  // const double CAMERA_TRUST_K = -10.0;

  // const struct RugConfig{
  //   vec::Vector2D direction = vec::Vector2D{1, 0}.rotate(1.57); //Where the carpet points
  //   vec::Vector2D perpDirection = direction.rotate(M_PI/2.0);
  //   double shiftDistance = 0.0; //Distance shifted by driving along carpet hairs -> m
  //   // double shiftDistanceK = 0.835; //Distance gained by driving with hair -> extra m/m
  //   double shiftDistanceK = 1; // disable shiftDistanceK
  //   double perpShiftDistance = 0.0; //Distance shifted by driving perpendicularly to the hair -> m
  //   double perpShiftDistanceK = 1.0; //Distance gained by driving perpendicularly with hair -> extra m/m
  // } RUG_CONFIG;

  // const double E0 = 1.0;
  // const double Q = 0.01;
  // const double CAM_TRUST_KANG = 10.0; // unused for now, can use if relying on apriltag angle
  // const double CAM_TRUST_KPOS = 500.0;
  // const double CAM_TRUST_KPOSINT = 100.0;
  // const double MAX_TIME = 0.5;

  // // trust term for wheels
  // const double ALPHA = 0.7;

  // for cam data greater than AT_REJECT meters, we reject
  const double AT_REJECT = 6;

  // drivebase system std dev x, y
  // TODO need to change
  const vec::Vector2D SYS_STD_DEV_AUTO = {0.00044, 0.00044}; // 0.001, 0.001
  const vec::Vector2D SYS_STD_DEV_TELE = {0.00044, 0.00044}; // 0.01

  // camera std dev coefficient (0 stddev if 0 m from camera, increases quadratically with distance)
  const double CAM_STD_DEV_COEF_AUTO = 0.015; // 0.03
  const double CAM_STD_DEV_COEF_TELE = 0.015; // 0.003

  // turning std dev coef
  const double CAM_TURN_STD_DEV_COEF = 0.01909859317;

  // if greater than this distance but less than AT_REJECT, immediately correct
  const double TRUST_CAMS_MORE_THRESH = 1;

  // angular speed above which we use swerve angle, in degrees / sec
  const double USE_SWERVE_ANG_VEL = Utils::DegToRad(5);

  // offset from caams, in s
  const double CAM_TIME_OFFSET = 0.133;

  // camera to robot, in m
  const units::meter_t X_OFFSET = units::meter_t{0.083};
  const units::meter_t Y_OFFSET = units::meter_t{0.12};
  const units::meter_t Z_OFFSET = units::meter_t{0.6604};
  const units::radian_t PITCH_OFFSET = units::radian_t{0.509};
  const units::radian_t ROLL_OFFSET = units::radian_t{0};
  const units::radian_t YAW_OFFSET = units::radian_t{0};
  const frc::Transform3d ROBOT_CAM_TRANSFORM{X_OFFSET, Y_OFFSET, Z_OFFSET, frc::Rotation3d{PITCH_OFFSET, ROLL_OFFSET, YAW_OFFSET}};
}
