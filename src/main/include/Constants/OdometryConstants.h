/**
 * Constants for odometry
*/

#pragma once

#include "Util/simplevectors.hpp"

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
  const double AT_REJECT = 600;

  // drivebase system std dev x, y
  // TODO need to change
  const vec::Vector2D SYS_STD_DEV = {0.003, 0.003};

  // camera std dev coefficient (0 stddev if 0 m from camera, increases quadratically with distance)
  // TODO need to change
  const double CAM_STD_DEV_COEF = 1.71e-4;
}
