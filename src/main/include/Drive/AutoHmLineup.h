#pragma once

#include "Util/Odometry.h"
#include "Util/hermite.hpp"
#include "Util/PID.h"
#include "Util/simplevectors.hpp"
#include "ShuffleboardSender/ShuffleboardSender.h"

namespace hm = hermite;
namespace vec = svector;

/**
 *  Auto hermite lineup
*/
class AutoHmLineup {
public:
  enum ExecutePosState {
      P_NOT_EXECUTING,
      P_EXECUTING_TARGET,
      P_AT_TARGET
  };

  enum ExecuteAngState {
      A_NOT_EXECUTING,
      A_EXECUTING_TARGET,
      A_AT_TARGET
  };

  AutoHmLineup(bool shuffleboard, Odometry &odom);

  void Start();
  void Stop();

  vec::Vector2D GetExpPos() const;
  vec::Vector2D GetTargPos() const;
  double GetExpAng() const;
  double GetTargAng() const;

  vec::Vector2D GetExpVel() const;
  double GetExpAngVel() const;
  bool HasStarted() const;

  void SetTarget(const vec::Vector2D &tgtPos, const double &tgtAng);

  void Periodic();
  void Periodic(double angVel);

  vec::Vector2D GetPos() const;
  bool IsDoneHermite() const;
  bool AtPosTarget() const;
  bool AtAngTarget() const;
  bool AtTarget() const;

  void ShuffleboardInit();
  void ShuffleboardPeriodic();

private:
  ShuffleboardSender m_shuff;
  Odometry &m_odom;
  hm::Hermite<2> m_posSpline;
  hm::Hermite<1> m_angSpline;

  vec::Vector2D m_targPos;
  double m_targAng;

  bool m_hasStarted;

  PID m_posCorrectX;
  PID m_posCorrectY;
  PID m_angCorrect;

  double m_startTime;
  double m_kTime;

  vec::Vector2D m_curExpVel;
  double m_curExpAngVel;

  bool m_isDonePos;
  bool m_isDoneAng;
};