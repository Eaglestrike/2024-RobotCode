#pragma once

#include <atomic>
#include <thread>

#include <frc/DigitalInput.h>

#include "Constants/IntakeConstants.h"

class IOThread {
public:
  IOThread() = default;

  void Init();

  bool GetBeamBreak1();
  bool GetBeamBreak2();
private:
  void Loop();

  std::thread m_th;

  frc::DigitalInput m_bb1{IntakeConstants::BEAM_BREAK1_ID};
  frc::DigitalInput m_bb2{IntakeConstants::BEAM_BREAK2_ID};

  std::atomic<bool> m_beamBreak1{false};
  std::atomic<bool> m_beamBreak2{false};
};
