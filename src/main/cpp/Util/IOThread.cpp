#include "Util/IOThread.h"

/**
 * Initializes thread
 * 
 * @note only call this once!
*/
void IOThread::Init() {
  m_th = std::thread([this]
                     { this->Loop(); });
  m_th.detach();
}

/**
 * Gets first beam break
 * 
 * @returns If first beam break is tripped
*/
bool IOThread::GetBeamBreak1() {
  return m_beamBreak1.load();
}

/**
 * Gets second beam break
 * 
 * @returns If second beam break is tripped
*/
bool IOThread::GetBeamBreak2() {
  return m_beamBreak2.load();
}

/**
 * Loops IO Thread
*/
void IOThread::Loop() {
  while (true) {
    bool bb1 = m_bb1.Get();
    m_beamBreak1.store(bb1);
    bool bb2 = m_bb2.Get();
    m_beamBreak2.store(bb2);
  }
}