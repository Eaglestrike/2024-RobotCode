// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#define SWERVE_AUTOTUNING false

#include <string>

#include <AHRS.h>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>

#include "ShuffleboardSender/ShuffleboardSender.h"

#include "FFAutotuner/FFAutotuner.h"

#include "Controller/Controller.h"
#include "Drive/SwerveControl.h"
#include "Intake/Wrist.h"
#include "Intake/Channel.h"
#include "Intake/Intake.h"
#include "Intake/Rollers.h"
#include "Util/Logger.h"
#include "Util/Odometry.h"
#include "Util/SocketClient.h"

// FOR DEBUG ONLY
#include "Auto/AutoPathSegment.h"

class Robot : public frc::TimedRobot {
public:
  Robot();
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  void SimulationInit() override;
  void SimulationPeriodic() override;

private: 
  void ShuffleboardInit();
  void ShuffleboardPeriodic();

  // Controller
  Controller m_controller;

  // navX (gyroscope)
  AHRS *m_navx;

  // Swerve
  SwerveControl m_swerveController{true, false};

  //intake
  Wrist m_wrist {true, true};
  Channel m_channel {false, false};
  Rollers m_rollers {false, false};
  Intake m_intake {false, false};

  // Jetson
  #if SWERVE_AUTOTUNING
  FFAutotuner m_swerveXTuner{"Swerve X", FFAutotuner::SIMPLE}; //0.1833, 1.455, 0.1410
  FFAutotuner m_swerveYTuner{"Swerve Y", FFAutotuner::SIMPLE}; //0.1711, 1.384, 0.1398
  #endif
  
  // Vision (Jetson)
  SocketClient m_client;
  bool m_isSecondTag;
  
  // Odometry
  Odometry m_odom;

  // Logger
  FRCLogger m_logger;
  bool m_prevIsLogging;

  // DEBUG ONLY
  AutoPathSegment m_autoPath;
  frc::SendableChooser<std::string> m_chooser;
  // double m_wheelAng = 0;
};
