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
#include "Drive/AutoAngLineup.h"
#include "Drive/SwerveControl.h"
#include "Intake/Intake.h"
#include "Auto/Auto.h"
#include "Util/Logger.h"
#include "Util/Odometry.h"
#include "Util/SocketClient.h"

#include "Constants/AutoConstants.h"

#include "Shooter/Shooter.h"

class Robot : public frc::TimedRobot {
  const static uint AUTO_LENGTH = 4;
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
    Intake m_intake {true, false};

    //Auto 
    Auto m_auto;
    frc::SendableChooser<std::string> m_autoChoosers[AUTO_LENGTH];

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

    // Shooter
    // Shooter m_shooter{"Shooter", true, true};

    // Logger
    FRCLogger m_logger;
    bool m_prevIsLogging;

    // STARTING POS
    frc::SendableChooser<std::string> m_startChooser;

    // Shooter shooter_{"Shooter", true, true};

    // auto lineup
    AutoAngLineup m_autoLineup;

    bool m_amp = true;
};
