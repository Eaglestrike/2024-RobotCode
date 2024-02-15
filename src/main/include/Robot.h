// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#define SWERVE_AUTOTUNING false

#include <string>

#include <AHRS.h>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/Field2d.h>

#include "ShuffleboardSender/ShuffleboardSender.h"

#include "FFAutotuner/FFAutotuner.h"

#include "Auto/Auto.h"
#include "Auto/AutoChooser.h"
#include "Controller/Controller.h"
#include "Drive/AutoAngLineup.h"
#include "Drive/SwerveControl.h"
#include "Climb/Climb.h"
#include "Intake/Intake.h"
#include "Util/Logger.h"
#include "Util/Odometry.h"
#include "Util/SocketClient.h"

#include "Constants/AutoConstants.h"

#include "Shooter/Shooter.h"

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

    // Logger
    FRCLogger m_logger;
    bool m_prevIsLogging;

    // Swerve
    SwerveControl m_swerveController;

    //intake
    Intake m_intake;

    //climb
    Climb m_climb;

    // Shooter
    Shooter m_shooter;

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
    frc::Field2d m_field;
    std::string m_prevSelectedStart = "";

    // auto lineup
    AutoAngLineup m_autoLineup;

    //Auto 
    Auto m_auto;
    AutoChooser m_autoChooser; 

    // STARTING POS + AUTO CHOOSERS
    frc::SendableChooser<std::string> m_startChooser;
    frc::SendableChooser<std::string> m_autoChoosers[AutoConstants::POS_ARR_SIZE - 2];
    frc::SendableChooser<std::string> m_autoEndChooser;

    // current buttonboard states
    bool m_amp = true;
    bool m_wristManual = false;
    bool m_climbManual = false;
    bool m_eject = false;

    // zerored states
    bool m_intakeZeroed = false;
    bool m_climbZeroed = false;
};
