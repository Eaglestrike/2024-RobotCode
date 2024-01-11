// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "Controller/ControllerMap.h"

using namespace Actions;

Robot::Robot() :
  m_swerveController{true, false},
  m_client{"10.1.14.21", 5807, 500, 5000},
  m_logger{"log", {}}
  {
  // navx
  try
  {
    m_navx = new AHRS(frc::SerialPort::kMXP);
  }
  catch (const std::exception &e)
  {
    std::cerr << e.what() << std::endl;
  }

  m_logger.SetLogToConsole(true);
}

void Robot::RobotInit() {
  ShuffleboardInit();

  m_swerveController.Init();
}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
  ShuffleboardPeriodic();

  m_logger.Periodic(Utils::GetCurTimeS());
}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  m_swerveController.SetAngCorrection(true);
  m_swerveController.SetAutoMode(false);
}

void Robot::TeleopPeriodic() {
  double lx = m_controller.getWithDeadContinuous(SWERVE_STRAFEX, 0.1);
  double ly = m_controller.getWithDeadContinuous(SWERVE_STRAFEY, 0.1);

  double rx = m_controller.getWithDeadContinuous(SWERVE_ROTATION, 0.1);

  double mult = SwerveConstants::NORMAL_SWERVE_MULT;
  double vx = std::clamp(lx, -1.0, 1.0) * mult;
  double vy = std::clamp(ly, -1.0, 1.0) * mult;
  double w = -std::clamp(rx, -1.0, 1.0) * mult / 2;

  vec::Vector2D setVel = {-vy, -vx};
  double curYaw = m_navx->GetYaw();

  m_swerveController.SetRobotVelocityTele(setVel, w, 0, 0);
  m_swerveController.Periodic();
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

/**
 * Shuffleboard Init
*/
void Robot::ShuffleboardInit() {
  frc::SmartDashboard::PutBoolean("Logging", false);
}

/**
 * Shuffleboard Periodic
 */
void Robot::ShuffleboardPeriodic() {
  bool isLogging = frc::SmartDashboard::GetBoolean("Logging", true);
  if (isLogging && !m_prevIsLogging) {
    m_logger.Enable();
  } else if (!isLogging & m_prevIsLogging) {
    m_logger.Disable();
  }
  m_prevIsLogging = isLogging;
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
