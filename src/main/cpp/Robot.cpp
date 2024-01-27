// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <iostream>

#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "Constants/AutoConstants.h"
#include "Controller/ControllerMap.h"

using namespace Actions;

Robot::Robot() :
  m_swerveController{true, false},
  m_auto{true, m_swerveController, m_odom, m_intake},
  m_client{"10.1.14.46", 5590, 300, 5000},
  m_isSecondTag{false},
  m_odom{true},
  m_logger{"log", {"ang input", "navX ang", "Unique ID", "Tag ID", "Raw camX", "Raw camY", "Raw angZ"}},
  m_prevIsLogging{false}
{
  // navx
  try
  {
    m_navx = new AHRS(frc::SerialPort::kUSB2);
  }
  catch (const std::exception &e)
  {
    std::cerr << e.what() << std::endl;
  }

  // logger
  m_logger.SetLogToConsole(true);

  // Periodic fast
  AddPeriodic([&](){
    // update drivebase odometry
    double curAng = m_navx->GetAngle();
    double curYaw = m_navx->GetYaw();
    if (!SwerveConstants::NAVX_UPSIDE_DOWN) {
      curAng = -curAng;
      curYaw = -curYaw;
    }
    double angNavX = Utils::DegToRad(curAng);
    double yawNavX = Utils::DegToRad(curYaw);
    vec::Vector2D vel = m_swerveController.GetRobotVelocity(angNavX + m_odom.GetStartAng());
    double angVel = m_swerveController.GetRobotAngularVel();
    m_odom.UpdateEncoder(vel, angNavX, yawNavX, angVel);

    // update camera odometry
    std::vector<double> camData = m_client.GetData();
    if (m_client.HasConn() && !m_client.IsStale()) {
      // int camId = static_cast<int>(camData[0]);
      int tagId = static_cast<int>(camData[1]);
      double x = camData[2];
      double y = camData[3];
      // double angZ = camData[4];
      long long age = static_cast<long long>(camData[5]);
      long long uniqueId = static_cast<long long>(camData[6]);

      if (tagId != 0 && m_isSecondTag) {
        frc::SmartDashboard::PutNumber("Last Tag ID", tagId);
        m_odom.UpdateCams({x, y}, tagId, uniqueId, age);
        
        m_logger.LogNum("Raw camX", x);
        m_logger.LogNum("Raw camY", y);
        m_logger.LogNum("Unique ID", uniqueId);
        m_logger.LogNum("Tag ID", tagId);
      }

      m_isSecondTag = true;
    } else {
      m_isSecondTag = false;
    }
  }, 5_ms, 2_ms);
}

void Robot::RobotInit() {
  ShuffleboardInit();
  m_auto.ShuffleboardInit();
  m_odom.ShuffleboardInit();

  m_navx->Reset();
  m_navx->ZeroYaw();
  m_odom.Reset();
  m_odom.SetStartingConfig({1.3374878369040135, 5.704795059447731}, 0.0, 0);

  m_intake.Init();
  m_client.Init();
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
  m_auto.ShuffleboardPeriodic();
  m_odom.ShuffleboardPeriodic();

  if (m_controller.getPressedOnce(ZERO_YAW)) {
    m_navx->Reset();
    m_navx->ZeroYaw();
    m_odom.Reset();
    m_swerveController.ResetAngleCorrection(m_odom.GetAng());
    m_swerveController.ResetFF();
  }

  #if SWERVE_AUTOTUNING
  m_swerveXTuner.ShuffleboardUpdate();
  m_swerveYTuner.ShuffleboardUpdate();
  #endif

  m_logger.Periodic(Utils::GetCurTimeS());
  m_intake.Periodic();
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
void Robot::AutonomousInit() {
  m_swerveController.SetAngCorrection(false);
  m_swerveController.SetAutoMode(true);
  
  m_auto.AutoInit();
}

void Robot::AutonomousPeriodic() {
  m_auto.AutoPeriodic();
  m_swerveController.Periodic();
  m_intake.TeleopPeriodic();
}

void Robot::TeleopInit() {
  m_swerveController.SetAngCorrection(true);
  m_swerveController.SetAutoMode(false);
}

void Robot::TeleopPeriodic() {
  //Swerve
  double lx = m_controller.getWithDeadContinuous(SWERVE_STRAFEX, 0.15);
  double ly = m_controller.getWithDeadContinuous(SWERVE_STRAFEY, 0.15);

  double rx = m_controller.getWithDeadContinuous(SWERVE_ROTATION, 0.15);

  double mult = SwerveConstants::NORMAL_SWERVE_MULT;
  if (m_controller.getPressed(SLOW_MODE)) {
    mult = SwerveConstants::SLOW_SWERVE_MULT;
  }
  double vx = std::clamp(lx, -1.0, 1.0) * mult;
  double vy = std::clamp(ly, -1.0, 1.0) * mult;
  double w = -std::clamp(rx, -1.0, 1.0) * mult / 2;

  vec::Vector2D setVel = {-vy, -vx};
  double curYaw = m_odom.GetAngNorm();
  double curJoystickAng = m_odom.GetJoystickAng();

  m_logger.LogNum("ang input", rx);
  m_logger.LogNum("navX ang", m_odom.GetAng());

  m_swerveController.SetRobotVelocityTele(setVel, w, curYaw, curJoystickAng);
  m_swerveController.Periodic();

  //Intake
  if(m_controller.getPressedOnce(INTAKE_TO_AMP)){
    m_amp = true;
  }
  if(m_controller.getPressedOnce(INTAKE_TO_CHANNEL)){
    m_amp = false;
  }
  if (m_controller.getPressedOnce(SHOOT)){
    if (m_amp) {
      m_intake.AmpOuttake();
    } else {
      // code shooter later
      // if somehow switched from shooter to amp when in channel
      // HANDLE THIS CASE
    }
  }
  else if(m_controller.getPressed(INTAKE) && (!m_intake.HasGamePiece())){
    if (m_amp)
    m_intake.AmpIntake();
    else
    m_intake.Passthrough();
  } else if ((m_intake.GetState() == Intake::AMP_INTAKE || m_intake.GetState() == Intake::PASSTHROUGH) && !m_intake.HasGamePiece()){
    m_intake.Stow();
  }
  m_intake.TeleopPeriodic();
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {
  // AUTO
  std::string path = m_auto1.GetSelected();
  if(AutoConstants::PATHS.find(path) != AutoConstants::PATHS.end()){
    m_auto.SetPath(AutoConstants::PATHS.at(path),0);
  }
  path = m_auto2.GetSelected();
  if(AutoConstants::PATHS.find(path) != AutoConstants::PATHS.end()){
    m_auto.SetPath(AutoConstants::PATHS.at(path),1);
  }
}

void Robot::TestInit() {}

void Robot::TestPeriodic() {
  double curAng = m_navx->GetAngle();
  if (!SwerveConstants::NAVX_UPSIDE_DOWN) {
    curAng = -curAng;
  }

  double angNavX = Utils::DegToRad(curAng);

  vec::Vector2D pos = m_odom.GetPos();
  vec::Vector2D vel = m_swerveController.GetRobotVelocity(angNavX + m_odom.GetStartAng());

  #if SWERVE_AUTOTUNING
  m_swerveXTuner.setPose({pos.x(), vel.x(), 0.0});
  m_swerveYTuner.setPose({pos.y(), vel.y(), 0.0});

  double xVolts = m_swerveXTuner.getVoltage();
  double yVolts = m_swerveYTuner.getVoltage();

  m_swerveController.SetRobotVelocity({xVolts, yVolts}, 0.0, angNavX);
  #endif

  m_swerveController.Periodic();
}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

/**
 * Shuffleboard Init
*/
void Robot::ShuffleboardInit() {
  frc::SmartDashboard::PutBoolean("Logging", false);

  // Auto
  m_auto1.SetDefaultOption("Nothing")
  for (auto path : AutoConstants::PATHS) {
    m_auto1.AddOption(path.first, path.first);
    m_auto2.AddOption(path.first, path.first);
  }
  frc::SmartDashboard::PutData("Auto Spline Chooser", &m_auto1);
}

/**
 * Shuffleboard Periodic
 */
void Robot::ShuffleboardPeriodic() {
  // LOGGING
  {
    bool isLogging = frc::SmartDashboard::GetBoolean("Logging", true);
    if (isLogging && !m_prevIsLogging) {
      m_logger.Enable();
    } else if (!isLogging & m_prevIsLogging) {
      m_logger.Disable();
    }
    m_prevIsLogging = isLogging;
  }

  // ODOMETRY
  {
    double ang = m_odom.GetAng();
    vec::Vector2D pos = m_odom.GetPos();

    frc::SmartDashboard::PutBoolean("Cams Connected", m_client.HasConn());
    frc::SmartDashboard::PutBoolean("Cams Stale", m_client.IsStale());
    frc::SmartDashboard::PutBoolean("Tag Detected", m_odom.GetTagDetected());

    frc::SmartDashboard::PutNumber("Robot Angle", ang);
    frc::SmartDashboard::PutString("Robot Position", pos.toString());
  }

  // DEBUG
  {
    // double navXAngVel = m_odom.GetAngVel();
    // double wheelAngVel = m_swerveController.GetRobotAngularVel();

    // m_wheelAng += wheelAngVel * 0.02;

    // frc::SmartDashboard::PutNumber("navX Ang Vel", navXAngVel);
    // frc::SmartDashboard::PutNumber("wheel Ang Vel", m_swerveController.GetRobotAngularVel());
    // frc::SmartDashboard::PutNumber("diff Ang Vel", navXAngVel - wheelAngVel);
    // frc::SmartDashboard::PutNumber("wheel ang", m_wheelAng);
    // frc::SmartDashboard::PutNumber("error ang", m_odom.GetAng() - m_wheelAng);
  }
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
