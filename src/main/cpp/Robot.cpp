// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <iostream>

#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "Constants/AutoConstants.h"
#include "Constants/AutoLineupConstants.h"
#include "Controller/ControllerMap.h"
#include "Util/SideHelper.h"

using namespace Actions;

Robot::Robot() :
  m_swerveController{true, false},
  m_auto{true, m_swerveController, m_odom, m_autoLineup, m_intake, m_shooter},
  m_client{"10.1.14.46", 5590, 500, 5000},
  m_isSecondTag{false},
  m_odom{false},
  m_logger{"log", {"ang input", "navX ang", "Unique ID", "Tag ID", "Raw camX", "Raw camY", "Raw angZ"}},
  m_prevIsLogging{false},
  m_autoLineup{false, m_odom}
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
  m_autoLineup.ShuffleboardInit();

  m_navx->Reset();
  m_navx->ZeroYaw();
  m_odom.Reset();

  m_intake.Init();
  m_client.Init();
  m_swerveController.Init();

  m_shooter.Init();
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
  m_autoLineup.ShuffleboardPeriodic();

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

  m_shooter.Periodic();
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
  m_swerveController.ResetAngleCorrection(m_odom.GetAng());
  m_swerveController.SetAutoMode(false);
}

void Robot::TeleopPeriodic() {
  //Swerve
  double lx = m_controller.getWithDeadContinuous(SWERVE_STRAFEX, 0.15);
  double ly = m_controller.getWithDeadContinuous(SWERVE_STRAFEY, 0.15);

  double rx = m_controller.getWithDeadContinuous(SWERVE_ROTATION, 0.15);

  int posVal = m_controller.getValue(ControllerMapData::SCORING_POS, 0);
  if (posVal) {
    m_autoLineup.SetTarget(SideHelper::GetManualLineupAng(posVal - 1));
  }

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

  // auto lineup to amp
  if (m_controller.getPressedOnce(AMP_AUTO_LINEUP)) {
    m_autoLineup.SetTarget(AutoLineupConstants::AMP_LINEUP_ANG);
    m_autoLineup.Start();
  }
  //Intake
  if(m_controller.getPressedOnce(INTAKE_TO_AMP)){
    m_amp = true;
  }
  if(m_controller.getPressedOnce(INTAKE_TO_CHANNEL)){
    m_amp = false;
  }

  //Shooting (amp or speaker)
  if (m_controller.getPressedOnce(SHOOT)){
    if (m_amp) {
      m_intake.AmpOuttake();
    }
    else {
      m_shooter.SetOdometry(m_odom.GetPos(), m_odom.GetVel(),m_odom.GetAng());
      m_shooter.Prepare(true); //TODO use utils
      if(m_shooter.CanShoot()){
        m_intake.FeedIntoShooter(); //Shoot when ready
      }
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

  if (m_controller.getPressed(AMP_AUTO_LINEUP)) {
    double angVel = m_autoLineup.GetAngVel();
    m_swerveController.SetRobotVelocityTele(setVel, angVel, curYaw, curJoystickAng);
  } else {
    m_autoLineup.Stop();
    m_swerveController.SetRobotVelocityTele(setVel, w, curYaw, curJoystickAng);
  }

  m_intake.TeleopPeriodic();
  m_swerveController.Periodic();
  m_autoLineup.Periodic();

  m_shooter.TeleopPeriodic();
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {
  // STARTING POS
  {
    std::string selected = m_startChooser.GetSelected();
    AutoConstants::StartPose startPos = SideHelper::GetStartingPose(selected);
    double joystickAng = SideHelper::GetJoystickAng();
    m_odom.SetStartingConfig(startPos.pos, startPos.ang, joystickAng);
  }

  // AUTO
  std::string path;
  for(uint i = 0; i < AUTO_LENGTH; i++){
    path = m_autoChoosers[i].GetSelected();
    if(AutoConstants::PATHS.find(path) != AutoConstants::PATHS.end()){
      m_auto.SetPath(i, AutoConstants::PATHS.at(path));
    }
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
  for (auto path : AutoConstants::PATHS) {
    for (auto& chooser: m_autoChoosers){
      chooser.AddOption(path.first, path.first);
    }
  }
  for(uint i = 0; i < AUTO_LENGTH; i++){
    frc::SmartDashboard::PutData("Auto " + std::to_string(i+1), &m_autoChoosers[i]);
  }
  // STARTING POS
  {
    m_startChooser.SetDefaultOption("Middle", "Middle");
    m_startChooser.AddOption("Left", "Left");
    m_startChooser.AddOption("Middle", "Middle");
    m_startChooser.AddOption("Right", "Right");
    frc::SmartDashboard::PutData("Starting Position", &m_startChooser);
  }

  // DEBUG
  {
    // double navXAngVel = m_odom.GetAngVel();
    // double wheelAngVel = m_swerveController.GetRobotAngularVel();

    // frc::SmartDashboard::PutNumber("navX Ang Vel", navXAngVel);
    // frc::SmartDashboard::PutNumber("wheel Ang Vel", m_swerveController.GetRobotAngularVel());
    // frc::SmartDashboard::PutNumber("diff Ang Vel", navXAngVel - wheelAngVel);

    // frc::SmartDashboard::PutNumber("wheel ang", m_wheelAng);
    // frc::SmartDashboard::PutNumber("error ang", 0);
  }
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
