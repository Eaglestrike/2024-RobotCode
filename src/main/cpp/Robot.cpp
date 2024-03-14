// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <iostream>
#include <cmath>
#include <string>
#include <vector>

#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "Constants/AutoConstants.h"
#include "Constants/AutoLineupConstants.h"
#include "Controller/ControllerMap.h"
#include "DebugConfig.h"
#include "Util/SideHelper.h"

using namespace Actions;

Robot::Robot() :
  //Controller
  m_controller{},
  //Logger
  m_logger{"log", {"Cams Stale", "Cams Connected", "Tag Detected", "Pos X", "Pos Y", "Manual Pos Val", "Amp mode", "Intake State", "In intake", "In channel", "Shot Vel", "Shot Ang", "Pivot pos", "Pivot vel", "Pivot state", "Top Flywheel state", "Bottom flywheel state", "Shooter state", "Can shoot", "Pivot tol", "Trim X", "Trim Y", "Shooter ang lineup targ", "Shooter ang lineup exp", "Shooter ang lineup state", "Auto path num", "Auto index", "Drive Started", "Drive Finished", "Shooter Started", "Shooter Finished", "Intake Started", "Intake Finished"}},
  m_prevIsLogging{false},
  //Mechanisms
  m_swerveController{EnableConfig::DRIVE, DebugConfig::DRIVE},
  m_intake{EnableConfig::INTAKE, DebugConfig::INTAKE},
  m_climb{EnableConfig::CLIMB, DebugConfig::CLIMB},
  m_shooter{"Shooter", EnableConfig::SHOOTER, DebugConfig::SHOOTER.SHOOTER},
  //Sensors
  m_client{"10.1.14.202", 44590, 500, 5000}, // 10.1.14.202
  m_isSecondTag{false},
  m_odom{false},
  //Auto
  m_autoLineup{false, m_odom},  
  m_auto{DebugConfig::AUTO, m_swerveController, m_odom, m_autoLineup, m_intake, m_shooter, m_logger},
  m_autoChooser{false, m_auto},
  m_led{}
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
  m_logger.SetLogToConsole(false);
  m_logger.SetLevel(LogLevels::INFO);

  // Periodic fast
  AddPeriodic([&]()
              {
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
        frc::SmartDashboard::PutNumber("Cams x dist", x);
        frc::SmartDashboard::PutNumber("Cams z dist", y);
        m_odom.UpdateCams({x, y}, tagId, uniqueId, age);
      }

      m_isSecondTag = true;
    } else {
      m_isSecondTag = false;
    } },
              5_ms, 2_ms);
}

void Robot::RobotInit()
{
  ShuffleboardInit();
  m_autoChooser.ShuffleboardInit();
  m_auto.ShuffleboardInit();
  m_odom.ShuffleboardInit();
  m_autoLineup.ShuffleboardInit();

  m_navx->Reset();
  m_navx->ZeroYaw();
  m_odom.Reset();

  m_intake.Init();
  m_climb.Init();
  m_client.Init();
  m_swerveController.Init();
  m_shooter.Init();
  m_led.Init();
}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic()
{
  ShuffleboardPeriodic();
  m_autoChooser.ShuffleboardPeriodic();
  m_auto.ShuffleboardPeriodic();
  m_odom.ShuffleboardPeriodic();
  m_autoLineup.ShuffleboardPeriodic();

  // ZERO DRIVEBASE
  if (m_controller.getPressedOnce(ZERO_YAW))
  {
    m_driveZeroed = true;
    m_navx->Reset();
    m_navx->ZeroYaw();
    m_odom.Reset();
    m_swerveController.ResetAngleCorrection(m_odom.GetAng());
    m_swerveController.ResetFF();
  }

  // ZERO CLIMB + INTAKE
  if (m_controller.getPressed(ZERO_1) && m_controller.getPressed(ZERO_2))
  {
    if (m_controller.getPressed(ZERO_CLIMB))
    {
      m_climbZeroed = true;
      m_climb.Zero();
    }
    if (m_controller.getPressedOnce(ZERO_INTAKE))
    {
      m_intakeZeroed = true;
      m_intake.Zero();
    }
  }

  m_shooter.Trim(m_controller.getValueOnce(ControllerMapData::GET_SHOOTER_TRIM, {0, 0})); // Trim shooter
  m_shooter.SetOdometry(m_odom.GetPos(), m_odom.GetVel(), m_odom.GetYaw());
  m_shooter.SetGamepiece(m_intake.InChannel());

  // LED vertical
  if (m_intake.HasGamePiece() || m_eject)
  {
    if (m_intake.InChannel()) {
      m_led.SetLEDSegment(LEDConstants::LEDSegment::VERTICAL, 0, 100, 0, 0);
    } else {
      m_led.SetLEDSegment(LEDConstants::LEDSegment::VERTICAL, 0, 100, 0, 40);
    }
  }
  else
  {
    m_led.SetLEDSegment(LEDConstants::LEDSegment::VERTICAL, 0, 0, 255, 0);
  }

  if (m_shooter.IsManual()) {
    m_led.SetLEDSegment(LEDConstants::LEDSegment::HORIZONTAL, 255, 69, 0, 40);
  } else if (m_posVal != 0) {
    m_led.SetLEDSegment(LEDConstants::LEDSegment::HORIZONTAL, 255, 110, 199, 0);
  } else if (m_amp) {
    m_led.SetLEDSegment(LEDConstants::LEDSegment::HORIZONTAL, 0, 255, 0, 0);
  } else {
    m_led.SetLEDSegment(LEDConstants::LEDSegment::HORIZONTAL, 0, 0, 255, 0);
  }

#if SWERVE_AUTOTUNING
  m_swerveXTuner.ShuffleboardUpdate();
  m_swerveYTuner.ShuffleboardUpdate();
#endif
  m_logger.Periodic(Utils::GetCurTimeS());
  m_intake.Periodic();
  m_climb.Periodic();
  m_shooter.Periodic();
  m_led.Periodic();
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
void Robot::AutonomousInit()
{
  m_shooter.SetHooked(true);
  m_shooter.ZeroRelative();
  
  m_odom.SetAuto(true);
  m_swerveController.SetAngCorrection(false);
  m_swerveController.SetAutoMode(true);
  m_autoLineup.SetPID(AutoLineupConstants::ANG_AUTO_P, AutoLineupConstants::ANG_AUTO_I, AutoLineupConstants::ANG_AUTO_D);

  // zero
  if (!m_intakeZeroed) {
    m_intake.Zero();
    m_intakeZeroed = true;
  }
  if (!m_climbZeroed) {
    m_climb.Zero();
    m_climbZeroed = true;
  }
  m_navx->Reset();
  m_navx->ZeroYaw();
  m_odom.Reset();
  m_swerveController.ResetAngleCorrection(m_odom.GetAng());
  m_swerveController.ResetFF();
  m_driveZeroed = true;

  m_autoChooser.ProcessChoosers(false);
  m_auto.AutoInit();
  m_shooter.Stop();
}

void Robot::AutonomousPeriodic()
{
  m_auto.AutoPeriodic();
  m_swerveController.Periodic();
  m_intake.TeleopPeriodic();
  m_shooter.TeleopPeriodic();
}

void Robot::TeleopInit()
{
  m_odom.SetAuto(false);
  m_swerveController.SetAngCorrection(true);
  m_swerveController.ResetAngleCorrection(m_odom.GetAng());
  m_swerveController.SetAutoMode(false);
  m_autoLineup.SetPID(AutoLineupConstants::ANG_P, AutoLineupConstants::ANG_I, AutoLineupConstants::ANG_D);
  m_autoLineup.SetTarget(AutoLineupConstants::AMP_LINEUP_ANG);
  m_shooter.ZeroRelative();
  m_shooter.Stop();
}

void Robot::TeleopPeriodic()
{
  // logging controls
  if (m_controller.getPressedOnce(FORCE_SHOOT)) {
    m_logger.Warn("Input", "Force shooting");
  } else if (m_controller.getPressedOnce(SHOOT)) {
    m_logger.Info("Input", "Pressed shoot");
  } else if (m_controller.getPressedOnce(INTAKE)) {
    m_logger.Info("Input", "Pressed intaking");
  } else if (m_controller.getPressedOnce(MANUAL_EJECT_IN) || m_controller.getPressedOnce(MANUAL_EJECT_OUT)) {
    m_logger.Info("Input", "Ejecting");
  }

  // Swerve
  double lx = m_controller.getWithDeadContinuous(SWERVE_STRAFEX, 0.15);
  double ly = m_controller.getWithDeadContinuous(SWERVE_STRAFEY, 0.15);
  double rx = m_controller.getWithDeadContinuous(SWERVE_ROTATION, 0.15);

  // angle lock
  int ctrlVal = m_controller.getValueOnce(ControllerMapData::SCORING_POS, 0);
  if (ctrlVal != 0) {
    m_shooter.Stop();
    m_posVal = ctrlVal;
  }

  if (m_controller.getPressedOnce(SHOOT_AUTO)) {
    m_shooter.Stop();
    m_posVal = 0;
  }

  // slow mode
  double mult = SwerveConstants::NORMAL_SWERVE_MULT;
  if (m_controller.getPressed(SLOW_MODE))
  {
    mult = SwerveConstants::SLOW_SWERVE_MULT;
  }
  double vx = std::clamp(lx, -1.0, 1.0) * mult;
  double vy = std::clamp(ly, -1.0, 1.0) * mult;
  double w = -std::clamp(rx, -1.0, 1.0) * mult / 2;

  // velocity vectors
  vec::Vector2D setVel = {-vy, -vx};
  double curYaw = m_odom.GetAngNorm();
  double curJoystickAng = m_odom.GetJoystickAng();

  // auto lineup to amp
  if (m_controller.getPOVDownOnce(AMP_AUTO_LINEUP))
  {
    m_logger.Info("Input", "Starting Angle Lineup");
    m_autoLineup.SetTarget(AutoLineupConstants::AMP_LINEUP_ANG);
    m_autoLineup.Start();
  }

  // Intake
  if (!m_wristManual)
  {
    if (m_controller.getPOVDownOnce(HALF_STOW))
    {
      m_intake.HalfStow();
    }
    if (m_controller.getPressedOnce(INTAKE_TO_AMP))
    {
      m_amp = true;
    }
    if (m_controller.getPressedOnce(INTAKE_TO_CHANNEL))
    {
      m_amp = false;
    }

    //Shooting
    if (m_controller.getPressed(FORCE_SHOOT)) {
      m_intake.FeedIntoShooter();
    } else if (m_controller.getPressed(SHOOT)){
      if(m_intake.HasGamePiece()){
        if (!m_amp) {
          if (m_posVal != 3) {
            m_autoLineup.Recalc(m_shooter.GetTargetRobotYaw());
          } else {
            m_autoLineup.Recalc(m_odom.GetAngNorm());
          }
        }
        if(m_shooter.CanShoot(m_posVal)){
          m_intake.FeedIntoShooter();
        }
      }
      else
      {
        m_intake.FeedIntoShooter();
      }
    }
    else if (m_controller.getPressed(INTAKE))
    {
      m_intake.Passthrough();
      m_shooter.Stroll();
    }
    else if ((m_intake.GetState() == Intake::AMP_INTAKE || m_intake.GetState() == Intake::PASSTHROUGH) && !m_intake.HasGamePiece())
    {
      m_intake.Stow();
    }
  }

  //Shooter config
  if(m_amp){
    m_shooter.Amp();
  }
  else{
    if(m_posVal == 0){
      m_shooter.Prepare(m_odom.GetPos(), m_odom.GetVel(), true);  //Shoot into speaker
    }
    else{
      vec::Vector2D manualLineupPos = SideHelper::GetPos(AutoLineupConstants::BLUE_SHOOT_LOCATIONS[m_posVal - 1]); //Manual positions
      m_shooter.Prepare(manualLineupPos, {0, 0}, true);
    }
  }

  // Manual
  if (m_controller.getTriggerDown(MANUAL_1) && m_controller.getTriggerDown(MANUAL_2))
  {
    m_climb.SetManualInput(m_controller.getWithDeadband(MANUAL_CLIMB));
    if (m_controller.getPressedOnce(BRAKE))
    {
      m_climb.ChangeBrake(true);
    }
    else if (m_controller.getPressedOnce(UNBRAKE))
    {
      m_climb.ChangeBrake(false);
    }
    m_climbManual = true;

    double wristCtrl = m_controller.getWithDeadband(MANUAL_INTAKE_WRIST);
    if (std::abs(wristCtrl) > 0.05)
    {
      if (!m_wristManual)
      {
        m_intake.SetManual(true);
      }
      m_wristManual = true;
    }
    m_intake.SetManualInput(wristCtrl);
  }
  else
  {
    if (m_wristManual)
    {
      m_intake.SetManual(false);
      m_intake.SetManualInput(0);
    }

    m_climbManual = false;
    m_wristManual = false;
  }

  //Shooter Manual
  double shooterManualPos = m_controller.getValueOnce(ControllerMapData::SHOOT_MANUAL, -10000.0);
  if (shooterManualPos != -10000.0)
  {
    m_shooter.ManualTarget(shooterManualPos);
  }

  // eject
  if (m_controller.getPressed(MANUAL_EJECT_IN) || m_controller.getPressed(MANUAL_EJECT_OUT))
  {
    m_eject = true;
    m_shooter.Eject();
    if (m_controller.getPressed(MANUAL_EJECT_IN) && m_controller.getPressed(MANUAL_EJECT_OUT))
    {
      m_intake.EjectSplit();
    }
    else if (m_controller.getPressed(MANUAL_EJECT_IN))
    {
      m_intake.EjectForward();
    }
    else
    {
      m_intake.EjectBack();
    }
  }
  else if (m_eject == true)
  {
    m_eject = false;
    m_intake.EjectStop();
    m_shooter.Stroll();
  }

  // climb
  if (!m_climbManual)
  {
    if (m_controller.getPOVDownOnce(CLIMB))
    {
      m_intake.Climb();
      m_climb.PullUp();
    }
    else if (m_controller.getPOVDownOnce(EXTEND))
    {
      m_climb.Extend();
    }
  }

  // auto lineup
  if (m_controller.getPOVDown(AMP_AUTO_LINEUP) ||
      (m_controller.getPressed(SHOOT) && !m_amp && m_shooter.UseAutoLineup()) // Angle lineup when shooting
  )
  {
    double angVel = m_autoLineup.GetAngVel();
    m_swerveController.SetRobotVelocityTele(setVel, angVel, curYaw, curJoystickAng);
  }
  else
  {
    m_autoLineup.Stop();
    m_swerveController.SetRobotVelocityTele(setVel, w, curYaw, curJoystickAng);
  }

  // Shooter

  m_climb.TeleopPeriodic();
  m_intake.TeleopPeriodic();
  m_swerveController.Periodic();
  m_autoLineup.Periodic();
  m_shooter.TeleopPeriodic();
}

void Robot::DisabledInit() {
}

void Robot::DisabledPeriodic()
{
  // STARTING POS + AUTO CHOOSER
  {
    // process starting position
    std::string selectedStart = m_startChooser.GetSelected();
    AutoConstants::StartPose startPos = SideHelper::GetStartingPose(selectedStart);
    double joystickAng = SideHelper::GetJoystickAng();
    if (selectedStart != m_prevSelectedStart || SideHelper::IsBlue() != m_prevIsBlue)
    {
      m_odom.SetStartingConfig(startPos.pos, startPos.ang, joystickAng);
    }
    m_prevSelectedStart = selectedStart;
    m_prevIsBlue = SideHelper::IsBlue();

    // put starting position in auto chooser
    m_autoChooser.SetPosition(0, selectedStart, "");

    // put intermediate auto paths in auto chooser
    for (int i = 0; i < AutoConstants::POS_ARR_SIZE - 2; i++)
    {
      std::string path = m_autoChoosers[i].GetSelected();
      std::string secondary = "";

      if (path == AutoConstants::L_FAR || path == AutoConstants::ML_FAR)
      {
        secondary = AutoConstants::L_SCORE;
      }
      if (path == AutoConstants::R_FAR || path == AutoConstants::MR_FAR)
      {
        secondary = AutoConstants::R_SCORE;
      }
      if (path == AutoConstants::M_FAR)
      {
        secondary = AutoConstants::M_SCORE;
      }
      if (path == AutoConstants::S_NAME)
      {
        path = "";
      }
      m_autoChooser.SetPosition(i + 1, path, secondary);
    }

    // put end path in auto chooser
    std::string end = m_autoEndChooser.GetSelected();
    if (end == AutoConstants::S_NAME)
    {
      end = "";
    }
    m_autoChooser.SetPosition(AutoConstants::POS_ARR_SIZE - 1, end, "");
  }
}

void Robot::TestInit() {
  m_swerveController.SetAutoMode(false);
}

void Robot::TestPeriodic()
{
  // Swerve
  double lx = m_controller.getWithDeadContinuous(SWERVE_STRAFEX, 0.15);
  double ly = m_controller.getWithDeadContinuous(SWERVE_STRAFEY, 0.15);
  double rx = m_controller.getWithDeadContinuous(SWERVE_ROTATION, 0.15);

  double mult = SwerveConstants::NORMAL_SWERVE_MULT;
  if (m_controller.getPressed(SLOW_MODE))
  {
    mult = SwerveConstants::SLOW_SWERVE_MULT;
  }
  double vx = std::clamp(lx, -1.0, 1.0) * mult;
  double vy = std::clamp(ly, -1.0, 1.0) * mult;
  double w = -std::clamp(rx, -1.0, 1.0) * mult / 2;

  // velocity vectors
  vec::Vector2D setVel = {-vy, -vx};
  double curYaw = m_odom.GetAngNorm();
  double curJoystickAng = m_odom.GetJoystickAng();

  double curAng = m_navx->GetAngle();
  if (!SwerveConstants::NAVX_UPSIDE_DOWN)
  {
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

  m_swerveController.SetRobotVelocity({xVolts, yVolts}, 0.0, curYaw);
#else
  m_swerveController.SetRobotVelocityTele(setVel, w, curYaw, curJoystickAng);
#endif

  if (m_controller.getPressed(INTAKE))
  {
    m_intake.Passthrough();
  }
  if (m_controller.getPressed(SHOOT))
  {
    m_intake.FeedIntoShooter();
  }

  m_swerveController.Periodic();
  m_shooter.TeleopPeriodic();
  m_intake.TeleopPeriodic();
}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

/**
 * Shuffleboard Init
 */
void Robot::ShuffleboardInit()
{
  frc::SmartDashboard::PutBoolean("Logging", false);

  // STARTING POS
  {
    m_startChooser.SetDefaultOption(AutoConstants::M_START, AutoConstants::M_START);
    m_startChooser.AddOption(AutoConstants::L_START, AutoConstants::L_START);
    m_startChooser.AddOption(AutoConstants::M_START, AutoConstants::M_START);
    m_startChooser.AddOption(AutoConstants::R_START, AutoConstants::R_START);
    frc::SmartDashboard::PutData("Start", &m_startChooser);

    for (int i = 0; i < AutoConstants::POS_ARR_SIZE - 2; i++)
    {
      m_autoChoosers[i].SetDefaultOption(AutoConstants::S_NAME, AutoConstants::S_NAME);
      m_autoChoosers[i].AddOption(AutoConstants::L_NEAR, AutoConstants::L_NEAR);
      m_autoChoosers[i].AddOption(AutoConstants::M_NEAR, AutoConstants::M_NEAR);
      m_autoChoosers[i].AddOption(AutoConstants::R_NEAR, AutoConstants::R_NEAR);
      m_autoChoosers[i].AddOption(AutoConstants::L_FAR, AutoConstants::L_FAR);
      m_autoChoosers[i].AddOption(AutoConstants::ML_FAR, AutoConstants::ML_FAR);
      m_autoChoosers[i].AddOption(AutoConstants::M_FAR, AutoConstants::M_FAR);
      m_autoChoosers[i].AddOption(AutoConstants::MR_FAR, AutoConstants::MR_FAR);
      m_autoChoosers[i].AddOption(AutoConstants::R_FAR, AutoConstants::R_FAR);
      m_autoChoosers[i].AddOption(AutoConstants::S_NAME, AutoConstants::S_NAME);
      frc::SmartDashboard::PutData("Auto " + std::to_string(i + 1), &(m_autoChoosers[i]));
    }

    m_autoEndChooser.SetDefaultOption(AutoConstants::S_NAME, AutoConstants::S_NAME);
    m_autoEndChooser.AddOption(AutoConstants::L_FAR, AutoConstants::L_FAR);
    m_autoEndChooser.AddOption(AutoConstants::ML_FAR, AutoConstants::ML_FAR);
    m_autoEndChooser.AddOption(AutoConstants::M_FAR, AutoConstants::M_FAR);
    m_autoEndChooser.AddOption(AutoConstants::MR_FAR, AutoConstants::MR_FAR);
    m_autoEndChooser.AddOption(AutoConstants::R_FAR, AutoConstants::R_FAR);
    m_autoEndChooser.AddOption(AutoConstants::S_NAME, AutoConstants::S_NAME);
    frc::SmartDashboard::PutData("End", &m_autoEndChooser);
  }

  // ZERO
  {
    frc::SmartDashboard::PutBoolean("Intake Zeroed", m_intakeZeroed);
    frc::SmartDashboard::PutBoolean("Climb Zeroed", m_climbZeroed);
    frc::SmartDashboard::PutBoolean("Drive Zeroed", m_driveZeroed);
  }

  // MANUAL
  {
    frc::SmartDashboard::PutBoolean("Climb Manual", m_climbManual);
    frc::SmartDashboard::PutBoolean("Wrist Manual", m_wristManual);
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
void Robot::ShuffleboardPeriodic()
{
  // LOGGING
  {
    bool isLogging = frc::SmartDashboard::GetBoolean("Logging", true);
    if (isLogging && !m_prevIsLogging)
    {
      m_logger.Enable();
    }
    else if (!isLogging & m_prevIsLogging)
    {
      m_logger.Disable();
    }
    m_prevIsLogging = isLogging;

    m_intake.Log(m_logger);
    m_shooter.Log(m_logger);
    m_logger.LogNum("Manual Pos Val", m_posVal);
    m_logger.LogBool("Amp mode", m_amp);
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

    m_field.SetRobotPose(frc::Pose2d{units::meter_t{x(pos)}, units::meter_t{y(pos)}, units::radian_t{ang}});
    frc::SmartDashboard::PutData("Robot Field", &m_field);

    // logger
    m_logger.LogBool("Cams Stale", m_client.IsStale());
    m_logger.LogBool("Cams Connected", m_client.HasConn());
    m_logger.LogBool("Tag Detected", m_odom.GetTagDetected());
    m_logger.LogNum("Pos X", pos.x());
    m_logger.LogNum("Pos Y", pos.y());
    m_logger.LogNum("Ang", ang);
  }

  // ZERO
  {
    frc::SmartDashboard::PutBoolean("Intake Zeroed", m_intakeZeroed);
    frc::SmartDashboard::PutBoolean("Climb Zeroed", m_climbZeroed);
    frc::SmartDashboard::PutBoolean("Drive Zeroed", m_driveZeroed);
  }

  // TRIM
  {
    frc::SmartDashboard::PutNumber("Trim X", m_shooter.GetTrim().x());
    frc::SmartDashboard::PutNumber("Trim Y", m_shooter.GetTrim().y());
    m_logger.LogNum("Trim X", m_shooter.GetTrim().x());
    m_logger.LogNum("Trim Y", m_shooter.GetTrim().y());
  }

  // MANUAL
  {
    frc::SmartDashboard::PutBoolean("Climb Manual", m_climbManual);
    frc::SmartDashboard::PutBoolean("Wrist Manual", m_wristManual);
    frc::SmartDashboard::PutNumber("Manual Pos", m_posVal);
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

  // SOCKET LOGGING (only if enabled)
  {
    while (!m_client.m_logQueue.empty())
    {
      std::cout << m_client.m_logQueue.front() << "\n";
      m_client.m_logQueue.pop();
    }
  }
}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
