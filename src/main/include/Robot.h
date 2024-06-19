// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include "Subsystem/Drivetrain.h"
#include "frc/Joystick.h"
#include "lib/NL/MotionControl/DriveTrain/Characterization/NLMotorCharacterization.h"
#include "lib/NL/MotionControl/Trajectory/NLFollowerTank.h"
#include "lib/NL/MotionControl/Trajectory/NLTrajectoryPack.h"
#include "lib/NL/MotionControl/Trajectory/NLTrajectoryActionMessagesEnum.h"
#include "lib/NL/MotionControl/Trajectory/NLTrajectorySystemMessage.h"
#include <AHRS.h>
#include <frc/Timer.h>
#include "RobotContainer.h"
#include <frc/smartdashboard/SendableChooser.h>
#include <units/pressure.h>
#include "lib/NLCsv.h"

class Robot : public frc::TimedRobot
{
public:
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

  void TakeNoteSwitch();
  void ShootSwitch();
  void PreShoot();
  void NearShoot();
  void Shoot(double speed, double angle);

  double m_deltatime;
  double m_previoustime;

  /// @brief //////////////////////////////////////////////////////////////////////////////////////////////////////////////
  void Center2Auto();
  enum class StateCenter2Auto
  {
    Nearshoot,
    Backward,
    wait,
    Shooting,
    End
  };
  StateCenter2Auto m_stateCenter2Auto;
  bool m_center2Auto;

  void ShootOnly();
  enum class StateShootOnly
  {
    Wait,
    Nearshoot,
    Backward,
    End
  };
  StateShootOnly m_stateShootOnly;
  bool m_shootOnly;
  int m_countWait;

  enum class StateTakeNote
  {
    Catch,
    Recul,
    Loaded,
    End,
    nule
  };

  StateTakeNote m_stateTakeNote;
  bool m_takeNote;

  enum class StateShootSwitch
  {
    Loaded,
    PreShoot,
    Shoot,
    Shooting,
    End
  };

  StateShootSwitch m_stateShootSwitch;
  double m_goal;
  double shooter_speed;
  double planteray_angle;
  int m_count;
  bool m_shoot;

  bool m_preShoot;

  enum class StateNearShoot
  {
    PreShoot,
    Shoot,
    Shooting,
    End
  };

  StateNearShoot m_stateNearShoot;

  enum class StateShoot
  {
    Loaded,
    PreShoot,
    Shoot,
    Shooting,
    End
  };

  StateShoot m_stateShoot;

  RobotContainer m_robotContainer;
  // enum STATE
  // {
  //   PATH_ERROR = 0, ///< L'initialisation du path following a rencontr� un probl�me ( erreur au chargement tr�s probablement ). Le Robot ne peut-�tre en �tat PATH_FOLLOWING.
  //   PATH_FOLLOWING, ///< Le robot est en �tat de suivit de chemin.
  //   PATH_END        ///< La Vitesse  est en d�passement.
  // };

  // STATE m_state;

  NLMOTOR_CHARACTERIZATION m_CrtzL;
  NLMOTOR_CHARACTERIZATION m_CrtzR;

  NLTRAJECTORY_PACK m_TrajectoryPack;
  NLFOLLOWER_TANK m_follower;

  // AHRS m_gyro{frc::SerialPort::Port::kUSB};
  int m_state;

  // Auto Selector
  frc::SendableChooser<std::string> m_autoChooser;
  frc::SendableChooser<std::string> m_sideChooser;

  const std::string kAutoNameDefault = "Default";
  const std::string kArenaBlueSide = "Blue";
  const std::string kArenaRedSide = "Red";
  const std::string kAutoNameAmpNear = "Amp Near";
  const std::string kAutoNameCenterNear = "Center Near";
  const std::string kAutoNameSourceNear = "Source Near";
  const std::string kAutoNameAmpFar = "Amp Far";
  const std::string kAutoNameCenterFar = "Center Far";
  const std::string kAutoNameSourceFar = "Source Far";

  std::string m_autoSelected;
  std::string m_sideSelected;

  int m_countable;
  double m_encoderRightValue;
  double m_encoderLeftValue;
  double m_gyroAngle;
  double m_VoltageLeft;
  double m_VoltageRight;
  double m_LeftV;
  double m_RightV;
};
