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
#include "RobotContainer.h"

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

  enum class StateTakeNote
  {
    Catch,
    Recul,
    Loaded,
    End
  };

  StateTakeNote m_stateTakeNote;
  bool m_takeNote;

  enum class StateShoot
  {
    Loaded,
    PreShoot,
    Shoot,
    Shooting,
    End
  };

  StateShoot m_stateShoot;
  double m_goal;
  double shooter_speed;
  double planteray_angle;
  int m_count;
  bool m_shoot;

  bool m_preShoot;

  enum class StateNearShoot
  {
    Loaded,
    PreShoot,
    Shoot,
    Shooting,
    End
  };

  StateNearShoot m_stateNearShoot;

  RobotContainer m_robotContainer;
  enum STATE
  {
    PATH_ERROR = 0, ///< L'initialisation du path following a rencontr� un probl�me ( erreur au chargement tr�s probablement ). Le Robot ne peut-�tre en �tat PATH_FOLLOWING.
    PATH_FOLLOWING, ///< Le robot est en �tat de suivit de chemin.
    PATH_END        ///< La Vitesse  est en d�passement.
  };

  STATE m_state;

  NLMOTOR_CHARACTERIZATION m_CrtzL;
  NLMOTOR_CHARACTERIZATION m_CrtzR;

  NLTRAJECTORY_PACK m_TrajectoryPack;
  NLFOLLOWER_TANK m_follower;

  AHRS m_gyro{frc::SerialPort::Port::kOnboard};
};
