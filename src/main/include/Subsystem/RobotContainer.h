// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <frc/Joystick.h>
#include <frc2/command/Command.h>
#include <frc2/JoystickButton.h>
#include "subsystem/Drivetrain.h"
#include "subsystem/Shooter.h"
#include "subsystem/Feeder.h"
#include "subsystem/Planetary.h"
#include "subsystem/Camera.h"
#include "subsystem/Intake.h"

#include "command/Drive.h"
#include "command/PreShoot.h"
#include "command/Shoot.h"

class RobotContainer
{
public:
  RobotContainer();
  void ConfigureButtonBindings();

private:
  frc::Joystick m_joystickRight{0};
  frc::Joystick m_joystickLeft{1};

  Drivetrain m_drivetrain;
  Shooter m_shooter;
  Feeder m_feeder;
  Planetary m_planetary;
  Camera m_camera;
  Intake m_intake;
};
