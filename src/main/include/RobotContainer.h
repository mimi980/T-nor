// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <frc/Joystick.h>
#include <frc2/command/Command.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <frc2/command/RunCommand.h>
#include "subsystem/Drivetrain.h"
#include "subsystem/Shooter.h"
#include "subsystem/Feeder.h"
#include "subsystem/Planetary.h"
#include "subsystem/Camera.h"
#include "subsystem/Intake.h"

#include "command/Drive.h"
#include "command/PreShoot.h"
#include "command/Shoot.h"
#include "command/TakeNote.h"
#include "command/SpitNote.h"
#include "command/NearShoot.h"

class RobotContainer
{
public:
  RobotContainer();
  void ConfigureButtonBindings();

  frc::Joystick m_joystickRight{0};
  frc::Joystick m_joystickLeft{1};

  Drivetrain m_drivetrain;
  Shooter m_shooter;
  Feeder m_feeder;
  Planetary m_planetary;
  Camera m_camera;
  Intake m_intake;

private:
};
