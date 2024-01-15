// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Joystick.h>
#include <cameraserver/CameraServer.h>
#include <frc/Compressor.h>


class RobotContainer
{
public:
  RobotContainer();

  void ConfigureButtonBindings();
  frc::Joystick m_joystickRight{0};
  frc::Joystick m_joystickLeft{1};
  frc::Joystick m_joystickCopilot{2};

private:
  cs::UsbCamera m_CameraPilote;

  frc::Compressor m_compressor{frc::PneumaticsModuleType::REVPH};
};
