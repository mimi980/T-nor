// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/Phoenix/motorcontrol/can/TalonFX.h>

class Shooter : public frc2::SubsystemBase
{
public:
  Shooter();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

private:
  ctre::phoenix::motorcontrol::can::TalonFX m_shooterMotorRight{1};
  ctre::phoenix::motorcontrol::can::TalonFX m_shooterMotorLeft{2};
};
