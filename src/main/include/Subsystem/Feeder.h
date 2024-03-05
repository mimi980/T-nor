// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/Phoenix/motorcontrol/can/TalonFX.h>
#include "Constants.h"

class Feeder : public frc2::SubsystemBase
{
public:
  Feeder();
  void Periodic() override;
  void ActivateFeeder(double speed);

private:
  ctre::phoenix::motorcontrol::can::TalonFX m_feederMotor{ID_MOTOR_FEEDER};
};
