// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/Phoenix/motorcontrol/can/TalonFX.h>
#include "Constants.h"
#include "lib/RblUtils.h"

class Shooter : public frc2::SubsystemBase
{
public:
  Shooter();
  void SetShooter(double speed);
  double GetShooterVelocity();

  int getNearestElementId(double target);
  static constexpr double shooterDataTable[SHOOTER_TABLE_SIZE][3] = {
      // pitch, angle, vitesse
      {-17.4, 42, 0.5},
      {-13.8, 35, 0.5},
      {-11.3, 29, 0.5},
      {14.7, 23, 0.55},
      {12.1, 18.5, 0.65},
      {9.79, 14.5, 0.8},
      {8.3, 13, 0.85},
  };
  bool IsShoot;
  bool IsPreShoot;

private:
  ctre::phoenix::motorcontrol::can::TalonFX m_shooterMotorRight{ID_MOTOR_SHOOTER_DOWN};
  ctre::phoenix::motorcontrol::can::TalonFX m_shooterMotorLeft{ID_MOTOR_SHOOTER_HIGH};
};
