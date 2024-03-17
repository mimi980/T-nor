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
      {-9.5, 12.5, 1.0},
      {-8.1, 15.0, 0.85},
      {-6.2, 17.0, 0.75},
      {-3.27, 21.0, 0.65},
      {0.2, 24.0, 0.6},
      {5.88, 30.0, 0.5},
      {13.4, 33.0, 0.5}

  };
  bool IsShoot;
  bool IsPreShoot;

private:
  ctre::phoenix::motorcontrol::can::TalonFX m_shooterMotorRight{ID_MOTOR_SHOOTER_DOWN};
  ctre::phoenix::motorcontrol::can::TalonFX m_shooterMotorLeft{ID_MOTOR_SHOOTER_HIGH};
};
