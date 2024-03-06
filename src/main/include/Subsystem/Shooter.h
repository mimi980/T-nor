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
      {-17.4, 33, 0.7},
      {-13.8, 36, 0.67},
      {-11.3, 30, 0.6},
      {-4.31, 24, 0.58},
      {7.6, 15, 0.52},
      {10, 8, 0.55},
      {18.6, 8, 0.6},
  };
  bool IsShoot;
  bool IsPreShoot;

private:
  ctre::phoenix::motorcontrol::can::TalonFX m_shooterMotorRight{ID_MOTOR_SHOOTER_RIGHT};
  ctre::phoenix::motorcontrol::can::TalonFX m_shooterMotorLeft{ID_MOTOR_SHOOTER_LEFT};
};
