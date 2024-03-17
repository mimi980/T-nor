// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix/motorcontrol/can/TalonFX.h>
#include "lib/Pid.h"
#include "frc/Encoder.h"
#include "Constants.h"

class Climber : public frc2::SubsystemBase
{
public:
  Climber();
  void SetClimberMotor(double speed);
  void SetSetpoint(double setpoint);
  double GetEncoder();
  void Periodic() override;

private:
  ctre::phoenix::motorcontrol::can::TalonFX m_climberMotorRight{ID_MOTOR_CLIMBER_RIGHT};
  ctre::phoenix::motorcontrol::can::TalonFX m_climberMotorLeft{ID_MOTOR_CLIMBER_LEFT};
  Pid m_pidClimber{0.0, CLIMBER_PID_P, CLIMBER_PID_I, CLIMBER_PID_D};
  frc::Encoder m_climberEncoder{ID_ENCODER_CLIMBER_A, ID_ENCODER_CLIMBER_B};
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
