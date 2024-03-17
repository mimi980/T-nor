// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

void Robot::RobotInit() {}
void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit()
{

  m_MotorLeft.ConfigFactoryDefault();
  m_MotorRight.ConfigFactoryDefault();

  m_MotorLeft.EnableVoltageCompensation(true);
  m_MotorRight.EnableVoltageCompensation(true);

  m_MotorLeft.ConfigVoltageCompSaturation(10.0);
  m_MotorRight.ConfigVoltageCompSaturation(10.0);

  m_MotorLeft.ConfigOpenloopRamp(0.1);
  m_MotorRight.ConfigOpenloopRamp(0.1);

  m_MotorLeft.ConfigSupplyCurrentLimit(ctre::phoenix::motorcontrol::SupplyCurrentLimitConfiguration(true, 40, 40, 0.0));
  m_MotorRight.ConfigSupplyCurrentLimit(ctre::phoenix::motorcontrol::SupplyCurrentLimitConfiguration(true, 40, 40, 0.0));

  m_MotorLeft.SetInverted(true);
  m_MotorRight.SetInverted(false);

  m_MotorLeft.Follow(m_MotorRight);

  m_MotorLeft.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
  m_MotorRight.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);

  frc::SmartDashboard::PutNumber("kp", 0.05);
  frc::SmartDashboard::PutNumber("ki", 0.0);
  frc::SmartDashboard::PutNumber("kd", 0.04);
  frc::SmartDashboard::PutNumber("m_setpoint", 0.0);
  m_Encoder.Reset();
  m_Encoder.SetDistancePerPulse(((1.0 / 2048.0) / 4.5) * 360.0);
}
void Robot::TeleopPeriodic()
{
  m_pid.SetGains(frc::SmartDashboard::GetNumber("kp", 0.05), frc::SmartDashboard::GetNumber("ki", 0.0), frc::SmartDashboard::GetNumber("kd", 0.04));
  m_mesure = m_Encoder.GetDistance();
  m_setpoint = frc::SmartDashboard::GetNumber("m_setpoint", 0.0);
  m_pid.SetSetpoint(m_setpoint);
  m_output = m_pid.Calculate(m_mesure);
  frc::SmartDashboard::PutNumber("output", m_output);
  frc::SmartDashboard::PutNumber("mesure", m_mesure);
  frc::SmartDashboard::PutNumber("error", m_pid.m_error);

  if (m_Joystick.GetRawButton(1))
  {
    m_MotorRight.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, std::clamp(m_output, -0.2, 0.2));
  }
  else
  {
    m_MotorRight.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
  }
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
