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
  m_Gros.RestoreFactoryDefaults();
  m_Gros.SetSmartCurrentLimit(40);
  m_Gros.EnableVoltageCompensation(12);
  m_Gros.SetInverted(true);
  m_Gros.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  frc::SmartDashboard::PutNumber("kp", 0.1);
  frc::SmartDashboard::PutNumber("ki", 0.0);
  frc::SmartDashboard::PutNumber("kd", 0.0);
  m_Encoder.Reset();
  m_Encoder.SetDistancePerPulse(1.0 / 2048.0 * 360.0);
}
void Robot::TeleopPeriodic()
{
  m_pid.SetGains(frc::SmartDashboard::GetNumber("kp", 0.1), frc::SmartDashboard::GetNumber("ki", 0.0), frc::SmartDashboard::GetNumber("kd", 0.0));
  m_mesure = m_Encoder.GetDistance();
  m_setpoint = m_Joystick.GetY() * 360.0;
  m_pid.SetSetpoint(m_setpoint);
  m_output = m_pid.Calculate(m_mesure);
  frc::SmartDashboard::PutNumber("setpoint", m_setpoint);
  frc::SmartDashboard::PutNumber("output", m_output);
  frc::SmartDashboard::PutNumber("mesure", m_mesure);
  frc::SmartDashboard::PutNumber("error", m_pid.m_error);
  // m_Gros.Set(std::clamp(m_output, -0.4, 0.4));
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
