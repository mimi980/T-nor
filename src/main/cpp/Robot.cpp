// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

void Robot::RobotInit() {}
void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  m_Gros.RestoreFactoryDefaults();
  m_Gros.SetSmartCurrentLimit(40);
  m_Gros.EnableVoltageCompensation(12);
  m_Gros.SetInverted(true);
  m_Gros.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
}
void Robot::TeleopPeriodic() 
{
  m_Gros.Set(m_Joystick.GetY());
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
