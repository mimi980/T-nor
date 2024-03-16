// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

void Robot::RobotInit() {}
void Robot::RobotPeriodic()
{
  frc2::CommandScheduler::GetInstance().Run();
}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit()
{
  frc::SmartDashboard::SmartDashboard::PutNumber("Shooter Speed", 0.0);
  frc::SmartDashboard::SmartDashboard::PutNumber("m_setpoint", 0.0);
}
void Robot::TeleopPeriodic()
{
  m_robotContainer.m_planetary.m_planetaryPid.m_setpoint = frc::SmartDashboard::SmartDashboard::GetNumber("m_setpoint", 0.0);
  m_robotContainer.m_shooter.shooter_speed = frc::SmartDashboard::SmartDashboard::GetNumber("Shooter Speed", 0.0);
  if (m_robotContainer.m_joystickLeft.GetRawButtonPressed(1))
  {
    m_robotContainer.m_intake.IsIntaked ? m_robotContainer.m_intake.IsIntaked = false : m_robotContainer.m_intake.IsIntaked = true;
  }
  if (m_robotContainer.m_joystickRight.GetRawButtonPressed(2))
  {
    m_robotContainer.m_shooter.IsPreShoot ? m_robotContainer.m_shooter.IsPreShoot = false : m_robotContainer.m_shooter.IsPreShoot = true;
  }
  std::cout << m_robotContainer.m_feeder.GetFeederInfraSensorValue() << std::endl;
}

void Robot::DisabledInit()
{
}

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
