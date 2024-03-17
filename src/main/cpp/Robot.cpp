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
}
void Robot::TeleopPeriodic()
{
  if (m_robotContainer.m_joystickLeft.GetRawButtonPressed(1))
  {
    m_robotContainer.m_intake.IsIntaked ? m_robotContainer.m_intake.IsIntaked = false : m_robotContainer.m_intake.IsIntaked = true;
  }
  if (m_robotContainer.m_joystickRight.GetRawButtonPressed(2))
  {
    m_robotContainer.m_shooter.IsPreShoot ? m_robotContainer.m_shooter.IsPreShoot = false : m_robotContainer.m_shooter.IsPreShoot = true;
  }

  if (m_robotContainer.m_joystickLeft.GetRawButton(2))
  {
    m_robotContainer.m_drivetrain.drive_auto = true;
  }
  else
  {
    m_robotContainer.m_drivetrain.drive_auto = false;
  }
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
