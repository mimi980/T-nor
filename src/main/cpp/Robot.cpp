// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

int Robot::Signe(double a)
{
  if (a > 0)
  {
    return 1;
  }
  else if (a < 0)
  {
    return -1;
  }
  else
  {
    return 0;
  }
}
void Robot::RobotInit() {}
void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit()
{
  m_motorLeft.RestoreFactoryDefaults();
  m_motorLeft2.RestoreFactoryDefaults();
  m_motorRight.RestoreFactoryDefaults();
  m_motorRight2.RestoreFactoryDefaults();

  m_motorLeft.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_motorLeft2.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_motorRight.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_motorRight2.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

  m_motorLeft.SetInverted(true);
  m_motorLeft2.SetInverted(true);
  m_motorRight.SetInverted(false);
  m_motorRight2.SetInverted(false);

  m_motorLeft2.Follow(m_motorLeft);
  m_motorRight2.Follow(m_motorRight);

  m_motorLeft.SetSmartCurrentLimit(40);
  m_motorLeft2.SetSmartCurrentLimit(40);
  m_motorRight.SetSmartCurrentLimit(40);
  m_motorRight2.SetSmartCurrentLimit(40);

  m_motorLeft.SetOpenLoopRampRate(0.5);
  m_motorLeft2.SetOpenLoopRampRate(0.5);
  m_motorRight.SetOpenLoopRampRate(0.5);
  m_motorRight2.SetOpenLoopRampRate(0.5);

  m_encoderLeft.Reset();
  m_encoderRight.Reset();

  m_encoderLeft.SetDistancePerPulse(1.0 / 2048.0);
  m_encoderRight.SetDistancePerPulse(1.0 / 2048.0);

  frc::SmartDashboard::PutNumber("P", 0.0);
  frc::SmartDashboard::PutNumber("I", 0.0);
  frc::SmartDashboard::PutNumber("D", 0.0);
}
void Robot::TeleopPeriodic()
{
  m_pid.SetGains(frc::SmartDashboard::GetNumber("P", 0.0), frc::SmartDashboard::GetNumber("I", 0.0), frc::SmartDashboard::GetNumber("D", 0.0));
  m_error = camera.GetHorizontalError();
  m_speed = NABS(NCLAMP(-0.3, m_pid.Calculate(m_error), 0.3));
  m_setpoint = 0.0;
  frc::SmartDashboard::PutNumber("Error", m_error);
  frc::SmartDashboard::PutNumber("speed", m_speed);
  frc::SmartDashboard::PutNumber("setpoint", m_setpoint);
  std::cout << "Id: " << camera.getAprilId() << std::endl;
  std::cout << "distance: " << camera.GetDistance() << std::endl;

  if (m_joystick.GetRawButton(1))
  {
    m_motorLeft.Set(Signe(m_error) * m_speed);
    m_motorRight.Set(-Signe(m_error) * m_speed);
  }
  else
  {
    m_motorLeft.Set(m_joystick.GetZ() * 0.1);
    m_motorRight.Set(-m_joystick.GetZ() * 0.1);
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
