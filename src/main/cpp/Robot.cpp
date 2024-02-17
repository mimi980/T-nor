// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <time.h>

void Robot::RobotInit() {}
void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {

  m_Motor.ConfigFactoryDefault();
  m_Motor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
  m_Motor.EnableVoltageCompensation(true);
  m_Motor.ConfigVoltageCompSaturation(12);
  m_Motor.ConfigClosedloopRamp(0.5);

}
void Robot::TeleopPeriodic() {

m_coeff = m_joystickRight.GetRawAxis(0)*0.8;
m_Motor.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::PercentOutput,m_coeff);

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