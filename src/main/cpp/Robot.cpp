// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

void Robot::RobotInit() {}
void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {

    m_MotorLeft.ConfigFactoryDefault(); 
    m_MotorRight.ConfigFactoryDefault();


    m_MotorLeft.ConfigSupplyCurrentLimit(ctre::phoenix::motorcontrol::SupplyCurrentLimitConfiguration(true, 40, 40, 0)); 
    m_MotorRight.ConfigSupplyCurrentLimit(ctre::phoenix::motorcontrol::SupplyCurrentLimitConfiguration(true, 40, 40, 0));

    m_MotorRight.SetInverted(true);
    m_MotorLeft.SetInverted(false);


    m_MotorLeft.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake); 
    m_MotorRight.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);

    m_MotorLeft.EnableVoltageCompensation(true);
    m_MotorRight.EnableVoltageCompensation(true);


    m_MotorRight.ConfigVoltageCompSaturation(12); 
    m_MotorLeft.ConfigVoltageCompSaturation(12);

    m_MotorLeft.ConfigClosedloopRamp(0.5);

    m_miniNeo.RestoreFactoryDefaults();
    m_miniNeo.EnableVoltageCompensation(12);
    m_miniNeo.SetSmartCurrentLimit(20);

}
void Robot::TeleopPeriodic() {

  if (m_Jostick.GetRawButton(3)) //aspiration
  {
    m_MotorLeft.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,0.2);
    m_MotorRight.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,0.2);
    m_miniNeo.Set(0.2);
  }

  if (m_Jostick.GetRawButton(4))//shoot
  {
    m_MotorLeft.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,1);
    m_MotorRight.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,1);
    m_miniNeo.Set(1);
  }

  m_MotorLeft.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,0);
  m_MotorRight.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,0);
  m_miniNeo.Set(0);


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
