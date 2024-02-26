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
  m_FeederMotor.ConfigFactoryDefault();

  m_MotorLeft.ConfigSupplyCurrentLimit(ctre::phoenix::motorcontrol::SupplyCurrentLimitConfiguration(true, 40, 40, 0));
  m_MotorRight.ConfigSupplyCurrentLimit(ctre::phoenix::motorcontrol::SupplyCurrentLimitConfiguration(true, 40, 40, 0));
  m_FeederMotor.ConfigSupplyCurrentLimit(ctre::phoenix::motorcontrol::SupplyCurrentLimitConfiguration(true, 40, 40, 0));

  m_MotorRight.SetInverted(false);
  m_MotorLeft.SetInverted(false);
  m_FeederMotor.SetInverted(true);
  m_Pivot.SetInverted(true);

  m_MotorLeft.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
  m_MotorRight.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
  m_FeederMotor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);

  m_MotorLeft.EnableVoltageCompensation(true);
  m_MotorRight.EnableVoltageCompensation(true);
  m_FeederMotor.EnableVoltageCompensation(true);

  m_MotorLeft.ConfigClosedloopRamp(0.1);
  m_MotorRight.ConfigClosedloopRamp(0.1);
  m_FeederMotor.ConfigClosedloopRamp(0.1);

  m_MotorRight.ConfigVoltageCompSaturation(12);
  m_MotorLeft.ConfigVoltageCompSaturation(12);
  m_FeederMotor.ConfigVoltageCompSaturation(12);

  frc::SmartDashboard::PutNumber("speedShooter", 0.65);
  frc::SmartDashboard::PutNumber("speedAspiration", -0.2);
  frc::SmartDashboard::PutNumber("speedCatch", 0.6);
  frc::SmartDashboard::PutNumber("coef", 0.04);

  m_count = 0;
  m_state = State::End;

  m_pid.Reset();
  m_encoder.SetDistancePerPulse(((1.0 / 2048.0) / 4.5) * 360.0);

  frc::SmartDashboard::PutNumber("P", 0.0);
  frc::SmartDashboard::PutNumber("I", 0.0);
  frc::SmartDashboard::PutNumber("D", 0.0);

  m_csv.open("/home/lvuser/", true);

  m_csv.setItem(0, "current", 5, &m_current);
  m_csv.setItem(1, "speed", 5, &m_speed);
  m_csv.setItem(2, "encoder", 5, &m_position);
}
void Robot::TeleopPeriodic()
{
  ///////CSV//////
  m_csv.write();
  m_current = m_Pivot.GetOutputCurrent();

  //////pid///////////////////////////////////////////
  m_position = -m_encoder.GetDistance();
  m_speed = m_pid.Calculate(m_position);
  m_pid.SetSetpoint(-m_Jostick_Right.GetY() * 180.0);
  frc::SmartDashboard::PutNumber("setpoint", -m_Jostick_Right.GetY() * 180.0);
  frc::SmartDashboard::PutNumber("encoder", m_position);
  frc::SmartDashboard::PutNumber("speed", m_speed);
  m_pid.SetGains(frc::SmartDashboard::GetNumber("P", 0.0), frc::SmartDashboard::GetNumber("I", 0.0), frc::SmartDashboard::GetNumber("D", 0.0));
  m_Pivot.Set(NCLAMP(-0.9, m_speed, 0.9));
  frc::SmartDashboard::PutNumber("current", m_current);

  ////////////////////////////////////////////////////////////////////////

  m_coeff = frc::SmartDashboard::GetNumber("coef", 0.04);

  m_speedShootHigh = (m_speedShoot * 1.5 - (0.5 * m_speedCatch * 0.26)) / 1.5 - m_coeff;
  // m_speedShootHigh = m_speedShoot + 0.06;

  m_speedShoot = frc::SmartDashboard::GetNumber("speedShooter", 0.65);
  frc::SmartDashboard::PutNumber("HighspeedShooter", m_speedShootHigh);
  m_speedAspiration = frc::SmartDashboard::GetNumber("speedAspiration", -0.2);
  m_speedCatch = frc::SmartDashboard::GetNumber("speedCatch", 0.6);

  if (m_Jostick_Right.GetRawButton(1)) // shoot
  {
    m_MotorLeft.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, m_speedShoot);
    m_MotorRight.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, m_speedShootHigh);
    if (m_Jostick_Right.GetRawButton(2))
    {
      m_FeederMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, m_speedCatch);
    }
  }
  else if (m_Jostick_Right.GetRawButton(3)) // aspiration
  {
    m_MotorLeft.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, m_speedAspiration);
    m_MotorRight.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, m_speedAspiration);
    m_FeederMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, m_speedAspiration);
  }
  else
  {

    m_MotorLeft.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
    m_MotorRight.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
    m_FeederMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
  }

  // if (m_Jostick_Right.GetRawButtonPressed(1))
  // {
  //   m_state=State::Aspiration;
  //   m_count=0;
  // }
  //   m_count++;
  //   switch (m_state)
  //   {
  //   case State::Aspiration:
  //     m_MotorLeft.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,m_speedAspiration);
  //     m_MotorRight.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,m_speedAspiration);
  //     m_FeederMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,m_speedAspiration);
  //     if (m_count>20)
  //     {
  //       m_count=0;
  //       m_FeederMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,0.0);
  //       m_state=State::Acceleration;
  //     }
  //     break;
  //   case State::Acceleration:
  //    m_MotorLeft.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,m_speedShoot);
  //    m_MotorRight.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,m_speedShoot);
  //     if (m_count>140)
  //     {
  //       m_count=0;
  //       m_state=State::Shoot;
  //     }
  //     break;
  //   case State::Shoot:
  //     m_FeederMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,m_speedCatch);
  //     if(m_count>10)
  //     {
  //       m_count=0;
  //       m_state=State::End;
  //     }
  //     break;
  //   case State::End:
  //     m_MotorLeft.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,0.0);
  //     m_MotorRight.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,0.0);
  //     m_FeederMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,0.0);

  //   default:
  //     break;
  //   }
}

void Robot::DisabledInit()
{
  m_csv.close();
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
