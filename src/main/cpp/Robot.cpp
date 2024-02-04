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


    m_MotorLeft.ConfigSupplyCurrentLimit(ctre::phoenix::motorcontrol::SupplyCurrentLimitConfiguration(true, 45, 45, 0)); 
    m_MotorRight.ConfigSupplyCurrentLimit(ctre::phoenix::motorcontrol::SupplyCurrentLimitConfiguration(true, 45, 45, 0));

    m_MotorRight.SetInverted(false);
    m_MotorLeft.SetInverted(false);


    m_MotorLeft.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake); 
    m_MotorRight.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);

    m_MotorLeft.EnableVoltageCompensation(true);
    m_MotorRight.EnableVoltageCompensation(true);


    m_MotorRight.ConfigVoltageCompSaturation(12); 
    m_MotorLeft.ConfigVoltageCompSaturation(12);


    m_miniNeo.RestoreFactoryDefaults();
    m_miniNeo.EnableVoltageCompensation(12);
    m_miniNeo.SetSmartCurrentLimit(30);
    m_miniNeo.SetClosedLoopRampRate(0.2);

    

    frc::SmartDashboard::PutNumber("speedShooter",1.0);
    frc::SmartDashboard::PutNumber("speedAspiration",-0.3);
    frc::SmartDashboard::PutNumber("speedCatch",1.0);

}
void Robot::TeleopPeriodic() {

  m_speedShoot=frc::SmartDashboard::GetNumber("speedShooter",1.0);
  m_speedAspiration=frc::SmartDashboard::GetNumber("speedAspiration",-0.3);
  m_speedCatch=frc::SmartDashboard::GetNumber("speedCatch",1.0);


  double m_speedShoootR=m_speedShoot*7.0/8.0;


 if (m_Jostick.GetRawButton(1)) //shoot
    {
      m_MotorLeft.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,m_speedShoot);
      m_MotorRight.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,m_speedShoootR);
      if (m_Jostick.GetRawButton(2))
      {
        m_miniNeo.Set(m_speedCatch);
      }
    }
  else if (m_Jostick.GetRawButton(3)) //aspiration
  {
    m_MotorLeft.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,m_speedAspiration);
    m_MotorRight.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,m_speedAspiration);
    m_miniNeo.Set(m_speedAspiration);
  }
  else
  {

    m_MotorLeft.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,0.0);
    m_MotorRight.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,0.0);
    m_miniNeo.Set(0.0);
  }
  
  


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
