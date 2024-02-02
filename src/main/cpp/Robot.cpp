// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <ctre/Phoenix/motorcontrol/can/TalonFX.h>
#include <frc/Joystick.h>

void Robot::RobotInit() {}
void Robot::RobotPeriodic() {

}

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


    // m_MotorLeft.Follow(m_MotorRight); 
    //frc::SmartDashboard::PutNumber("coeff droit",0.1);
    //frc::SmartDashboard::PutNumber("coeff gauche",0.1);
    frc::SmartDashboard::PutNumber("coeff",0.1);
    frc::SmartDashboard::PutNumber("Percent",0.0);
    frc::SmartDashboard::PutNumber("speed",0.0);
    m_coeff=0.6;
    

    

}
void Robot::TeleopPeriodic() {

  
  m_percent=m_joystickRight.GetY();
  frc::SmartDashboard::PutNumber("Percent",-m_percent);
  frc::SmartDashboard::PutNumber("speed",m_MotorLeft.GetSensorCollection().GetIntegratedSensorVelocity());

  //m_coeff= (m_joystickRight.GetRawAxis(3) +1 )/2;

 if (m_joystickRight.GetRawButtonPressed(4) and m_sate == true)
 {
  m_coeff+=0.01;
  m_sate = false;
 }

 else if (m_joystickRight.GetRawButtonPressed(3) and m_sate == true)
 {
  m_coeff-=0.01;
  m_sate = false;
 }
 
 if (m_joystickRight.GetRawButtonReleased(3) or m_joystickRight.GetRawButtonReleased(4))
 {
  m_sate=true;
 }

 if (m_joystickRight.GetRawButton(1)){
    //m_coeff_droit=frc::SmartDashboard::GetNumber("coeff droit",0.1);
    //m_coeff_gauche=frc::SmartDashboard::GetNumber("coeff gauche",0.1);
    m_moteur=m_coeff;
 }

 else{
  m_coeff_droit=0.0;
  m_coeff_gauche=0.0;
  m_moteur=0.0;
 }
  
  
  frc::SmartDashboard::PutNumber("coeff",m_coeff);
  m_MotorRight.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,m_moteur);
  m_MotorLeft.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,m_moteur);
  



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
