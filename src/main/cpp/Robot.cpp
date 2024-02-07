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

    m_motor775.ConfigFactoryDefault(); 

    m_motor775.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake); 

    m_motor775.EnableVoltageCompensation(true);
    m_motor775.ConfigVoltageCompSaturation(12); 

    m_motor775.ConfigClosedloopRamp(0.5);

    m_state = 0;

}
void Robot::TeleopPeriodic() {
  
frc::SmartDashboard::PutBoolean("Sensor", m_infraSensor.Get());
frc::SmartDashboard::PutNumber("Coeff",m_coeff);

switch (m_state) {
  case 0:

    if (m_joystickRight.GetRawButton(1))
    {
      m_coeff=0.2;
      frc::SmartDashboard::PutBoolean("Trigger",true);

      if (not m_infraSensor.Get())
      {
        m_state = 1;
      }
    }

    else 
    {
      m_coeff=0.0;
      frc::SmartDashboard::PutBoolean("Trigger",false);
    }

  break;

  case 1 ://initialisation timmer
  m_timer = 25;
  m_state ++;

  break;

  case 2://freinage, la note avance encore un peu après avoir été captée
    m_timer -=1; 
    if (m_timer<=0)
    {
      m_coeff=0.0;
      m_state ++;
    }
  break;

  case 3://shoot
    if (m_joystickRight.GetRawButton(1))
    {
      m_coeff=1.0;
      m_timer=25;
      m_state ++;
    }

  case 4 ://delai pour que la note ait le temps de sortir de l'intake
    if (m_timer<=0)
    {
      m_state=0;
    }
  break;
}

m_motor775.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,m_coeff);

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