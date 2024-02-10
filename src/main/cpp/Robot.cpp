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

    m_state = State::off;

}
void Robot::TeleopPeriodic() {

frc::SmartDashboard::PutBoolean("Sensor", m_infraSensor.Get());
frc::SmartDashboard::PutNumber("Coeff",m_coeff);

switch (m_state) {
  case State::off : //motor_off

    m_coeff = 0;

    if (m_joystickRight.GetRawButton(1))
    {
      m_state = State::take;
    }
  break;

  case State::take ://motor_on

    m_coeff = -0.3;

    if (!m_joystickRight.GetRawButton(1))
    {
      m_state = State::off;
    }

    if (!m_infraSensor.Get())
    {
      m_state = State::detected_forward;
    }

  break;

  case State::detected_forward:
    if (m_infraSensor.Get())
    {
      m_state = State::not_detected_backward;
    }

  break;

  case State::not_detected_backward:

        m_coeff = 0.1;

        if (!m_infraSensor.Get())
        {
          m_state = State::detected_stop;
        }

  break;

  case State::detected_stop:

    m_coeff = 0.0;
    if (!m_joystickRight.GetRawButton(1))
    {
      m_state = State::wait;
    }

  break;

  case State::wait://wait_for_pressed
  
    if (m_joystickRight.GetRawButton(1))
    {
      m_state = State::shoot;
    }
  break;

  case State::shoot ://motor_on_delay

    m_coeff = -0.6;

    if (!m_joystickRight.GetRawButton(1))
    {
      m_state=State::off;
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