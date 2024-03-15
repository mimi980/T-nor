// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include <frc/Joystick.h>
#include "rev/CANSparkMax.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include "Pid.h"
#include <frc/Encoder.h>

class Robot : public frc::TimedRobot
{
public:
  void RobotInit() override;
  void RobotPeriodic() override;

  void AutonomousInit() override;
  void AutonomousPeriodic() override;

  void TeleopInit() override;
  void TeleopPeriodic() override;

  void DisabledInit() override;
  void DisabledPeriodic() override;

  void TestInit() override;
  void TestPeriodic() override;

  void SimulationInit() override;
  void SimulationPeriodic() override;

private:
  Pid m_pid{0.0, 0.1, 0.1, 0.1}; // m_setpoint, m_kp, m_ki, m_kd
  frc::Joystick m_Joystick{0};
  rev::CANSparkMax m_Gros{10, rev::CANSparkMax::CANSparkLowLevel::MotorType::kBrushless};
  frc::Encoder m_Encoder{6, 7};
  double m_setpoint;
  double m_output;
  double m_mesure;
};
