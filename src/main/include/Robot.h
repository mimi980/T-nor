// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include "Camera.h"
#include <iostream>
#include "rev/CANSparkMax.h"
#include "Pid.h"
#include "frc/Encoder.h"
#include "frc/Joystick.h"
#include "frc/smartdashboard/SmartDashboard.h"

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

  int Signe(double a);

private:
  Camera camera;
  rev::CANSparkMax m_motorLeft{1, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_motorLeft2{2, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_motorRight{3, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_motorRight2{4, rev::CANSparkMax::MotorType::kBrushless};

  frc::Encoder m_encoderRight{0, 1};
  frc::Encoder m_encoderLeft{2, 3};

  Pid m_pid{0.0, 0.0, 0.0, 0.0};
  double m_error;
  double m_speed;
  double m_setpoint;

  frc::Joystick m_joystick{0};
};
