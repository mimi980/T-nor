// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include <ctre/Phoenix/motorcontrol/can/TalonFX.h>
#include <rev/CANSparkMax.h>
#include <frc/Joystick.h>
#include "frc/smartdashboard/SmartDashboard.h"

class Robot : public frc::TimedRobot {
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

  ctre::phoenix::motorcontrol::can::TalonFX m_MotorRight{1};
  ctre::phoenix::motorcontrol::can::TalonFX m_MotorLeft{2};
  ctre::phoenix::motorcontrol::can::TalonFX m_miniNeo{3};
  frc::Joystick m_Jostick{0};

  double m_speedShoot;
  double m_speedAspiration;
  double m_speedCatch;
  double m_encoder;
  int m_count;

  enum State{
    Aspiration,
    Acceleration,
    Shoot,
    End
  };

  State m_state;
};
